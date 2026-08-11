from ctypes import alignment
from turtle import fd

from vis_nav_game import Player, Action, Phase
import pygame
import tempfile
import cv2
import os
import numpy as np

# ── Pipeline imports ──────────────────────────────────────────────────────────
from build_graph import (
    DINOv2Descriptor,
    extract_keyframes_uniform,
    detect_turns,
    merge_and_sort,
    deduplicate,
    encode_geometric,
    build_faiss_index,
    build_pose_graph,
    infer_action_from_flow,
    save_artifacts,
)

from maze_navigator import (
    MazeNavigator,
    label_actions_from_commands,
)

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(BASE_DIR)

FRAME_DIR = r"C:\Users\Pedro\vis_nav_player\source\data\exploration_data\images"
GRAPH_DIR = r"C:\Users\Pedro\vis_nav_player\source\data\exploration_data\fusion_output"
GRAPH_PATH = os.path.join(GRAPH_DIR, "maze_graph.pkl")
FAISS_PATH = os.path.join(GRAPH_DIR, "keyframe_index.faiss")
META_PATH = os.path.join(GRAPH_DIR, "keyframe_meta.pkl")

AUTO             = True

STUCK_CHECK_EVERY    = 15
STUCK_DIFF_THRESHOLD = 4.0
STUCK_TRIGGER        = 2
STEP             = 10
ROTATION_THRESH  = 15.0
MIN_GAP          = 10
HAMMING          = 8
LOOP_THRESHOLD   = 0.85
LOOP_MIN_GAP     = 50
DINO_SIZE        = "s"

# ── PD controller ─────────────────────────────────────────────────────────────
PD_KP     = 1.5
PD_KD     = 0.8
PD_T_TURN = 0.12

# ── Route bias ────────────────────────────────────────────────────────────────
GRAPH_BIAS = 0.25

# ── confirm_step rate ─────────────────────────────────────────────────────────
CONFIRM_STEP_INTERVAL = 8

ACTION_STR_TO_ENUM = {
    "forward":    Action.FORWARD,
    "turn_left":  Action.LEFT,
    "turn_right": Action.RIGHT,
    "backward":   Action.BACKWARD,
    "stop":       Action.CHECKIN,
}


class KeyboardPlayerPyGame(Player):
    def __init__(self):
        self.nav_frame_idx = 0
        self.cached_localize = None
        self.cached_direction_scores = None
        self.cached_alignment = None
        self.encoder      = DINOv2Descriptor(model_size=DINO_SIZE)
        self.graph        = None
        self.index        = None
        self.descriptors  = None
        self.keyframes    = []
        self._frame_count = 0

        self._goal_img_path     = None
        self._goal_pending      = True
        self._target_images_set = None

        # Navigation state machine
        self.state = "IDLE"
        self.navigator = None
        self.alignment_history = []
        self.consecutive_low_alignment = 0

        self._goal_confirm_count = 0
        self._GOAL_CONFIRM_NEEDED = 5

        # Heading tracking
        self._prev_fpv_gray = None
        self._cumulative_yaw = 0.0

        self.fpv = None
        self.prev_fpv = None
        self._last_suggestion = "IDLE"
        self._last_suggestion_detail = ""
        self.last_act = Action.IDLE
        self.screen = None
        self.keymap = None

        # PD controller state
        self._prev_e_k = 0.0

        # Stuck detection
        self._stuck_snapshot    = None
        self._stuck_frame_count = 0
        self._stuck_consec      = 0
        self._radar_sweep_dir   = "turn_right"

        super(KeyboardPlayerPyGame, self).__init__()

    def reset(self):
        self.fpv = None
        self.last_act = Action.IDLE
        self.screen = None

        pygame.init()

        self.keymap = {
            pygame.K_LEFT: Action.LEFT,
            pygame.K_RIGHT: Action.RIGHT,
            pygame.K_UP: Action.FORWARD,
            pygame.K_DOWN: Action.BACKWARD,
            pygame.K_SPACE: Action.CHECKIN,
            pygame.K_ESCAPE: Action.QUIT
        }
        print("KeyboardPlayerPyGame reset complete. Ready to play!")
        return self.last_act

    # ── Pre_exploration ───────────────────────────────────────────────────────
    def pre_exploration(self):
        print("\n" + "=" * 60)
        print("  PRE-RECORDED DATASET MODE  (skipping live exploration)")
        print(f"  Frame dir : {FRAME_DIR}")
        print(f"  Graph dir : {GRAPH_DIR}")
        K = self.get_camera_intrinsic_matrix()
        print(f'K={K}')
        print("=" * 60)

        os.makedirs(GRAPH_DIR, exist_ok=True)

        if (os.path.exists(GRAPH_PATH) and
                os.path.exists(FAISS_PATH) and
                os.path.exists(META_PATH)):
            print("\n[pre_exploration]  Existing map found — loading from disk…")
            self._load_existing_map()
        else:
            print("\n[pre_exploration]  No map found — building from dataset…")
            self._build_map_from_dataset()

        print("[pre_exploration]  Done. Exploration phase will idle.\n")
        super(KeyboardPlayerPyGame, self).pre_exploration()

    # ── Pre_navigation ────────────────────────────────────────────────────────
    def pre_navigation(self) -> None:
        super(KeyboardPlayerPyGame, self).pre_navigation()
        print("\n[pre_navigation]  Setting up navigator…")

        if self.graph is None or self.index is None:
            print("[pre_navigation]  Map missing — rebuilding now…")
            self._build_map_from_dataset()

        self.navigator = MazeNavigator(
            graph=self.graph,
            index=self.index,
            keyframes=self.keyframes,
            descriptors=self.descriptors,
            encoder=self.encoder,
        )
        print("[pre_navigation]  Navigator ready — starting run.\n")

        target_imgs = self.get_target_images()
        self._target_images_set = target_imgs

        self._goal_pending = True

        target_imgs = self._target_images_set or self.get_target_images()

        if target_imgs is not None and len(target_imgs) > 0:
            self._goal_img_path = None
            self._goal_pending = True
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            print(f"[pre_navigation] Using image at {best_path}, node={best_node} (combined={best_score:.4f})")
            self._goal_img_path = best_path
            self._goal_node_direct = best_node
        else:
            print("[pre_navigation] Target images not available — preserving existing goal state.")
            self._goal_pending = (self._goal_img_path is None)

    def set_target_images(self, images):
        super(KeyboardPlayerPyGame, self).set_target_images(images)
        self._target_images_set = images
        self.show_target_images()

    # ── see ───────────────────────────────────────────────────────────────────
    def see(self, fpv):
        if fpv is None or len(fpv.shape) < 3:
            self.prev_fpv = None
            return

        self.fpv = fpv

        if self.screen is None:
            h, w, _ = fpv.shape
            if not pygame.get_init():
                pygame.init()
            self.screen = pygame.display.set_mode((w, h))

        def convert_opencv_img_to_pygame(opencv_image):
            opencv_image = opencv_image[:, :, ::-1]
            shape = opencv_image.shape[1::-1]
            pygame_image = pygame.image.frombuffer(opencv_image.tobytes(), shape, 'RGB')
            return pygame_image

        pygame.display.set_caption("KeyboardPlayer:fpv")
        rgb = convert_opencv_img_to_pygame(fpv)
        self.screen.blit(rgb, (0, 0))
        pygame.display.update()

        if not self._goal_pending or self.navigator is None:
            self.prev_fpv = fpv.copy()
            return

        self._frame_count += 1

        if not self._goal_pending and self.navigator is not None and self._frame_count % 10 == 0:
            cur  = self.navigator.current_node
            goal = self.navigator._goal_node
            path = self.navigator.current_path or []
            hops = max(0, len(path) - 1)
            nxt  = self.navigator.next_action() if len(path) >= 2 else "stop"
            print(f"\n{'='*55}")
            print(f"  NODE: {cur} → GOAL: {goal} | HOPS: {hops} | NEXT: {nxt.upper()}")
            print(f"  SUGGEST: {self._last_suggestion.upper():<12} | STATE: {self.state}")
            print(f"{'='*55}")

        target_imgs = self._target_images_set or self.get_target_images() or []
        if len(target_imgs) == 0:
            self.prev_fpv = fpv.copy()
            return

        if self._goal_img_path is None:
            print("[see] Scoring target images (all 4 views combined)…")
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            if best_path is None:
                self.prev_fpv = fpv.copy()
                return
            self._goal_img_path = best_path
            self._goal_node_direct = best_node
            print(f"[see] Goal image: {best_path}, goal node: {best_node} (combined={best_score:.4f})")

        print("[see] Localizing…")
        self.navigator.localize_robust(self.fpv)
        print("[see] Planning path to goal…")
        if hasattr(self, '_goal_node_direct') and self._goal_node_direct is not None:
            self.navigator.set_goal(self._goal_node_direct)
        else:
            self.navigator.set_goal_by_image(self._goal_img_path)

        if self.navigator._goal_node is not None and self.navigator.current_path:
            self._goal_pending = False
            print("[see] ✓ Goal initialized — act() will now execute.")
        else:
            print("[see] ✗ Planning failed — will retry next frame.")

        self.prev_fpv = fpv.copy()

    # ── act ───────────────────────────────────────────────────────────────────
    def act(self):
        if self._state and self._state[1] == Phase.EXPLORATION:
            frame_num = self._state[2]
            if frame_num == 0:
                self.last_act = Action.IDLE
                return Action.QUIT

        q_pressed = False
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("[act] Window closed by user.")
                    return Action.QUIT
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        q_pressed = True
                    elif event.key in self.keymap:
                        self.last_act |= self.keymap[event.key]
                elif event.type == pygame.KEYUP:
                    if event.key in self.keymap:
                        self.last_act ^= self.keymap[event.key]
        except pygame.error as e:
            print(f"[act] Pygame error: {e}")

        if self.last_act == Action.QUIT:
            return Action.QUIT

        if self._goal_pending or self.navigator is None or self.navigator._goal_node is None:
            return Action.IDLE

        if AUTO:
            final_action = self._execute_navigation()
            if final_action is None:
                return self.last_act
            if final_action == Action.QUIT:
                print("[act] Navigator has requested QUIT. Goal reached!")
                return Action.QUIT
            return final_action

        if q_pressed and self.fpv is not None:
            self._manual_update_and_display()

        return self.last_act

    # -------- ALL HELPER FUNCTIONS BELOW THIS LINE --------

    def _load_existing_map(self):
        import faiss, pickle
        from build_graph import load_graph
        self.graph = load_graph(GRAPH_PATH)
        with open(META_PATH, "rb") as f:
            meta = pickle.load(f)
        self.keyframes   = meta["keyframes"]
        self.descriptors = meta["descriptors"]
        self.index       = faiss.read_index(FAISS_PATH)
        lc = sum(1 for _, _, d in self.graph.edges(data=True)
                 if d.get("edge_type") == "loop_closure") if self.graph else 0
        print(f"  Keyframes  : {len(self.keyframes)}")
        print(f"  Nodes      : {self.graph.number_of_nodes()}")
        print(f"  Edges      : {self.graph.number_of_edges()}")
        print(f"  Loop edges : {lc}")

    def _build_map_from_dataset(self):
        print("\n── Stage 1 & 2: Keyframe extraction")
        uniform_kfs = extract_keyframes_uniform(FRAME_DIR, step=STEP)
        turn_kfs    = detect_turns(FRAME_DIR, rotation_threshold=ROTATION_THRESH)

        print("\n── Stage 3: Merge")
        candidates = merge_and_sort(uniform_kfs + turn_kfs)

        print("\n── Stage 4: Deduplication")
        self.keyframes = deduplicate(candidates, hamming_threshold=HAMMING)
        if not self.keyframes:
            raise RuntimeError(f"No keyframes after dedup — check FRAME_DIR={FRAME_DIR}")

        print("\n── Stage 7: DINOv2 encode")
        paths            = [kf["path"] for kf in self.keyframes]
        self.descriptors = encode_geometric(self.encoder, paths, batch_size=32)

        print("\n── Stage 8: FAISS index")
        self.index = build_faiss_index(self.descriptors)

        print("\n── Stage 9: Pose graph")
        self.graph = build_pose_graph(
            self.keyframes, self.descriptors, self.index,
            loop_threshold=LOOP_THRESHOLD,
            loop_min_gap=LOOP_MIN_GAP,
            infer_actions=True,
        )

        print("\n── Stage 10: Save artefacts")
        save_artifacts(
            output_dir=GRAPH_DIR,
            keyframes=self.keyframes,
            descriptors=self.descriptors,
            index=self.index,
            graph=self.graph,
        )

    def _score_target_images(self, target_imgs):
        view_names = ["front", "left", "back", "right"]
        view_encodings = []
        view_paths = []

        for i, img in enumerate(target_imgs[:4]):
            fd, tmp = tempfile.mkstemp(suffix=f"_{view_names[i]}.jpg")
            os.close(fd)
            try:
                ok = cv2.imwrite(tmp, img)
                if not ok:
                    print(f"  [{view_names[i]}] failed to write temp image")
                    continue
                q = self.encoder.encode(tmp).reshape(1, -1).astype(np.float32)
                view_encodings.append((view_names[i], q, tmp))
                view_paths.append(tmp)
            except Exception as e:
                print(f"  [{view_names[i]}] encoding error: {e}")
                if os.path.exists(tmp):
                    os.remove(tmp)

        if not view_encodings:
            return None, -1.0, None

        node_votes = {}
        for vname, q, _ in view_encodings:
            scores, idxs = self.index.search(q, 10)
            for sc, idx in zip(scores[0], idxs[0]):
                if idx >= 0:
                    idx = int(idx)
                    node_votes[idx] = node_votes.get(idx, 0.0) + float(sc)
            best_idx = int(idxs[0][0])
            best_sc  = float(scores[0][0])
            print(f"  [{vname:5s}] best match: node {best_idx} (sim={best_sc:.4f})")

        if not node_votes:
            return None, -1.0, None

        best_node     = max(node_votes, key=node_votes.get)
        best_combined = node_votes[best_node]

        n_views_agree = sum(
            1 for vname, q, _ in view_encodings
            for _, idx in zip(*self.index.search(q, 10))
            if int(idx[0]) == best_node
        )

        print(f"  [combined] Best goal node: {best_node} (combined={best_combined:.4f})")

        best_view_path  = None
        best_view_score = -1.0

        for vname, q, tmp_path in view_encodings:
            if best_node < len(self.descriptors):
                desc = self.descriptors[best_node].reshape(1, -1).astype(np.float32)
                sim  = float(np.dot(q, desc.T)[0, 0])
                if sim > best_view_score:
                    best_view_score = sim
                    best_view_path  = tmp_path

        for vname, q, tmp_path in view_encodings:
            if tmp_path != best_view_path and os.path.exists(tmp_path):
                os.remove(tmp_path)

        return best_view_path, best_combined, best_node

    def show_target_images(self):
        targets = self.get_target_images()
        if targets is None or len(targets) <= 0:
            return
        hor1 = cv2.hconcat(targets[:2])
        hor2 = cv2.hconcat(targets[2:])
        concat_img = cv2.vconcat([hor1, hor2])

        w, h = concat_img.shape[:2]
        color = (0, 0, 0)

        concat_img = cv2.line(concat_img, (int(h/2), 0), (int(h/2), w), color, 2)
        concat_img = cv2.line(concat_img, (0, int(w/2)), (h, int(w/2)), color, 2)

        w_offset = 25
        h_offset = 10
        font     = cv2.FONT_HERSHEY_SIMPLEX
        line     = cv2.LINE_AA
        size     = 0.75
        stroke   = 1

        cv2.putText(concat_img, 'Front View',  (h_offset, w_offset),                           font, size, color, stroke, line)
        cv2.putText(concat_img, 'Left View',   (int(h/2) + h_offset, w_offset),                font, size, color, stroke, line)
        cv2.putText(concat_img, 'Back View',   (h_offset, int(w/2) + w_offset),                font, size, color, stroke, line)
        cv2.putText(concat_img, 'Right View',  (int(h/2) + h_offset, int(w/2) + w_offset),     font, size, color, stroke, line)

        cv2.imshow(f'KeyboardPlayer:target_images', concat_img)
        cv2.imwrite('target.jpg', concat_img)
        cv2.waitKey(1)
        print("Displayed target images")

    # ── _execute_navigation ───────────────────────────────────────────────────
    def _execute_navigation(self):
        self.nav_frame_idx += 1

        if self.navigator is None or self.fpv is None:
            return Action.IDLE

        act = Action.IDLE

        if AUTO:
            if self.state == "IDLE":
                return self._act_idle()
            if self.state == "MOVING":
                return self._act_moving()
            if self.state == "GOAL_REACHED":
                print("[execute] Goal reached!")
                return Action.CHECKIN
        else:
            if self.state == "IDLE":
                act = self._act_idle()
            elif self.state == "MOVING":
                act = self._act_moving()
            if self.state == "GOAL_REACHED":
                print("[execute] Goal reached!")
                return Action.CHECKIN

        return self.last_act

    def _small_fpv(self, fpv, scale=0.6):
        h, w = fpv.shape[:2]
        return cv2.resize(fpv, (int(w * scale), int(h * scale)))

    def _get_sorted_frame_list(self):
        if not hasattr(self, '_sorted_frames_cache') or self._sorted_frames_cache is None:
            from pathlib import Path
            frames = [
                f for f in Path(FRAME_DIR).iterdir()
                if f.suffix.lower() in {".jpg", ".jpeg", ".png"}
            ]
            frames.sort(key=lambda f: int(f.stem) if f.stem.isdigit() else f.stem)
            self._sorted_frames_cache = frames
            print(f"[cache] Loaded {len(frames)} frame paths from {FRAME_DIR}")
        return self._sorted_frames_cache

    def _is_facing_corner(self, prev_img, curr_img, divergence_threshold=2.0):
        if prev_img is None or curr_img is None:
            return False
        try:
            prev_gray = cv2.cvtColor(prev_img, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
            if prev_gray.shape != curr_gray.shape:
                prev_gray = cv2.resize(prev_gray, (curr_gray.shape[1], curr_gray.shape[0]))
            flow = cv2.calcOpticalFlowFarneback(
                prev_gray, curr_gray, None,
                pyr_scale=0.5, levels=3, winsize=15,
                iterations=3, poly_n=5, poly_sigma=1.2, flags=0
            )
            _, w = prev_gray.shape
            mid  = w // 2
            left_mean  = float(np.mean(flow[:, :mid, 0]))
            right_mean = float(np.mean(flow[:, mid:, 0]))
            divergence = right_mean - left_mean
            if divergence > divergence_threshold:
                print(f"[{divergence:.2f} Flow] CORNER/WALL DETECTED via Optical Flow!")
                return True
            return False
        except Exception as e:
            print(f"[flow] Corner detector failed: {e}")
            return False

    def _is_stuck(self) -> bool:
        if self.fpv is None:
            return False
        self._stuck_frame_count += 1
        if self._stuck_frame_count < STUCK_CHECK_EVERY:
            return False
        self._stuck_frame_count = 0
        small = cv2.resize(self.fpv, (80, 60))
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        if self._stuck_snapshot is None:
            self._stuck_snapshot = gray
            return False
        diff = float(np.mean(np.abs(gray - self._stuck_snapshot)))
        self._stuck_snapshot = gray
        print(f"[stuck] Frame diff={diff:.2f} (threshold={STUCK_DIFF_THRESHOLD})")
        if diff < STUCK_DIFF_THRESHOLD:
            self._stuck_consec += 1
            print(f"[stuck] Stagnant {self._stuck_consec}/{STUCK_TRIGGER}")
        else:
            self._stuck_consec = 0
        return self._stuck_consec >= STUCK_TRIGGER

    # ── _act_idle ─────────────────────────────────────────────────────────────
    def _act_idle(self):
        if self.navigator._goal_node is None:
            print("[idle] ⚠️  No goal set.")
            return Action.IDLE

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # 1. Localize current position
        if self.nav_frame_idx % 3 == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.localize_robust(small_fpv)

        current_node, confidence = self.cached_localize
        print(f"[idle] Current node: {current_node} (confidence: {confidence:.3f})")

        if confidence > 0.70:
            self.navigator.current_node = current_node
            self.navigator.set_goal(self.navigator._goal_node)
            print(f"[idle] ✓ Replanned from {current_node} (confidence={confidence:.3f})")

        # 2. Check path is valid
        if not self.navigator.current_path or len(self.navigator.current_path) < 2:
            print(f"[idle] ⚠️  Path planning failed. "
                  f"current_path={self.navigator.current_path}, "
                  f"current_node={current_node}, "
                  f"goal={self.navigator._goal_node}")
            return Action.IDLE

        # 3. Scan directions (cached every 3 frames)
        if self.nav_frame_idx % 3 == 0 or self.cached_direction_scores is None:
            self.cached_direction_scores = self.navigator.scan_directions(small_fpv)

        direction_scores = self.cached_direction_scores
        print(f"[idle] Direction scores: {direction_scores}")

        # Initialize stateful safeguards
        if not hasattr(self, '_escape_attempts'):
            self._escape_attempts = 0
            self._blind_frames = 0
            self._radar_sweep_dir = "turn_right"

        # Gather environment data
        graph_action = self.navigator.next_action()
        is_corner = self._is_facing_corner(self.prev_fpv, self.fpv)
        is_blind = self.navigator.detect_dead_end(direction_scores, threshold=0.4)
        out_edges = list(self.navigator.G.successors(self.navigator.current_node))
        is_junction = len(out_edges) > 1 and self.navigator.detect_junction(direction_scores, threshold=0.6)

        # Update blind hysteresis
        if is_blind:
            self._blind_frames += 1
        else:
            self._blind_frames = 0
            self._escape_attempts = 0

        # ESCALATION PROTOCOL: Persistent failure
        if self._escape_attempts > 10:
            print("[override] 🚨 Persistent failure. Forcing relocalization and cache wipe.")
            self.cached_localize = None
            self.cached_direction_scores = None
            self.cached_alignment = None
            self._escape_attempts = 0
            self._blind_frames = 0
            return Action.IDLE

        # PRIORITY 1: GEOMETRY (We are physically hitting a wall/corner)
        if is_corner:
            self._escape_attempts += 1
            print(f"[override] 🔄 Physics Corner! Trusting map '{graph_action}' (Attempt {self._escape_attempts})")
            self.prev_fpv = self.fpv.copy() if self.fpv is not None else None
            if graph_action == "turn_right":
                self._last_suggestion = "turn_right"
                return Action.RIGHT
            elif graph_action == "turn_left":
                self._last_suggestion = "turn_left"
                return Action.LEFT
            else:
                self._last_suggestion = self._radar_sweep_dir
                return ACTION_STR_TO_ENUM.get(self._radar_sweep_dir, Action.RIGHT)

        # PRIORITY 2: LOW OBSERVABILITY (Dead End / Blindness / use_radar)
        if self._blind_frames >= 2 or graph_action == "use_radar":
            self._escape_attempts += 1
            print(f"[override] Blind/radar. Attempt {self._escape_attempts}")

            try:
                from PIL import Image
                small = self._small_fpv(self.fpv, scale=0.6)
                h, w  = small.shape[:2]
                pil   = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
                left_blocked  = self.navigator._check_if_blocked(
                    pil.crop((0,      h // 6, w // 2, 5 * h // 6)))
                right_blocked = self.navigator._check_if_blocked(
                    pil.crop((w // 2, h // 6, w,      5 * h // 6)))
            except Exception:
                left_blocked = right_blocked = False

            if left_blocked and not right_blocked:
                turn_dir = "turn_right"
            elif right_blocked and not left_blocked:
                turn_dir = "turn_left"
            else:
                turn_dir = self._radar_sweep_dir
                self._radar_sweep_dir = "turn_left" if self._radar_sweep_dir == "turn_right" else "turn_right"

            print(f"[override] Physical wall check → {turn_dir} "
                  f"(left_blocked={left_blocked}, right_blocked={right_blocked})")
            self._last_suggestion = turn_dir
            return ACTION_STR_TO_ENUM[turn_dir]

        # PRIORITY 3: TOPOLOGICAL CHOICE POINT (Junction)
        if is_junction:
            print(f"[pipeline] 🚦 Junction confirmed. Proceeding with Graph Bias: {graph_action}")

        # 4. Filter blocked directions and apply graph-intent bias
        raw_scores = {
            d: info.copy() for d, info in direction_scores.items()
        }

        graph_dir_idle = {
            "forward":    "front",
            "turn_left":  "left",
            "turn_right": "right",
            "backward":   "back",
            "use_radar":  None,
        }.get(graph_action, "front")

        if graph_dir_idle and graph_dir_idle in raw_scores:
            old = raw_scores[graph_dir_idle]["score"]
            raw_scores[graph_dir_idle]["score"] = min(1.0, old + GRAPH_BIAS)
            print(f"[idle]  Graph bias: {graph_dir_idle} {old:.3f} → "
                  f"{raw_scores[graph_dir_idle]['score']:.3f} "
                  f"(graph says '{graph_action}')")

        # Explicit backward handling
        if graph_action == "backward":
            back_info = raw_scores.get("back", {})
            if back_info and not back_info.get("blocked", False):
                print(f"[idle]  Graph wants backward — executing directly.")
                self.state             = "MOVING"
                self.alignment_history = []
                self.consecutive_low_alignment = 0
                self._last_suggestion  = "backward"
                return ACTION_STR_TO_ENUM["backward"]

        feasible = {
            d: info for d, info in raw_scores.items()
            if not info.get("blocked", False)
        }

        if not feasible:
            print("[idle] ⚠️  All directions blocked — staying IDLE")
            return Action.IDLE

        # If all scores are too similar, use wall check instead of DINO noise
        all_scores = [info.get("score", 0) for info in raw_scores.values()]
        score_spread = max(all_scores) - min(all_scores) if all_scores else 0

        if score_spread < 0.05:
            print(f"[idle] Scores too similar (spread={score_spread:.3f}) — using wall check")
            try:
                from PIL import Image
                small = self._small_fpv(self.fpv, scale=0.6)
                h, w  = small.shape[:2]
                pil   = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
                left_blocked  = self.navigator._check_if_blocked(
                    pil.crop((0,      h // 6, w // 2, 5 * h // 6)))
                right_blocked = self.navigator._check_if_blocked(
                    pil.crop((w // 2, h // 6, w,      5 * h // 6)))
                front_blocked = self.navigator._check_if_blocked(
                    pil.crop((w // 6, h // 6, 5 * w // 6, 5 * h // 6)))
            except Exception:
                left_blocked = right_blocked = front_blocked = False

            print(f"[idle] Wall check: front={front_blocked}, left={left_blocked}, right={right_blocked}")

            if not front_blocked:
                best_action = "forward"
            elif left_blocked and not right_blocked:
                best_action = "turn_right"
            elif right_blocked and not left_blocked:
                best_action = "turn_left"
            else:
                best_action = graph_action if graph_action in ("turn_left", "turn_right") else "forward"

            self.state = "MOVING"
            self.alignment_history = []
            self.consecutive_low_alignment = 0
            self._last_suggestion = best_action
            return ACTION_STR_TO_ENUM.get(best_action, Action.FORWARD)

        # PD controller
        r_score = raw_scores.get("right", {}).get("score", 0.0) if "right" in feasible else 0.0
        l_score = raw_scores.get("left",  {}).get("score", 0.0) if "left"  in feasible else 0.0
        f_score = raw_scores.get("front", {}).get("score", 0.0) if "front" in feasible else 0.0

        e_k  = r_score - l_score
        u_k  = PD_KP * e_k + PD_KD * (e_k - self._prev_e_k)
        self._prev_e_k = e_k

        front_dominates = f_score > max(r_score, l_score) + 0.10

        if front_dominates:
            best_action   = "forward"
            best_direction = "front"
        elif u_k > PD_T_TURN:
            best_action    = "turn_right"
            best_direction = "right"
        elif u_k < -PD_T_TURN:
            best_action    = "turn_left"
            best_direction = "left"
        else:
            best_action    = "forward"
            best_direction = "front"

        best_score    = raw_scores.get(best_direction, {}).get("score", 0.0)
        best_distance = raw_scores.get(best_direction, {}).get("distance", 0)
        blocked_dirs  = [d for d, info in direction_scores.items() if info.get("blocked")]

        print(f"[idle] PD u={u_k:+.3f} e={e_k:+.3f} → {best_action} "
              f"(score={best_score:.3f}, distance={best_distance})"
              f"{' | blocked: ' + ','.join(blocked_dirs) if blocked_dirs else ''}")

        # 5. Transition to MOVING
        self.state = "MOVING"
        self.alignment_history = []
        self.consecutive_low_alignment = 0
        self._last_suggestion = best_action

        return ACTION_STR_TO_ENUM.get(best_action, Action.FORWARD)

    # ── _act_moving ───────────────────────────────────────────────────────────
    def _act_moving(self):
        """
        MOVING State: execute the chosen direction while monitoring alignment.
        """
        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # ── Stuck detection ────────────────────────────────────────────────────
        if self._is_stuck():
            try:
                from PIL import Image
                small = self._small_fpv(self.fpv, scale=0.6)
                h, w  = small.shape[:2]
                pil   = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
                left_blocked  = self.navigator._check_if_blocked(
                    pil.crop((0,      h // 6, w // 2, 5 * h // 6)))
                right_blocked = self.navigator._check_if_blocked(
                    pil.crop((w // 2, h // 6, w,      5 * h // 6)))
            except Exception:
                left_blocked = right_blocked = False

            if left_blocked and not right_blocked:
                turn_dir = "turn_right"
            elif right_blocked and not left_blocked:
                turn_dir = "turn_left"
            else:
                turn_dir = self._radar_sweep_dir
                self._radar_sweep_dir = "turn_left" if self._radar_sweep_dir == "turn_right" else "turn_right"

            print(f"[stuck] Escape → {turn_dir} "
                  f"(left_blocked={left_blocked}, right_blocked={right_blocked})")
            self._last_suggestion = turn_dir
            return ACTION_STR_TO_ENUM[turn_dir]

        # 1. Get alignment (cached)
        if self.nav_frame_idx % 3 == 0 or self.cached_alignment is None:
            self.cached_alignment = self.navigator.get_alignment_scores(small_fpv)

        alignment  = self.cached_alignment
        primary    = alignment.get("primary", 0.0)
        confidence = alignment.get("route_confidence", 0.0)

        self.alignment_history.append(primary)
        if len(self.alignment_history) > 10:
            self.alignment_history.pop(0)

        print(f"[moving] Alignment: primary={primary:.3f}, confidence={confidence:.3f}")

        if self.nav_frame_idx % CONFIRM_STEP_INTERVAL == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.confirm_step(small_fpv)

        node_after_step, step_score = self.cached_localize
        print(f"[moving] Step confirmed: node={node_after_step}, score={step_score:.3f}")

        # 3. Goal reached?
        if self.navigator.current_node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                print(f"[moving] ✓ GOAL CONFIRMED ({self._goal_confirm_count} consecutive matches)")
                return Action.CHECKIN
            else:
                print(f"[moving] Near goal — confirmation {self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED}")
        else:
            self._goal_confirm_count = 0

        # 4. Alignment drop guard
        if primary < 0.3:
            self.consecutive_low_alignment += 1
            if self.consecutive_low_alignment > 3:
                print("[moving] ⚠️  ALIGNMENT DROP detected (3+ frames)")
                self.state = "IDLE"
                self.cached_direction_scores = None
                self.cached_alignment        = None
                return Action.IDLE
        else:
            self.consecutive_low_alignment = 0

        # 5. Scan directions (cached)
        if self.nav_frame_idx % 3 == 0 or self.cached_direction_scores is None:
            self.cached_direction_scores = self.navigator.scan_directions(small_fpv) or {}

        # Better node ahead?
        if "front" in self.cached_direction_scores:
            front_score    = self.cached_direction_scores["front"].get("score", 0)
            front_distance = self.cached_direction_scores["front"].get("distance", 0)
            if front_distance > 1 and front_score > 0.8:
                print(f"[moving] Better forward node at distance {front_distance}")
                self.state = "IDLE"
                self.cached_direction_scores = None
                self.cached_alignment        = None
                return Action.IDLE

        # Junction?
        if self.navigator.detect_junction(self.cached_direction_scores, threshold=0.6):
            print("[moving] ⚠️  JUNCTION detected while moving")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_alignment        = None
            return Action.IDLE

        # Optical flow corner check
        if self._is_facing_corner(self.prev_fpv, self.fpv):
            print("[moving] 🔄 Corner detected! Deferring to IDLE pipeline.")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_alignment        = None
            self.prev_fpv = self.fpv.copy() if self.fpv is not None else None
            return Action.IDLE

        # Dead end?
        if self.navigator.detect_dead_end(self.cached_direction_scores, threshold=0.4):
            print("[moving] ⚠️  DEAD END detected while moving")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_alignment        = None
            return Action.IDLE

        # ── HAZARDS CLEARED: NOW APPLY BIAS & STEER ───────────────────────────
        direction_scores = {
            d: info.copy() for d, info in self.cached_direction_scores.items()
        }

        graph_action = self.navigator.next_action()
        graph_dir = {
            "forward":    "front",
            "turn_left":  "left",
            "turn_right": "right",
            "backward":   "back",
            "use_radar":  None,
        }.get(graph_action, "front")

        if graph_dir and graph_dir in direction_scores:
            old_score = direction_scores[graph_dir]["score"]
            direction_scores[graph_dir]["score"] = min(1.0, old_score + GRAPH_BIAS)
            print(f"[moving] Route bias applied: {graph_dir} "
                  f"{old_score:.3f} → {direction_scores[graph_dir]['score']:.3f} "
                  f"(graph says '{graph_action}')")

        if graph_action == "backward":
            back_info = direction_scores.get("back", {})
            if back_info and not back_info.get("blocked", False):
                print(f"[moving] Graph wants backward — executing directly "
                      f"(score={back_info.get('score', 0):.3f})")
                self._last_suggestion = "backward"
                return ACTION_STR_TO_ENUM["backward"]

        if direction_scores:
            feasible = [
                (d, info) for d, info in direction_scores.items()
                if not info.get("blocked", False)
            ]

            if feasible:
                r_score = next((i["score"] for d, i in feasible if d == "right"), 0.0)
                l_score = next((i["score"] for d, i in feasible if d == "left"),  0.0)
                f_score = next((i["score"] for d, i in feasible if d == "front"), 0.0)

                e_k  = r_score - l_score
                u_k  = PD_KP * e_k + PD_KD * (e_k - self._prev_e_k)
                self._prev_e_k = e_k

                front_dominates = f_score > max(r_score, l_score) + 0.10

                if front_dominates:
                    action_str = "forward"
                elif u_k > PD_T_TURN:
                    action_str = "turn_right"
                elif u_k < -PD_T_TURN:
                    action_str = "turn_left"
                else:
                    action_str = "forward"

                print(f"[moving] PD u={u_k:+.3f} → {action_str}")
            else:
                action_str = "forward"
                print("[moving] All directions blocked — defaulting forward")
        else:
            action_str = "forward"
            print("[moving] No direction data — defaulting forward")

        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)

    # ── _manual_update_and_display ────────────────────────────────────────────
    def _manual_update_and_display(self):
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator._goal_node is None:
            print("[manual] Goal not set yet.")
            return

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        yaw_delta = self._estimate_heading_change(small_fpv)
        self._cumulative_yaw += yaw_delta
        print(f"[manual] Heading: delta={yaw_delta:+.1f}°, cumulative={self._cumulative_yaw:+.1f}°")

        node, score = self.navigator.localize_robust(small_fpv)
        print(f"[manual] Localized → node {node} (score={score:.3f})")

        self.navigator.current_node = node
        self.navigator.set_goal(self.navigator._goal_node)
        path = self.navigator.current_path or []
        hops = max(0, len(path) - 1)

        if node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            print(f"[manual] At goal node! Confirmation {self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED}")
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                print("[manual] ✓ GOAL CONFIRMED — press SPACE to check in!")
        else:
            self._goal_confirm_count = 0

        direction_scores = self.navigator.scan_directions(small_fpv)
        self.cached_direction_scores = direction_scores

        for d, info in direction_scores.items():
            blocked_str = " [BLOCKED]" if info.get("blocked") else ""
            print(f"[manual]   {d:>6}: score={info['score']:.3f} dist={info['distance']}{blocked_str}")

        raw_action = self.navigator.next_action() if hops >= 1 else "stop"
        resolved_action, detail = self._resolve_action(raw_action, small_fpv, direction_scores)
        self._last_suggestion        = resolved_action
        self._last_suggestion_detail = detail

        print(f"[manual] Suggestion: {resolved_action.upper()}")
        print(f"[manual] Scores: {detail}")

        gray = cv2.cvtColor(small_fpv, cv2.COLOR_BGR2GRAY)
        self._prev_fpv_gray = gray

        self.display_next_best_view()

    def _estimate_heading_change(self, fpv: np.ndarray) -> float:
        gray = cv2.cvtColor(fpv, cv2.COLOR_BGR2GRAY)
        if self._prev_fpv_gray is None:
            return 0.0
        try:
            prev = self._prev_fpv_gray
            if prev.shape != gray.shape:
                prev = cv2.resize(prev, (gray.shape[1], gray.shape[0]))
            h, w = gray.shape
            orb = cv2.ORB_create(nfeatures=200)
            kp1, des1 = orb.detectAndCompute(prev, None)
            kp2, des2 = orb.detectAndCompute(gray, None)
            if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
                return 0.0
            bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            if len(matches) < 5:
                return 0.0
            dx_list = []
            for m in matches:
                pt1 = kp1[m.queryIdx].pt
                pt2 = kp2[m.trainIdx].pt
                dx_list.append(pt2[0] - pt1[0])
            median_dx         = float(np.median(dx_list))
            fov_degrees       = 90.0
            degrees_per_pixel = fov_degrees / w
            yaw_delta         = -median_dx * degrees_per_pixel
            return yaw_delta
        except Exception as e:
            print(f"[heading] Estimation failed: {e}")
            return 0.0

    def _resolve_action(self, raw_action: str, small_fpv: np.ndarray,
                        direction_scores: dict = None) -> tuple:
        dir_map = {"front": "forward", "left": "turn_left", "right": "turn_right",
                   "back": "backward"}

        if raw_action == "stop":
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                return "stop", "GOAL CONFIRMED — press SPACE"
            return "stop", "end of path (may need replan)"

        if direction_scores:
            feasible = [
                (d, info) for d, info in direction_scores.items()
                if not info.get("blocked", False)
            ]
            feasible.sort(key=lambda x: -x[1]["score"])

            detail_parts = []
            best_dir = feasible[0][0] if feasible else None
            for d in ["front", "left", "right", "back"]:
                if d in direction_scores:
                    info   = direction_scores[d]
                    marker = ">>>" if d == best_dir else "   "
                    blk    = " [WALL]" if info.get("blocked") else ""
                    detail_parts.append(f"{marker}{d}={info['score']:.2f}{blk}")
            detail = " | ".join(detail_parts)

            if feasible:
                resolved = dir_map.get(best_dir, "forward")
                return resolved, detail

            return "forward", "all directions blocked — defaulting forward"

        return "forward", "no scan data — defaulting forward"

    def _simplify_path(self, path: list) -> list:
        if not path or len(path) < 2:
            return []

        G    = self.navigator.G
        goal = self.navigator._goal_node
        waypoints = []
        MAX_STRAIGHT      = 5
        last_waypoint_hop = 0

        for i in range(1, len(path)):
            node = path[i]
            prev = path[i - 1]
            hops_away = i

            ed          = G.get_edge_data(prev, node)
            edge_action = ed.get("action", "forward") if ed else "forward"
            edge_type   = ed.get("edge_type", "sequential") if ed else "sequential"

            n_successors = len(list(G.successors(node)))

            is_turn     = edge_action in ("turn_left", "turn_right")
            is_loop     = edge_type == "loop_closure" or edge_action == "loop"
            is_junction = n_successors > 2
            is_goal     = node == goal
            hops_since_last = hops_away - last_waypoint_hop

            if is_goal:
                waypoints.append({"node": node, "hops_away": hops_away, "type": "goal",       "label": "GOAL"})
                last_waypoint_hop = hops_away
                break
            elif is_loop:
                waypoints.append({"node": node, "hops_away": hops_away, "type": "loop",       "label": "WARP"})
                last_waypoint_hop = hops_away
            elif is_turn:
                direction = "LEFT" if edge_action == "turn_left" else "RIGHT"
                waypoints.append({"node": node, "hops_away": hops_away, "type": "turn",       "label": f"TURN {direction}"})
                last_waypoint_hop = hops_away
            elif is_junction:
                waypoints.append({"node": node, "hops_away": hops_away, "type": "junction",   "label": "JUNCTION"})
                last_waypoint_hop = hops_away
            elif hops_since_last >= MAX_STRAIGHT:
                remaining = max(0, len(path) - 1 - hops_away)
                waypoints.append({"node": node, "hops_away": hops_away, "type": "checkpoint", "label": f"STRAIGHT ({remaining} left)"})
                last_waypoint_hop = hops_away

            if len(waypoints) >= 5:
                break

        if not waypoints and goal is not None:
            waypoints.append({"node": goal, "hops_away": max(0, len(path) - 1), "type": "goal", "label": "GOAL"})

        return waypoints

    def display_next_best_view(self):
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator.current_node is None or self.navigator._goal_node is None:
            print("[panel] Goal not yet initialized — press Q after navigation starts.")
            return

        FONT = cv2.FONT_HERSHEY_SIMPLEX
        AA   = cv2.LINE_AA
        TW, TH = 260, 195
        PW, PH = 156, 117

        cur_node  = self.navigator.current_node
        goal_node = self.navigator._goal_node
        path      = self.navigator.current_path or []
        hops      = max(0, len(path) - 1)

        next_action = self._last_suggestion if self._last_suggestion else "stop"
        detail_text = getattr(self, '_last_suggestion_detail', '')
        near        = hops <= 5

        panel_w = TW * 3
        bar_h   = 60 if detail_text else 40
        bar     = np.zeros((bar_h, panel_w, 3), dtype=np.uint8)

        if self.state == "GOAL_REACHED":
            bar[:] = (0, 160, 0)
        elif near:
            bar[:] = (0, 0, 160)
        else:
            bar[:] = (50, 35, 15)

        heading_str = f"  hdg={self._cumulative_yaw:+.0f}deg" if self._cumulative_yaw != 0 else ""
        txt = (f"Node {cur_node}  |  Goal {goal_node}"
               f"  |  {hops} hops  |  >> {next_action.upper()}{heading_str}")
        cv2.putText(bar, txt, (8, 22), FONT, 0.48, (255, 255, 255), 1, AA)

        if self.state == "GOAL_REACHED":
            cv2.putText(bar, "GOAL CONFIRMED — PRESS SPACE!",
                        (panel_w - 300, 22), FONT, 0.45, (0, 255, 255), 1, AA)
        elif near:
            confirm_txt = f"NEAR GOAL ({self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED} confirms)"
            cv2.putText(bar, confirm_txt,
                        (panel_w - 340, 22), FONT, 0.45, (0, 255, 255), 1, AA)

        if detail_text:
            cv2.putText(bar, detail_text, (8, 48), FONT, 0.40, (180, 180, 255), 1, AA)

        def thumb(img, label, color, extra=None):
            t = cv2.resize(img, (TW, TH))
            cv2.rectangle(t, (0, 0), (TW-1, TH-1), color, 2)
            cv2.putText(t, label, (6, 22), FONT, 0.55, color, 1, AA)
            if extra:
                cv2.putText(t, extra, (6, 44), FONT, 0.45, (200, 200, 200), 1, AA)
            return t

        fpv_t = thumb(self.fpv, "Live FPV", (255, 255, 255))

        match_img = None
        if cur_node is not None and cur_node < len(self.keyframes):
            match_img = cv2.imread(self.keyframes[cur_node]["path"])
        if match_img is None:
            match_img = np.zeros((TH, TW, 3), dtype=np.uint8)
        match_t = thumb(match_img, f"Match: node {cur_node}", (0, 255, 0))

        tgt_img = None
        if self._goal_img_path and os.path.exists(self._goal_img_path):
            tgt_img = cv2.imread(self._goal_img_path)
        if tgt_img is None:
            tgt_img = np.zeros((TH, TW, 3), dtype=np.uint8)
        tgt_t = thumb(tgt_img, "Goal Image", (0, 140, 255))

        row1 = cv2.hconcat([fpv_t, match_t, tgt_t])

        N_MICRO     = 5
        micro_cells = []

        if len(path) >= 2:
            cur_fidx  = self.keyframes[path[0]]["frame_idx"] if path[0] < len(self.keyframes) else 0
            next_fidx = self.keyframes[path[1]]["frame_idx"] if path[1] < len(self.keyframes) else 0

            edge_data  = self.navigator.G.get_edge_data(path[0], path[1])
            hop_action = ""
            if edge_data:
                hop_action = edge_data.get("action", "forward")
                if hop_action == "loop" or edge_data.get("edge_type") == "loop_closure":
                    hop_action = self._last_suggestion

            if abs(next_fidx - cur_fidx) > 2:
                step_dir = 1 if next_fidx > cur_fidx else -1
                all_intermediate = list(range(cur_fidx + step_dir, next_fidx, step_dir))
                if len(all_intermediate) > N_MICRO + 2:
                    usable  = all_intermediate[2:]
                    indices = [usable[int(i * len(usable) / N_MICRO)] for i in range(N_MICRO)]
                elif len(all_intermediate) > N_MICRO:
                    indices = [all_intermediate[int(i * len(all_intermediate) / N_MICRO)] for i in range(N_MICRO)]
                else:
                    indices = all_intermediate
            else:
                indices = []

            sorted_frames = self._get_sorted_frame_list()
            for i, fidx in enumerate(indices):
                img = None
                if sorted_frames and 0 <= fidx < len(sorted_frames):
                    img = cv2.imread(str(sorted_frames[fidx]))
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))
                cv2.rectangle(img, (0, 0), (PW-1, PH-1), (0, 200, 200), 1)
                progress_pct = int(100 * (i + 1) / max(len(indices), 1))
                label = f"f{fidx} ({progress_pct}%)"
                cv2.putText(img, label, (4, 16), FONT, 0.32, (255, 255, 255), 1, AA)
                if i == 0 and hop_action:
                    cv2.putText(img, hop_action.upper(), (4, PH - 8),
                                FONT, 0.40, (0, 255, 255), 1, AA)
                micro_cells.append(img)

        while len(micro_cells) < N_MICRO:
            micro_cells.append(np.zeros((PH, PW, 3), dtype=np.uint8))

        row2 = cv2.hconcat(micro_cells)
        if row2.shape[1] < panel_w:
            pad  = np.zeros((PH, panel_w - row2.shape[1], 3), dtype=np.uint8)
            row2 = cv2.hconcat([row2, pad])

        row2_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row2_label, "Micro-steps to next node",
                    (6, 14), FONT, 0.38, (0, 200, 200), 1, AA)

        waypoints   = self._simplify_path(path)
        N_WAYPOINTS = 5
        cells       = []

        for p in range(N_WAYPOINTS):
            if p < len(waypoints):
                wp       = waypoints[p]
                node_idx = wp["node"]
                img      = None
                if node_idx < len(self.keyframes):
                    img = cv2.imread(self.keyframes[node_idx]["path"])
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))

                if   wp["type"] == "turn":       border_color = (0, 200, 255)
                elif wp["type"] == "loop":       border_color = (200, 100, 255)
                elif wp["type"] == "goal":       border_color = (0, 255, 0)
                elif wp["type"] == "junction":   border_color = (0, 255, 255)
                elif wp["type"] == "checkpoint": border_color = (180, 180, 180)
                else:                            border_color = (200, 200, 0)

                cv2.rectangle(img, (0, 0), (PW-1, PH-1), border_color, 2)
                cv2.putText(img, f"{wp['hops_away']} hops", (4, 16), FONT, 0.35, (255, 255, 255), 1, AA)
                cv2.putText(img, wp["label"], (4, PH - 8), FONT, 0.38, border_color, 1, AA)
            else:
                img = np.zeros((PH, PW, 3), dtype=np.uint8)
            cells.append(img)

        row3 = cv2.hconcat(cells)
        if row3.shape[1] < panel_w:
            pad  = np.zeros((PH, panel_w - row3.shape[1], 3), dtype=np.uint8)
            row3 = cv2.hconcat([row3, pad])

        row3_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row3_label, "Waypoints (turns, junctions, checkpoints every 5 hops)",
                    (6, 14), FONT, 0.38, (200, 200, 0), 1, AA)

        panel = cv2.vconcat([bar, row1, row2_label, row2, row3_label, row3])
        cv2.imshow("Navigation Panel", panel)
        cv2.waitKey(1)

        print(f"── NAV: {next_action:<12} | Node {cur_node} → {goal_node} | {hops} hops")


if __name__ == "__main__":
    import logging
    logging.basicConfig(filename='vis_nav_player.log', filemode='w', level=logging.INFO,
                        format='%(asctime)s - %(levelname)s: %(message)s', datefmt='%d-%b-%y %H:%M:%S')
    import vis_nav_game as vng
    logging.info(f'player.py is using vis_nav_game {vng.core.__version__}')
    vng.play(the_player=KeyboardPlayerPyGame())
