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
    infer_action,
    save_artifacts,
)

from maze_navigator import (
    MazeNavigator,
    label_actions_from_commands,
)

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(BASE_DIR)

FRAME_DIR  = r"C:\Users\Pedro\vis_nav_player\source\data\exploration_data\exploration_data\images"
GRAPH_DIR  = r"C:\Users\Pedro\vis_nav_player\source\data\exploration_data\fusion_output"
GRAPH_PATH = os.path.join(GRAPH_DIR, "maze_graph.pkl")
FAISS_PATH = os.path.join(GRAPH_DIR, "keyframe_index.faiss")
META_PATH  = os.path.join(GRAPH_DIR, "keyframe_meta.pkl")

AUTO = False  # True = fully autonomous; False = manual with Q-press display

STEP             = 5
ROTATION_THRESH  = 15.0
MIN_GAP          = 10
HAMMING          = 8
LOOP_THRESHOLD   = 0.85
LOOP_MIN_GAP     = 50
DINO_SIZE        = "s"

# ── Navigation thresholds ───────────────────────────────────────────────────
ARRIVAL_THRESHOLD     = 0.82   # cosine sim to declare arrival at next node
LOCALIZE_INTERVAL     = 3      # localize every N nav frames (AUTO moving state)
CONFIRM_STEP_INTERVAL = 8      # confirm step every N nav frames
JUNCTION_DEGREE       = 3      # graph node degree >= this → treat as junction

# ── PD controller (used only in MANUAL mode display) ───────────────────────
PD_KP     = 1.5
PD_KD     = 0.8
PD_T_TURN = 0.12

# ── Route bias (MANUAL mode only) ──────────────────────────────────────────
GRAPH_BIAS = 0.25

ACTION_STR_TO_ENUM = {
    "forward":    Action.FORWARD,
    "turn_left":  Action.LEFT,
    "turn_right": Action.RIGHT,
    "backward":   Action.BACKWARD,
    "stop":       Action.CHECKIN,
}


# ─────────────────────────────────────────────────────────────────────────────
# Offline helper: tag junction nodes by graph degree
# ─────────────────────────────────────────────────────────────────────────────

def _tag_junctions(graph, degree_threshold: int = JUNCTION_DEGREE):
    """
    Mark every node whose undirected degree >= degree_threshold as a junction.
    Done once offline — AUTO mode reads the flag with zero compute cost.
    """
    import networkx as nx
    n_junctions = 0
    for node in graph.nodes:
        is_j = graph.degree(node) >= degree_threshold
        graph.nodes[node]['is_junction'] = is_j
        if is_j:
            n_junctions += 1
    print(f"[junctions] Tagged {n_junctions} / {graph.number_of_nodes()} nodes as junctions")
    return graph


# ─────────────────────────────────────────────────────────────────────────────
# Player
# ─────────────────────────────────────────────────────────────────────────────

class KeyboardPlayerPyGame(Player):
    def __init__(self):
        self.nav_frame_idx   = 0
        self.cached_localize = None

        # cached_direction_scores and cached_alignment are MANUAL mode only
        self.cached_direction_scores = None
        self.cached_alignment        = None

        self.encoder     = DINOv2Descriptor(model_size=DINO_SIZE)
        self.graph       = None
        self.index       = None
        self.descriptors = None
        self.keyframes   = []
        self._frame_count = 0

        self._goal_img_path      = None
        self._goal_pending       = True
        self._target_images_set  = None

        # Navigation state machine
        self.state     = "IDLE"
        self.navigator = None

        # Goal confirmation
        self._goal_confirm_count  = 0
        self._GOAL_CONFIRM_NEEDED = 5

        # Heading tracking (MANUAL mode)
        self._prev_fpv_gray   = None
        self._cumulative_yaw  = 0.0

        self.fpv      = None
        self.prev_fpv = None
        self._last_suggestion        = "IDLE"
        self._last_suggestion_detail = ""
        self.last_act = Action.IDLE
        self.screen   = None
        self.keymap   = None

        # PD controller state (MANUAL mode only)
        self._prev_e_k = 0.0

        # Recovery counters (AUTO mode)
        self._escape_attempts   = 0
        self._blind_frames      = 0
        self._radar_sweep_dir   = "turn_right"
        self._consecutive_low_alignment = 0

        super(KeyboardPlayerPyGame, self).__init__()

    def reset(self):
        self.fpv      = None
        self.last_act = Action.IDLE
        self.screen   = None
        pygame.init()
        self.keymap = {
            pygame.K_LEFT:  Action.LEFT,
            pygame.K_RIGHT: Action.RIGHT,
            pygame.K_UP:    Action.FORWARD,
            pygame.K_DOWN:  Action.BACKWARD,
            pygame.K_SPACE: Action.CHECKIN,
            pygame.K_ESCAPE: Action.QUIT,
        }
        print("KeyboardPlayerPyGame reset complete. Ready to play!")
        return self.last_act

    # ── Pre-exploration ────────────────────────────────────────────────────────
    def pre_exploration(self):
        print("\n" + "=" * 60)
        print("  PRE-RECORDED DATASET MODE")
        print(f"  Frame dir : {FRAME_DIR}")
        print(f"  Graph dir : {GRAPH_DIR}")
        K = self.get_camera_intrinsic_matrix()
        print(f"  K={K}")
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

        print("[pre_exploration]  Done.\n")
        super(KeyboardPlayerPyGame, self).pre_exploration()

    # ── Pre-navigation ─────────────────────────────────────────────────────────
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
        print("[pre_navigation]  Navigator ready.\n")

        target_imgs = self.get_target_images()
        self._target_images_set = target_imgs
        self._goal_pending = True

        if target_imgs is not None and len(target_imgs) > 0:
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            print(f"[pre_navigation] Goal node={best_node} (score={best_score:.4f})")
            self._goal_img_path   = best_path
            self._goal_node_direct = best_node
        else:
            print("[pre_navigation] Target images not yet available — deferring to see().")
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
            opencv_image  = opencv_image[:, :, ::-1]
            shape         = opencv_image.shape[1::-1]
            return pygame.image.frombuffer(opencv_image.tobytes(), shape, 'RGB')

        pygame.display.set_caption("KeyboardPlayer:fpv")
        self.screen.blit(convert_opencv_img_to_pygame(fpv), (0, 0))
        pygame.display.update()

        if not self._goal_pending or self.navigator is None:
            self.prev_fpv = fpv.copy()
            return

        self._frame_count += 1

        target_imgs = self._target_images_set or self.get_target_images() or []
        if len(target_imgs) == 0:
            self.prev_fpv = fpv.copy()
            return

        if self._goal_img_path is None:
            print("[see] Scoring target images…")
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            if best_path is None:
                self.prev_fpv = fpv.copy()
                return
            self._goal_img_path    = best_path
            self._goal_node_direct = best_node
            print(f"[see] Goal image: {best_path}, goal node: {best_node} (score={best_score:.4f})")

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
        # Skip exploration phase
        if self._state and self._state[1] == Phase.EXPLORATION:
            frame_num = self._state[2]
            if frame_num == 0:
                self.last_act = Action.IDLE
                return Action.QUIT

        # Pygame events
        q_pressed = False
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
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
                print("[act] Goal reached!")
                return Action.QUIT
            return final_action

        # MANUAL mode: Q press triggers display update
        if q_pressed and self.fpv is not None:
            self._manual_update_and_display()

        return self.last_act

    # ── _execute_navigation ────────────────────────────────────────────────────
    def _execute_navigation(self):
        self.nav_frame_idx += 1

        if self.navigator is None or self.fpv is None:
            return Action.IDLE

        if AUTO:
            if self.state == "IDLE":
                return self._act_idle()
            if self.state == "MOVING":
                return self._act_moving()
            if self.state == "GOAL_REACHED":
                print("[execute] Goal reached!")
                return Action.CHECKIN
        else:
            act = Action.IDLE
            if self.state == "IDLE":
                act = self._act_idle()
            elif self.state == "MOVING":
                act = self._act_moving()
            if self.state == "GOAL_REACHED":
                return Action.CHECKIN
            return act

        return self.last_act

    # ── _act_idle (AUTO mode) ─────────────────────────────────────────────────
    def _act_idle(self):
        """
        IDLE state — AUTO mode only.

        Decision pipeline:
          1. Localize with Bayesian filter (every LOCALIZE_INTERVAL frames)
          2. Replan if confidence is high enough
          3. Check for corners via optical flow
          4. Read next_action() directly from graph edge
          5. If junction: trust path completely, skip all visual checks
          6. If loop-closure edge: resolve via keyframe optical flow
          7. Transition to MOVING

        NO scan_directions() calls here.
        scan_directions() encodes crops which don't match full-frame
        keyframe descriptors → it produces wrong matches and was the
        primary cause of the robot picking incorrect paths.
        """
        if self.navigator._goal_node is None:
            return Action.IDLE

        # 1. Localize (Bayesian filter, full frame)
        if self.nav_frame_idx % LOCALIZE_INTERVAL == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.localize_robust(self.fpv)

        current_node, confidence = self.cached_localize
        print(f"[idle] Node={current_node}  conf={confidence:.3f}")

        if confidence > 0.70:
            self.navigator.current_node = current_node
            self.navigator.set_goal(self.navigator._goal_node)

        # 2. Validate path
        if not self.navigator.current_path or len(self.navigator.current_path) < 2:
            print("[idle] ⚠️  No valid path — staying IDLE.")
            return Action.IDLE

        # 3. Graph action — zero compute, just read the edge
        graph_action = self.navigator.next_action()

        # 4. Junction check — from offline-tagged node degree, not visual scan
        cur_node  = self.navigator.current_path[0]
        is_junction = self.navigator.G.nodes[cur_node].get('is_junction', False)

        if is_junction:
            # At a junction: path is ground truth, no visual ambiguity check
            print(f"[idle] JUNCTION node {cur_node} → trusting path: {graph_action}")
            if graph_action == "use_radar":
                graph_action = self._resolve_loop_closure_action()
            self._last_suggestion = graph_action
            self.state = "MOVING"
            return ACTION_STR_TO_ENUM.get(graph_action, Action.FORWARD)

        # 5. Corner check via optical flow (cheap, no DINOv2)
        if self._is_facing_corner(self.prev_fpv, self.fpv):
            self._escape_attempts += 1
            print(f"[idle] 🔄 Corner detected → trusting map: {graph_action} "
                  f"(attempt {self._escape_attempts})")
            if self._escape_attempts > 10:
                self._escape_attempts = 0
                self.cached_localize  = None
                return Action.IDLE
            if graph_action in ("turn_left", "turn_right", "use_radar"):
                if graph_action == "use_radar":
                    graph_action = self._resolve_loop_closure_action()
                self._last_suggestion = graph_action
                self.state = "MOVING"
                return ACTION_STR_TO_ENUM.get(graph_action, Action.FORWARD)
        else:
            self._escape_attempts = 0

        # 6. Resolve loop-closure edges
        if graph_action == "use_radar":
            graph_action = self._resolve_loop_closure_action()

        # 7. Transition to MOVING
        print(f"[idle] → {graph_action}")
        self._last_suggestion = graph_action
        self.state = "MOVING"
        self._consecutive_low_alignment = 0
        return ACTION_STR_TO_ENUM.get(graph_action, Action.FORWARD)

    # ── _act_moving (AUTO mode) ────────────────────────────────────────────────
    def _act_moving(self):
        """
        MOVING state — AUTO mode only.

        Every CONFIRM_STEP_INTERVAL frames:
          1. Encode full FPV (single DINOv2 call)
          2. Bayesian filter update via confirm_step()
          3. Dot-product arrival check vs precomputed target descriptor
          4. If arrived → advance path, transition to IDLE

        Corner check via optical flow (no DINOv2).
        NO scan_directions() calls.
        """
        graph_action = self.navigator.next_action()
        if not self.navigator.current_path or len(self.navigator.current_path) < 2:
            self.state = "IDLE"
            return Action.IDLE

        # Corner check — optical flow, no DINOv2
        if self._is_facing_corner(self.prev_fpv, self.fpv):
            print("[moving] 🔄 Corner detected — back to IDLE")
            self.state = "IDLE"
            self.cached_localize = None
            self.prev_fpv = self.fpv.copy() if self.fpv is not None else None
            return Action.IDLE

        # Confirm step every N frames
        if self.nav_frame_idx % CONFIRM_STEP_INTERVAL == 0 or self.cached_localize is None:
            node_after, step_score = self.navigator.confirm_step(self.fpv)
            self.cached_localize   = (node_after, step_score)
            print(f"[moving] confirm_step → node={node_after}  score={step_score:.3f}")

            # Goal check
            if self.navigator.current_node == self.navigator._goal_node:
                self._goal_confirm_count += 1
                print(f"[moving] Near goal — {self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED}")
                if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                    self.state = "GOAL_REACHED"
                    return Action.CHECKIN
            else:
                self._goal_confirm_count = 0

            # Direct arrival check: full frame dot-product vs target descriptor
            if len(self.navigator.current_path) >= 2:
                next_node    = self.navigator.current_path[1]
                arrival_sim  = self.navigator.arrival_score(self.fpv, next_node)
                print(f"[moving] arrival_score={arrival_sim:.3f} (threshold={ARRIVAL_THRESHOLD})")

                if arrival_sim >= ARRIVAL_THRESHOLD:
                    # Confirmed arrival
                    self.navigator.current_path = self.navigator.current_path[1:]
                    self.navigator.current_node = next_node
                    self.navigator.bayesian.force_node(next_node, certainty=0.92)
                    print(f"[moving] ✓ Arrived at node {next_node}")
                    self.state = "IDLE"
                    self.cached_localize = None
                    return Action.IDLE

        # Keep executing the planned action
        self._last_suggestion = graph_action
        return ACTION_STR_TO_ENUM.get(graph_action, Action.FORWARD)

    # ── _resolve_loop_closure_action ──────────────────────────────────────────
    def _resolve_loop_closure_action(self) -> str:
        """
        For loop-closure edges (action='loop' / 'use_radar'), infer direction
        from optical flow between the two keyframe images stored on disk.

        This uses offline data only — no live DINOv2 call needed.
        Falls back to 'forward' if images are unavailable.
        """
        if not self.navigator.current_path or len(self.navigator.current_path) < 2:
            return "forward"

        src  = self.navigator.current_path[0]
        dest = self.navigator.current_path[1]

        src_path  = self.navigator.keyframes[src].get("path") if src < len(self.navigator.keyframes) else None
        dest_path = self.navigator.keyframes[dest].get("path") if dest < len(self.navigator.keyframes) else None

        if src_path and dest_path and os.path.exists(src_path) and os.path.exists(dest_path):
            try:
                action = infer_action(src_path, dest_path)
                print(f"[loop] Resolved loop-closure {src}→{dest}: {action}")
                return action
            except Exception as e:
                print(f"[loop] infer_action failed: {e}")

        return "forward"

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _small_fpv(self, fpv, scale=0.6):
        h, w = fpv.shape[:2]
        return cv2.resize(fpv, (int(w * scale), int(h * scale)))

    def _is_facing_corner(self, prev_img, curr_img, divergence_threshold=2.0):
        """Optical flow corner/wall detection. No DINOv2 involved."""
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
                iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
            )
            _, w = prev_gray.shape
            mid  = w // 2
            divergence = float(np.mean(flow[:, mid:, 0])) - float(np.mean(flow[:, :mid, 0]))
            if divergence > divergence_threshold:
                print(f"[flow] Corner/wall detected (div={divergence:.2f})")
                return True
            return False
        except Exception as e:
            print(f"[flow] Corner detector error: {e}")
            return False

    def _get_sorted_frame_list(self):
        if not hasattr(self, '_sorted_frames_cache') or self._sorted_frames_cache is None:
            from pathlib import Path
            frames = [
                f for f in Path(FRAME_DIR).iterdir()
                if f.suffix.lower() in {".jpg", ".jpeg", ".png"}
            ]
            frames.sort(key=lambda f: int(f.stem) if f.stem.isdigit() else f.stem)
            self._sorted_frames_cache = frames
        return self._sorted_frames_cache

    # ── Map loading / building ─────────────────────────────────────────────────

    def _load_existing_map(self):
        import faiss, pickle
        from build_graph import load_graph

        self.graph = load_graph(GRAPH_PATH)

        with open(META_PATH, "rb") as f:
            meta = pickle.load(f)
        self.keyframes   = meta["keyframes"]
        self.descriptors = meta["descriptors"]
        self.index       = faiss.read_index(FAISS_PATH)

        # Tag junctions if not already tagged
        if self.graph and not any(
            'is_junction' in d for _, d in self.graph.nodes(data=True)
        ):
            self.graph = _tag_junctions(self.graph)

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

        print("\n── Stage 5: DINOv2 encode")
        paths            = [kf["path"] for kf in self.keyframes]
        self.descriptors = encode_geometric(self.encoder, paths, batch_size=32)

        print("\n── Stage 6: FAISS index")
        self.index = build_faiss_index(self.descriptors)

        print("\n── Stage 7: Pose graph")
        self.graph = build_pose_graph(
            self.keyframes, self.descriptors, self.index,
            loop_threshold=LOOP_THRESHOLD,
            loop_min_gap=LOOP_MIN_GAP,
            infer_actions=True,
        )

        print("\n── Stage 8: Tag junction nodes")
        self.graph = _tag_junctions(self.graph)

        print("\n── Stage 9: Save artefacts")
        save_artifacts(
            output_dir=GRAPH_DIR,
            keyframes=self.keyframes,
            descriptors=self.descriptors,
            index=self.index,
            graph=self.graph,
        )

    # ── Goal scoring ───────────────────────────────────────────────────────────

    def _score_target_images(self, target_imgs):
        """
        Score target images using all 4 views combined (multi-view voting).
        Returns (best_path, best_score, best_node).
        """
        view_names    = ["front", "left", "back", "right"]
        view_encodings = []
        view_paths     = []

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

    # ── Target image display ───────────────────────────────────────────────────

    def show_target_images(self):
        targets = self.get_target_images()
        if targets is None or len(targets) <= 0:
            return
        hor1       = cv2.hconcat(targets[:2])
        hor2       = cv2.hconcat(targets[2:])
        concat_img = cv2.vconcat([hor1, hor2])

        w, h   = concat_img.shape[:2]
        color  = (0, 0, 0)
        concat_img = cv2.line(concat_img, (int(h/2), 0), (int(h/2), w), color, 2)
        concat_img = cv2.line(concat_img, (0, int(w/2)), (h, int(w/2)), color, 2)

        font, size, stroke, line = cv2.FONT_HERSHEY_SIMPLEX, 0.75, 1, cv2.LINE_AA
        w_offset, h_offset = 25, 10
        cv2.putText(concat_img, 'Front View', (h_offset, w_offset),                        font, size, color, stroke, line)
        cv2.putText(concat_img, 'Left View',  (int(h/2) + h_offset, w_offset),             font, size, color, stroke, line)
        cv2.putText(concat_img, 'Back View',  (h_offset, int(w/2) + w_offset),             font, size, color, stroke, line)
        cv2.putText(concat_img, 'Right View', (int(h/2) + h_offset, int(w/2) + w_offset),  font, size, color, stroke, line)

        cv2.imshow('KeyboardPlayer:target_images', concat_img)
        cv2.imwrite('target.jpg', concat_img)
        cv2.waitKey(1)
        print("Displayed target images")

    # ── MANUAL MODE helpers ────────────────────────────────────────────────────

    def _manual_update_and_display(self):
        """
        Called once per Q press in manual mode.
        Runs fresh localize, replan, scan_directions (display only), heading.
        """
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator._goal_node is None:
            print("[manual] Goal not set yet.")
            return

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # Heading tracking
        yaw_delta = self._estimate_heading_change(small_fpv)
        self._cumulative_yaw += yaw_delta
        print(f"[manual] Heading: delta={yaw_delta:+.1f}°, cumulative={self._cumulative_yaw:+.1f}°")

        # Fresh localize
        node, score = self.navigator.localize_robust(small_fpv)
        print(f"[manual] Localized → node {node} (score={score:.3f})")

        self.navigator.current_node = node
        self.navigator.set_goal(self.navigator._goal_node)
        path = self.navigator.current_path or []
        hops = max(0, len(path) - 1)

        # Goal confirmation
        if node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            print(f"[manual] At goal! Confirmation {self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED}")
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                print("[manual] ✓ GOAL CONFIRMED — press SPACE!")
        else:
            self._goal_confirm_count = 0

        # scan_directions: MANUAL MODE ONLY — human reads scores on display panel
        direction_scores = self.navigator.scan_directions(small_fpv)
        self.cached_direction_scores = direction_scores

        for d, info in direction_scores.items():
            blocked_str = " [BLOCKED]" if info.get("blocked") else ""
            print(f"[manual]   {d:>6}: score={info['score']:.3f} dist={info['distance']}{blocked_str}")

        raw_action = self.navigator.next_action() if hops >= 1 else "stop"
        resolved_action, detail = self._resolve_action(raw_action, small_fpv, direction_scores)
        self._last_suggestion        = resolved_action
        self._last_suggestion_detail = detail
        print(f"[manual] Suggestion: {resolved_action.upper()} | {detail}")

        gray = cv2.cvtColor(small_fpv, cv2.COLOR_BGR2GRAY)
        self._prev_fpv_gray = gray

        self.display_next_best_view()

    def _estimate_heading_change(self, fpv: np.ndarray) -> float:
        """ORB-based heading estimation between Q presses."""
        gray = cv2.cvtColor(fpv, cv2.COLOR_BGR2GRAY)
        if self._prev_fpv_gray is None:
            return 0.0
        try:
            prev = self._prev_fpv_gray
            if prev.shape != gray.shape:
                prev = cv2.resize(prev, (gray.shape[1], gray.shape[0]))
            h, w = gray.shape
            orb  = cv2.ORB_create(nfeatures=200)
            kp1, des1 = orb.detectAndCompute(prev, None)
            kp2, des2 = orb.detectAndCompute(gray, None)
            if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
                return 0.0
            bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            if len(matches) < 5:
                return 0.0
            dx_list = [kp2[m.trainIdx].pt[0] - kp1[m.queryIdx].pt[0] for m in matches]
            median_dx        = float(np.median(dx_list))
            degrees_per_pixel = 90.0 / w
            return -median_dx * degrees_per_pixel
        except Exception as e:
            print(f"[heading] Estimation failed: {e}")
            return 0.0

    def _resolve_action(self, raw_action: str, small_fpv: np.ndarray,
                        direction_scores: dict = None) -> tuple:
        """
        MANUAL MODE: resolve graph action against scan_directions scores
        for display suggestion. Not used in AUTO mode.
        """
        dir_map = {"front": "forward", "left": "turn_left",
                   "right": "turn_right", "back": "backward"}

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
                return dir_map.get(best_dir, "forward"), detail

            return "forward", "all directions blocked — defaulting forward"

        return "forward", "no scan data — defaulting forward"

    def _simplify_path(self, path: list) -> list:
        """Collapse path into decision-relevant waypoints for display panel."""
        if not path or len(path) < 2:
            return []

        G    = self.navigator.G
        goal = self.navigator._goal_node
        waypoints, MAX_STRAIGHT, last_waypoint_hop = [], 5, 0

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
            is_junction = G.nodes[node].get('is_junction', False)
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
            waypoints.append({"node": goal, "hops_away": max(0, len(path) - 1),
                               "type": "goal", "label": "GOAL"})
        return waypoints

    # ── Display panel ──────────────────────────────────────────────────────────

    def display_next_best_view(self):
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator.current_node is None or self.navigator._goal_node is None:
            print("[panel] Goal not yet initialized.")
            return

        FONT = cv2.FONT_HERSHEY_SIMPLEX
        AA   = cv2.LINE_AA
        TW, TH = 260, 195
        PW, PH = 156, 117

        cur_node    = self.navigator.current_node
        goal_node   = self.navigator._goal_node
        path        = self.navigator.current_path or []
        hops        = max(0, len(path) - 1)
        next_action = self._last_suggestion if self._last_suggestion else "stop"
        detail_text = getattr(self, '_last_suggestion_detail', '')
        near        = hops <= 5

        panel_w = TW * 3

        # ── Info bar ──────────────────────────────────────────────────────────
        bar_h = 60 if detail_text else 40
        bar   = np.zeros((bar_h, panel_w, 3), dtype=np.uint8)
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
            cv2.putText(bar, f"NEAR GOAL ({self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED} confirms)",
                        (panel_w - 340, 22), FONT, 0.45, (0, 255, 255), 1, AA)
        if detail_text:
            cv2.putText(bar, detail_text, (8, 48), FONT, 0.40, (180, 180, 255), 1, AA)

        # ── Thumbnail helper ──────────────────────────────────────────────────
        def thumb(img, label, color, extra=None):
            t = cv2.resize(img, (TW, TH))
            cv2.rectangle(t, (0, 0), (TW-1, TH-1), color, 2)
            cv2.putText(t, label, (6, 22), FONT, 0.55, color, 1, AA)
            if extra:
                cv2.putText(t, extra, (6, 44), FONT, 0.45, (200, 200, 200), 1, AA)
            return t

        # ── Row 1: FPV | Best match | Target ─────────────────────────────────
        fpv_t     = thumb(self.fpv, "Live FPV", (255, 255, 255))

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

        # ── Row 2: Action status bar (replaces micro-steps) ──────────────────
        # Micro-steps (reading N intermediate frames from disk every render)
        # were removed — they cost disk I/O with no navigation value.
        row2 = np.zeros((30, panel_w, 3), dtype=np.uint8)
        is_junc = self.navigator.G.nodes[cur_node].get('is_junction', False) if cur_node is not None else False
        status  = f"Action: {next_action.upper()}  |  Hops: {hops}"
        if is_junc:
            status += "  |  [JUNCTION — path trusted]"
        cv2.putText(row2, status, (8, 20), FONT, 0.42, (0, 200, 200), 1, AA)

        # ── Row 3: Waypoints ──────────────────────────────────────────────────
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

                border_color = {
                    "turn":       (0, 200, 255),
                    "loop":       (200, 100, 255),
                    "goal":       (0, 255, 0),
                    "junction":   (0, 255, 255),
                    "checkpoint": (180, 180, 180),
                }.get(wp["type"], (200, 200, 0))

                cv2.rectangle(img, (0, 0), (PW-1, PH-1), border_color, 2)
                cv2.putText(img, f"{wp['hops_away']} hops", (4, 16),   FONT, 0.35, (255, 255, 255), 1, AA)
                cv2.putText(img, wp["label"],                (4, PH-8), FONT, 0.38, border_color,    1, AA)
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

        panel = cv2.vconcat([bar, row1, row2, row3_label, row3])
        cv2.imshow("Navigation Panel", panel)
        cv2.waitKey(1)

        print(f"── NAV: {next_action:<12} | Node {cur_node} → {goal_node} | {hops} hops")


if __name__ == "__main__":
    import logging
    logging.basicConfig(filename='vis_nav_player.log', filemode='w', level=logging.INFO,
                        format='%(asctime)s - %(levelname)s: %(message)s',
                        datefmt='%d-%b-%y %H:%M:%S')
    import vis_nav_game as vng
    logging.info(f'player.py is using vis_nav_game {vng.core.__version__}')
    vng.play(the_player=KeyboardPlayerPyGame())
