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

AUTO             = False   # If True, player will autonomously navigate using the navigator. If False, player will idle and allow manual control (e.g., via keyboard).

STEP             = 10
ROTATION_THRESH  = 15.0
MIN_GAP          = 10
HAMMING          = 8
LOOP_THRESHOLD   = 0.85
LOOP_MIN_GAP     = 50
DINO_SIZE        = "s"

ACTION_STR_TO_ENUM = {
    "forward":    Action.FORWARD,
    "turn_left":  Action.LEFT,
    "turn_right": Action.RIGHT,
    "backward":   Action.BACKWARD,
    "stop":       Action.CHECKIN,
}



class KeyboardPlayerPyGame(Player):
    def __init__(self):
         # Initialize the DINOv2-based geometric descriptor, pose graph, FAISS index, and keyframe metadata. These will be populated during the pre_exploration phase, either by loading existing artefacts from disk or by building them from the dataset.
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

        # Set after pre_navigation; stores the best target image path so that
        # the first act() call can do initial localise → set_goal in one shot.
        self._goal_img_path     = None
        self._goal_pending      = True   # True = need to localise+plan on next act()
        self._target_images_set = None   # Captured when set_target_images() is called

        # Navigation state machine
        self.state = "IDLE"  # "IDLE", "MOVING", or "GOAL_REACHED"
        self.navigator = None
        self.alignment_history = []      # Track alignment over frames
        self.consecutive_low_alignment = 0  # Counter for confidence monitoring

        # Goal-reached confirmation: require N consecutive matches before declaring victory
        self._goal_confirm_count = 0
        self._GOAL_CONFIRM_NEEDED = 5  # must match goal node 5 consecutive Q-presses

        # Heading tracking: estimate camera rotation between Q presses
        self._prev_fpv_gray = None   # grayscale of FPV at last Q press
        self._cumulative_yaw = 0.0   # estimated degrees turned since last Q press

        # The first-person view (FPV) image from the robot's camera.
        self.fpv = None
        self._last_suggestion = "IDLE"
        self._last_suggestion_detail = ""  # human-readable explanation
        self.last_act = Action.IDLE
        self.screen = None
        self.keymap = None
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
        """
        Build (or load) the full topological map before the exploration phase
        starts. The exploration phase itself just idles — the robot doesn't move.
        """
        print("\n" + "=" * 60)
        print("  PRE-RECORDED DATASET MODE  (skipping live exploration)")
        print(f"  Frame dir : {FRAME_DIR}")
        print(f"  Graph dir : {GRAPH_DIR}")
        K = self.get_camera_intrinsic_matrix()
        print(f'K={K}')
        print("=" * 60)

        # Ensure the graph directory exists
        os.makedirs(GRAPH_DIR, exist_ok=True)

        # Check if all artefacts (graph, FAISS index, metadata) exist on disk.
        # If they do, load them; otherwise, run the full pipeline to build the
        # map from the dataset.
        if (os.path.exists(GRAPH_PATH) and
                os.path.exists(FAISS_PATH) and
                os.path.exists(META_PATH)):
            print("\n[pre_exploration]  Existing map found — loading from disk…")
            self._load_existing_map()
        else:
            print("\n[pre_exploration]  No map found — building from dataset…")
            self._build_map_from_dataset()

        print("[pre_exploration]  Done. Exploration phase will idle.\n")
        super(KeyboardPlayerPyGame, self).pre_exploration()  # ← add this

    # ── Pre_navigation ────────────────────────────────────────────────────────
    def pre_navigation(self) -> None:
        """
        Initialize the MazeNavigator. Attempt to score target images if they
        are already available; defer to act() if not.
        """
        super(KeyboardPlayerPyGame, self).pre_navigation()
        print("\n[pre_navigation]  Setting up navigator…")

        # Check if the map was loaded or built during pre_exploration. If not, build it now.
        if self.graph is None or self.index is None:
            print("[pre_navigation]  Map missing — rebuilding now…")
            self._build_map_from_dataset()

        # Initialize the MazeNavigator with the loaded graph, index, keyframes, and descriptors. 
        # This object will handle localization, path planning, and navigation logic during the act() phase.
        self.navigator = MazeNavigator(
            graph=self.graph,
            index=self.index,
            keyframes=self.keyframes,
            descriptors=self.descriptors,
            encoder=self.encoder,
        )
        print("[pre_navigation]  Navigator ready — starting run.\n")

        # Cache the target images locally.
        # The game may not provide them again, so we store them for later use
        # (e.g., in act() if needed).
        target_imgs = self.get_target_images()
        self._target_images_set = target_imgs

        # Mark that goal initialization is still pending.
        # Even after selecting the best target image, we still need to:
        #   1. Localize the robot using fpv
        #   2. Map the goal image to a graph node
        #   3. Plan a path
        self._goal_pending = True

        # Retrieve target images using either:
        #   - the cached copy (preferred), or
        #   - the parent class getter (fallback / authoritative source)
        target_imgs = self._target_images_set or self.get_target_images()

         # Only reset goal state if we actually have new images to replace it with
        if target_imgs is not None and len(target_imgs) > 0:
            self._goal_img_path = None   # ← only clear if we can immediately refill it
            self._goal_pending = True
            best_path, best_score = self._score_target_images(target_imgs)
            print(f"[pre_navigation] Using image at {best_path} (sim={best_score:.4f})")
            self._goal_img_path = best_path
        else:
            # Images not available yet — preserve any previously set goal path
            # and let act() handle it when images arrive via set_target_images()
            print("[pre_navigation] Target images not available — preserving existing goal state.")
            self._goal_pending = (self._goal_img_path is None)
    
    def set_target_images(self, images):
        super(KeyboardPlayerPyGame, self).set_target_images(images)
        self._target_images_set = images   # ← cache HERE, this is the only reliable moment
        self.show_target_images()
    
    # ── see ───────────────────────────────────────────────────────────────────
    def see(self, fpv):
        #print("[see] called")  # ← add temporarily
        if fpv is None or len(fpv.shape) < 3:
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


        # ── Early return if goal already set ─────────────────────────────────
        if not self._goal_pending or self.navigator is None:
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

        # ── One-time goal init ────────────────────────────────────────────────
        target_imgs = self._target_images_set or self.get_target_images() or []
        if len(target_imgs) == 0:
            return

        if self._goal_img_path is None:
            print("[see] Scoring target images…")
            best_path, best_score = self._score_target_images(target_imgs)
            if best_path is None:
                return
            self._goal_img_path = best_path
            print(f"[see] Goal image: {best_path} (sim={best_score:.4f})")

        print("[see] Localizing…")
        self.navigator.localize_robust(self.fpv)
        print("[see] Planning path to goal…")
        self.navigator.set_goal_by_image(self._goal_img_path)

        if self.navigator._goal_node is not None and self.navigator.current_path:
            self._goal_pending = False
            print("[see] ✓ Goal initialized — act() will now execute.")
        else:
            print("[see] ✗ Planning failed — will retry next frame.")

    # ── act ───────────────────────────────────────────────────────────────────
    def act(self):
        # ── 1. Skip Exploration Safely ──
        if self._state and self._state[1] == Phase.EXPLORATION:
            frame_num = self._state[2]
            if frame_num == 0:
                self.last_act = Action.IDLE 
                return Action.QUIT
                
        # ── 2. Handle Pygame Keyboard Events ──
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

        # ── 3. Wait for Setup ──
        if self._goal_pending or self.navigator is None or self.navigator._goal_node is None:
            return Action.IDLE

        # ── 4. AUTO mode: run the state machine every frame ──
        if AUTO:
            final_action = self._execute_navigation()
            if final_action is None:
                return self.last_act
            if final_action == Action.QUIT:
                print("[act] Navigator has requested QUIT. Goal reached!")
                return Action.QUIT
            return final_action

        # ── 5. MANUAL mode: only update navigation on Q press ──
        if q_pressed and self.fpv is not None:
            self._manual_update_and_display()

        return self.last_act
    # -------- ALL HELPER FUNCTIONS BELOW THIS LINE --------

    # ── _load_existing_map ────────────────────────────────────────────────────
    def _load_existing_map(self):
        import faiss, pickle

        from build_graph import load_graph
        self.graph = load_graph(GRAPH_PATH)  # returns None if file missing

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

    # ── _build_map_from_dataset ───────────────────────────────────────────────
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

    # ── Score_target_images ──────────────────────────────────────────────────
    def _score_target_images(self, target_imgs):
        best_score = -1.0
        best_path = None
        view_names = ["front", "left", "back", "right"]

        for i, img in enumerate(target_imgs[:4]):
            fd, tmp = tempfile.mkstemp(suffix=f"_{view_names[i]}.jpg")
            os.close(fd)

            sim = -1.0
            try:
                ok = cv2.imwrite(tmp, img)
                if not ok:
                    print(f"  [{view_names[i]}] failed to write temp image")
                    continue

                q = self.encoder.encode(tmp).reshape(1, -1).astype(np.float32)
                scores, _ = self.index.search(q, 1)
                sim = float(scores[0][0])
                print(f"  [{view_names[i]:5s}] similarity: {sim:.4f}")

                if sim > best_score:
                    best_score = sim
                    best_path = tmp

            except Exception as e:
                print(f"  [{view_names[i]}] encoding error: {e}")

            # do NOT delete the best path yet, because you return it
            if tmp != best_path and os.path.exists(tmp):
                os.remove(tmp)

        return best_path, best_score
    
    # ── Show_target_images ────────────────────────────────────────────────────
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
        font = cv2.FONT_HERSHEY_SIMPLEX
        line = cv2.LINE_AA
        size = 0.75
        stroke = 1

        cv2.putText(concat_img, 'Front View', (h_offset, w_offset), font, size, color, stroke, line)
        cv2.putText(concat_img, 'Left View', (int(h/2) + h_offset, w_offset), font, size, color, stroke, line)
        cv2.putText(concat_img, 'Back View', (h_offset, int(w/2) + w_offset), font, size, color, stroke, line)
        cv2.putText(concat_img, 'Right View', (int(h/2) + h_offset, int(w/2) + w_offset), font, size, color, stroke, line)

        cv2.imshow(f'KeyboardPlayer:target_images', concat_img)
        cv2.imwrite('target.jpg', concat_img)
        cv2.waitKey(1)
        print("Displayed target images")

    # ── _execute_navigation ───────────────────────────────────────────────────
    def _execute_navigation(self):
        self.nav_frame_idx += 1
        """
        Main state machine for autonomous navigation with direction scanning.

        States:
            IDLE         → Localize, scan 4 directions, choose best aligned.
            MOVING       → Monitor alignment, detect stop conditions.
            GOAL_REACHED → Return CHECKIN.
        """
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
        """Cache and return the sorted list of all frame paths from FRAME_DIR."""
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
    
    # ── _act_idle ─────────────────────────────────────────────────────────────
    def _act_idle(self):
        """
        IDLE State: Scan 4 directions, compare to route, choose best aligned.
        Returns direction instruction or transitions to MOVING.
        """
        if self.navigator._goal_node is None:
            print("[idle] ⚠️  No goal set.")
            return Action.IDLE
        
        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # 1. Localize current position
        if self.nav_frame_idx % 3 == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.localize_robust(small_fpv)

        current_node, confidence = self.cached_localize
        print(f"[idle] Current node: {current_node} (confidence: {confidence:.3f})")

        # 1a. Replan after strong re-localization (normalized vote sum)
        MAX_POSSIBLE_VOTE = 4 * 10 * 1.0
        confidence_norm = confidence / MAX_POSSIBLE_VOTE
        if confidence_norm > 0.5:
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

        # 3. ONE scan_directions call — result reused for all checks below
        if self.nav_frame_idx % 3 == 0 or self.cached_direction_scores is None:
            self.cached_direction_scores = self.navigator.scan_directions(small_fpv)

        direction_scores = self.cached_direction_scores
        print(f"[idle] Direction scores: {direction_scores}")

        # 4. Junction only checked if topology suggests a choice point
        out_edges = list(self.navigator.G.successors(self.navigator.current_node))
        if len(out_edges) > 1:
            if self.navigator.detect_junction(direction_scores, threshold=0.6):
                print("[idle] ⚠️  JUNCTION detected — multiple strong matches")
                return Action.IDLE

        # 5. Dead end always checked
        if self.navigator.detect_dead_end(direction_scores, threshold=0.4):
            print("[idle] ⚠️  DEAD END detected — no strong matches")
            return Action.IDLE

        # 6. Choose best aligned direction (excluding blocked)
        feasible = {
            d: info for d, info in direction_scores.items()
            if not info.get("blocked", False)
        } if direction_scores else {}

        if not feasible:
            print("[idle] ⚠️  All directions blocked — staying IDLE")
            return Action.IDLE

        best_direction = max(
            feasible.items(),
            key=lambda x: x[1].get("score", 0)
        )[0]

        best_score    = feasible[best_direction]["score"]
        best_distance = feasible[best_direction]["distance"]
        blocked_dirs = [d for d, info in direction_scores.items() if info.get("blocked")]
        print(f"[idle] Best direction: {best_direction} "
            f"(score={best_score:.3f}, distance={best_distance})"
            f"{' | blocked: ' + ','.join(blocked_dirs) if blocked_dirs else ''}")

        # Map crop direction names → action strings
        dir_to_action = {"front": "forward", "left": "turn_left", 
                         "right": "turn_right", "back": "backward"}
        best_action = dir_to_action.get(best_direction, "forward")

        # 7. Transition to MOVING
        self.state = "MOVING"
        self.alignment_history = []
        self.consecutive_low_alignment = 0
        self._last_suggestion = best_action
        
        return ACTION_STR_TO_ENUM.get(best_action, Action.FORWARD)

    # ── _act_moving ───────────────────────────────────────────────────────────
    def _act_moving(self):
        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # 1. Get alignment
        if self.nav_frame_idx % 3 == 0 or self.cached_alignment is None:
            self.cached_alignment = self.navigator.get_alignment_scores(small_fpv)

        alignment = self.cached_alignment
        primary = alignment.get("primary", 0.0)
        confidence = alignment.get("route_confidence", 0.0)

        self.alignment_history.append(primary)
        if len(self.alignment_history) > 10:
            self.alignment_history.pop(0)

        print(f"[moving] Alignment: primary={primary:.3f}, confidence={confidence:.3f}")

        # 2. Confirm step
        if self.nav_frame_idx % 3 == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.confirm_step(small_fpv)

        node_after_step, step_score = self.cached_localize
        print(f"[moving] Step confirmed: node={node_after_step}, score={step_score:.3f}")

        # 3. Goal reached? Require multiple consecutive confirmations
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

        # 4. Alignment drop?
        if primary < 0.3:
            self.consecutive_low_alignment += 1
            if self.consecutive_low_alignment > 3:
                print("[moving] ⚠️  ALIGNMENT DROP detected (3+ frames)")
                self.state = "IDLE"
                self.cached_direction_scores = None
                self.cached_alignment = None
                return Action.IDLE
        else:
            self.consecutive_low_alignment = 0

        # 5. ONE scan_directions call — reused for ALL remaining checks
        if self.nav_frame_idx % 3 == 0 or self.cached_direction_scores is None:
            self.cached_direction_scores = self.navigator.scan_directions(small_fpv) or {}

        direction_scores = self.cached_direction_scores

        # Better node ahead?
        if "front" in direction_scores:
            front_score = direction_scores["front"].get("score", 0)
            front_distance = direction_scores["front"].get("distance", 0)
            if front_distance > 1 and front_score > 0.8:
                print(f"[moving] Better forward node at distance {front_distance}")
                self.state = "IDLE"
                self.cached_direction_scores = None
                self.cached_alignment = None
                return Action.IDLE

        # Junction?
        if self.navigator.detect_junction(direction_scores, threshold=0.6):
            print("[moving] ⚠️  JUNCTION detected while moving")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_alignment = None
            return Action.IDLE

        # Dead end?
        if self.navigator.detect_dead_end(direction_scores, threshold=0.4):
            print("[moving] ⚠️  DEAD END detected while moving")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_alignment = None
            return Action.IDLE

        # 6. Pick direction from scan_directions (ignores stored edge actions)
        if direction_scores:
            feasible = [
                (d, info) for d, info in direction_scores.items()
                if not info.get("blocked", False)
            ]
            feasible.sort(key=lambda x: -x[1]["score"])
            if feasible:
                dir_to_action = {"front": "forward", "left": "turn_left",
                                 "right": "turn_right"}
                action_str = dir_to_action.get(feasible[0][0], "forward")
                print(f"[moving] Best visible direction: {feasible[0][0]} "
                      f"(score={feasible[0][1]['score']:.3f})")
            else:
                action_str = "forward"
                print("[moving] All directions blocked — defaulting forward")
        else:
            action_str = "forward"
            print("[moving] No direction data — defaulting forward")

        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)
    
    # ── _manual_update_and_display (MANUAL mode: fresh update on Q press) ────
    def _manual_update_and_display(self):
        """
        Called once per Q press in manual mode.
        Does a FRESH localization (no cache), replans, scans directions,
        estimates heading change since last Q press, and shows the panel.
        """
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator._goal_node is None:
            print("[manual] Goal not set yet.")
            return

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # 0. Heading tracking: estimate rotation since last Q press
        yaw_delta = self._estimate_heading_change(small_fpv)
        self._cumulative_yaw += yaw_delta
        print(f"[manual] Heading: delta={yaw_delta:+.1f}°, cumulative={self._cumulative_yaw:+.1f}°")

        # 1. Fresh localization — NO cache
        node, score = self.navigator.localize_robust(small_fpv)
        print(f"[manual] Localized → node {node} (score={score:.3f})")

        # 2. Replan path from where we actually are
        self.navigator.current_node = node
        self.navigator.set_goal(self.navigator._goal_node)
        path = self.navigator.current_path or []
        hops = max(0, len(path) - 1)

        # 3. Goal proximity check WITH confirmation
        if node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            print(f"[manual] At goal node! Confirmation {self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED}")
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                print("[manual] ✓ GOAL CONFIRMED — press SPACE to check in!")
        else:
            self._goal_confirm_count = 0

        # 4. Scan directions with feasibility checking
        direction_scores = self.navigator.scan_directions(small_fpv)
        self.cached_direction_scores = direction_scores

        # Log direction info including blocked status
        for d, info in direction_scores.items():
            blocked_str = " [BLOCKED]" if info.get("blocked") else ""
            print(f"[manual]   {d:>6}: score={info['score']:.3f} dist={info['distance']}{blocked_str}")

        # 5. Pick direction purely from scan (ignores stored edge actions)
        raw_action = self.navigator.next_action() if hops >= 1 else "stop"
        resolved_action, detail = self._resolve_action(raw_action, small_fpv, direction_scores)
        self._last_suggestion = resolved_action
        self._last_suggestion_detail = detail

        print(f"[manual] Suggestion: {resolved_action.upper()}")
        print(f"[manual] Scores: {detail}")

        # 6. Save current FPV for next heading estimation
        gray = cv2.cvtColor(small_fpv, cv2.COLOR_BGR2GRAY)
        self._prev_fpv_gray = gray

        # 7. Show panel
        self.display_next_best_view()

    def _estimate_heading_change(self, fpv: np.ndarray) -> float:
        """
        Estimate how many degrees the camera rotated since the last Q press
        using optical flow on the horizontal axis.

        Positive = turned right, Negative = turned left.
        """
        gray = cv2.cvtColor(fpv, cv2.COLOR_BGR2GRAY)

        if self._prev_fpv_gray is None:
            return 0.0

        try:
            prev = self._prev_fpv_gray
            # Resize to match if shapes differ
            if prev.shape != gray.shape:
                prev = cv2.resize(prev, (gray.shape[1], gray.shape[0]))

            # Use phase correlation — fast and robust for pure rotation
            h, w = gray.shape
            # Convert to float for phase correlation
            prev_f = np.float32(prev)
            curr_f = np.float32(gray)

            # Use feature matching for rotation estimation
            # ORB is fast and doesn't need GPU
            orb = cv2.ORB_create(nfeatures=200)
            kp1, des1 = orb.detectAndCompute(prev, None)
            kp2, des2 = orb.detectAndCompute(gray, None)

            if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
                return 0.0

            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)

            if len(matches) < 5:
                return 0.0

            # Compute average horizontal displacement of matched keypoints
            dx_list = []
            for m in matches:
                pt1 = kp1[m.queryIdx].pt
                pt2 = kp2[m.trainIdx].pt
                dx_list.append(pt2[0] - pt1[0])

            median_dx = float(np.median(dx_list))

            # Convert pixel displacement to approximate degrees
            # Assuming ~90° horizontal FOV for the FPV camera
            fov_degrees = 90.0
            degrees_per_pixel = fov_degrees / w
            yaw_delta = -median_dx * degrees_per_pixel  # negative because moving right = positive yaw

            return yaw_delta

        except Exception as e:
            print(f"[heading] Estimation failed: {e}")
            return 0.0

    def _resolve_action(self, raw_action: str, small_fpv: np.ndarray,
                        direction_scores: dict = None) -> tuple:
        """
        Determine the physical direction to move based on what the camera
        ACTUALLY SEES right now.

        CRITICAL INSIGHT: stored edge actions (from exploration) are relative
        to the camera heading DURING EXPLORATION, not the player's current
        heading. They're unreliable. Instead, we ALWAYS use scan_directions
        which compares the current front/left/right crops against the NEXT
        path node. Whichever direction best matches = that's where to go.

        Returns (action_str, human_detail).
        """
        dir_map = {"front": "forward", "left": "turn_left", "right": "turn_right"}

        # ── Handle 'stop' ──
        if raw_action == "stop":
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                return "stop", "GOAL CONFIRMED — press SPACE"
            return "stop", "end of path (may need replan)"

        # ── ALWAYS use scan_directions as the primary direction source ──
        if direction_scores:
            # Filter to unblocked directions
            feasible = [
                (d, info) for d, info in direction_scores.items()
                if not info.get("blocked", False)
            ]
            feasible.sort(key=lambda x: -x[1]["score"])

            if feasible:
                best_dir = feasible[0][0]
                best_info = feasible[0][1]
                resolved = dir_map.get(best_dir, "forward")

                # Build detail showing all options
                detail_parts = []
                for d, info in feasible:
                    marker = ">>>" if d == best_dir else "   "
                    blk = " [WALL]" if info.get("blocked") else ""
                    detail_parts.append(f"{marker}{d}={info['score']:.2f}{blk}")
                detail = " | ".join(detail_parts)

                return resolved, detail

            return "forward", "all directions blocked — defaulting forward"

        # ── Fallback: no scan data (shouldn't happen in manual mode) ──
        return "forward", "no scan data — defaulting forward"

    def _simplify_path(self, path: list) -> list:
        """
        Collapse the full path into decision-relevant waypoints.
        Skips consecutive 'forward' edges; only keeps nodes where:
          - The edge action changes (turn, loop closure)
          - The node is a junction (multiple outgoing edges)
          - The node is the goal
        
        Returns list of dicts: [{node, hops_away, type, label}, ...]
        """
        if not path or len(path) < 2:
            return []

        G = self.navigator.G
        goal = self.navigator._goal_node
        waypoints = []

        for i in range(1, len(path)):
            node = path[i]
            prev = path[i - 1]
            hops_away = i  # hops from current position

            ed = G.get_edge_data(prev, node)
            edge_action = ed.get("action", "forward") if ed else "forward"
            edge_type = ed.get("edge_type", "sequential") if ed else "sequential"

            # Check if this node is a junction (multiple successors)
            n_successors = len(list(G.successors(node)))

            is_turn = edge_action in ("turn_left", "turn_right")
            is_loop = edge_type == "loop_closure" or edge_action == "loop"
            is_junction = n_successors > 2
            is_goal = node == goal

            if is_goal:
                waypoints.append({
                    "node": node, "hops_away": hops_away,
                    "type": "goal", "label": "GOAL"
                })
                break
            elif is_loop:
                waypoints.append({
                    "node": node, "hops_away": hops_away,
                    "type": "loop", "label": "SHORTCUT"
                })
            elif is_turn:
                direction = "LEFT" if edge_action == "turn_left" else "RIGHT"
                waypoints.append({
                    "node": node, "hops_away": hops_away,
                    "type": "turn", "label": f"TURN {direction}"
                })
            elif is_junction:
                waypoints.append({
                    "node": node, "hops_away": hops_away,
                    "type": "junction", "label": f"JUNCTION ({n_successors})"
                })

            if len(waypoints) >= 5:
                break

        # If no waypoints found (all forward), show goal distance
        if not waypoints and goal is not None:
            waypoints.append({
                "node": goal, "hops_away": max(0, len(path) - 1),
                "type": "goal", "label": "GOAL (straight)"
            })

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

        cur_node   = self.navigator.current_node
        goal_node  = self.navigator._goal_node
        path       = self.navigator.current_path or []
        hops       = max(0, len(path) - 1)

        # Next action — use the already-resolved suggestion
        next_action = self._last_suggestion if self._last_suggestion else "stop"
        detail_text = getattr(self, '_last_suggestion_detail', '')
        near = hops <= 5

        # ── Info bar ─────────────────────────────────────────────────────────
        panel_w = TW * 3
        bar_h = 60 if detail_text else 40
        bar = np.zeros((bar_h, panel_w, 3), dtype=np.uint8)

        if self.state == "GOAL_REACHED":
            bar[:] = (0, 160, 0)  # green
        elif near:
            bar[:] = (0, 0, 160)  # red
        else:
            bar[:] = (50, 35, 15)

        # Line 1: node, goal, hops, direction + heading
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

        # ── Thumbnail helper ─────────────────────────────────────────────────
        def thumb(img, label, color, extra=None):
            t = cv2.resize(img, (TW, TH))
            cv2.rectangle(t, (0, 0), (TW-1, TH-1), color, 2)
            cv2.putText(t, label, (6, 22), FONT, 0.55, color, 1, AA)
            if extra:
                cv2.putText(t, extra, (6, 44), FONT, 0.45, (200, 200, 200), 1, AA)
            return t

        # ── Row 1: FPV | Best match | Target ─────────────────────────────────
        fpv_t = thumb(self.fpv, "Live FPV", (255, 255, 255))

        # Best match — load keyframe image for current node
        match_img = None
        if cur_node is not None and cur_node < len(self.keyframes):
            match_img = cv2.imread(self.keyframes[cur_node]["path"])
        if match_img is None:
            match_img = np.zeros((TH, TW, 3), dtype=np.uint8)
        match_t = thumb(match_img, f"Match: node {cur_node}", (0, 255, 0))

        # Target — best scoring view saved to /tmp
        tgt_img = None
        if self._goal_img_path and os.path.exists(self._goal_img_path):
            tgt_img = cv2.imread(self._goal_img_path)
        if tgt_img is None:
            tgt_img = np.zeros((TH, TW, 3), dtype=np.uint8)
        tgt_t = thumb(tgt_img, "Goal Image", (0, 140, 255))

        row1 = cv2.hconcat([fpv_t, match_t, tgt_t])

        # ── Row 2: MICRO-STEPS — intermediate frames to next node ─────────
        # Shows what you'll actually see as you walk toward the next node
        N_MICRO = 5
        micro_cells = []

        if len(path) >= 2:
            cur_fidx  = self.keyframes[path[0]]["frame_idx"] if path[0] < len(self.keyframes) else 0
            next_fidx = self.keyframes[path[1]]["frame_idx"] if path[1] < len(self.keyframes) else 0

            # Also get the edge action for this hop
            edge_data = self.navigator.G.get_edge_data(path[0], path[1])
            hop_action = ""
            if edge_data:
                hop_action = edge_data.get("action", "forward")
                if hop_action == "loop" or edge_data.get("edge_type") == "loop_closure":
                    hop_action = self._last_suggestion  # use resolved action

            # Sample intermediate frame indices — skip more to show visible changes
            if abs(next_fidx - cur_fidx) > 2:
                step_dir = 1 if next_fidx > cur_fidx else -1
                all_intermediate = list(range(cur_fidx + step_dir, next_fidx, step_dir))
                # Pick N_MICRO evenly spaced samples, but skip the first few
                # (too similar to current) to show more visible progression
                if len(all_intermediate) > N_MICRO + 2:
                    # Skip first 2 frames (nearly identical to current), spread the rest
                    usable = all_intermediate[2:]
                    indices = [usable[int(i * len(usable) / N_MICRO)]
                               for i in range(N_MICRO)]
                elif len(all_intermediate) > N_MICRO:
                    indices = [all_intermediate[int(i * len(all_intermediate) / N_MICRO)]
                               for i in range(N_MICRO)]
                else:
                    indices = all_intermediate
            else:
                indices = []

            # Load and display each intermediate frame — clean thumbnails
            sorted_frames = self._get_sorted_frame_list()
            for i, fidx in enumerate(indices):
                img = None
                if sorted_frames and 0 <= fidx < len(sorted_frames):
                    img = cv2.imread(str(sorted_frames[fidx]))
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))

                cv2.rectangle(img, (0, 0), (PW-1, PH-1), (0, 200, 200), 1)
                # Show frame number + progress
                progress_pct = int(100 * (i + 1) / max(len(indices), 1))
                label = f"f{fidx} ({progress_pct}%)"
                cv2.putText(img, label, (4, 16), FONT, 0.32, (255, 255, 255), 1, AA)
                if i == 0 and hop_action:
                    cv2.putText(img, hop_action.upper(), (4, PH - 8),
                                FONT, 0.40, (0, 255, 255), 1, AA)
                micro_cells.append(img)

        # Pad to fill row
        while len(micro_cells) < N_MICRO:
            micro_cells.append(np.zeros((PH, PW, 3), dtype=np.uint8))

        row2 = cv2.hconcat(micro_cells)
        if row2.shape[1] < panel_w:
            pad = np.zeros((PH, panel_w - row2.shape[1], 3), dtype=np.uint8)
            row2 = cv2.hconcat([row2, pad])

        # ── Row 2 label ──
        row2_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row2_label, "Micro-steps to next node",
                    (6, 14), FONT, 0.38, (0, 200, 200), 1, AA)

        # ── Row 3: SIMPLIFIED WAYPOINTS — only show direction changes ─────
        # Collapse consecutive "forward" edges; show only turns/junctions/loops
        waypoints = self._simplify_path(path)
        N_WAYPOINTS = 5
        cells = []
        for p in range(N_WAYPOINTS):
            if p < len(waypoints):
                wp = waypoints[p]
                node_idx = wp["node"]
                img = None
                if node_idx < len(self.keyframes):
                    img = cv2.imread(self.keyframes[node_idx]["path"])
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))

                # Color-code by event type
                if wp["type"] == "turn":
                    border_color = (0, 200, 255)   # orange for turns
                elif wp["type"] == "loop":
                    border_color = (200, 100, 255)  # purple for shortcuts
                elif wp["type"] == "goal":
                    border_color = (0, 255, 0)      # green for goal
                else:
                    border_color = (200, 200, 0)    # yellow default

                cv2.rectangle(img, (0, 0), (PW-1, PH-1), border_color, 2)

                # Top label: hops away
                hops_to = wp["hops_away"]
                cv2.putText(img, f"{hops_to} hops", (4, 16),
                            FONT, 0.35, (255, 255, 255), 1, AA)

                # Bottom label: what happens here
                cv2.putText(img, wp["label"], (4, PH - 8),
                            FONT, 0.38, border_color, 1, AA)
            else:
                img = np.zeros((PH, PW, 3), dtype=np.uint8)
            cells.append(img)

        row3 = cv2.hconcat(cells)
        if row3.shape[1] < panel_w:
            pad = np.zeros((PH, panel_w - row3.shape[1], 3), dtype=np.uint8)
            row3 = cv2.hconcat([row3, pad])

        # ── Row 3 label ──
        row3_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row3_label, "Waypoints ahead (turns & decision points only)",
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