from vis_nav_game import Player, Action, Phase
import pygame
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

FRAME_DIR        = "/Users/demarcewilliams/vis_nav_player/Non-Autonomous/images"
GRAPH_DIR        = "/Users/demarcewilliams/vis_nav_player/Non-Autonomous/fusion_output"
GRAPH_PATH       = os.path.join(GRAPH_DIR, "maze_graph.pkl")
FAISS_PATH       = os.path.join(GRAPH_DIR, "keyframe_index.faiss")
META_PATH        = os.path.join(GRAPH_DIR, "keyframe_meta.pkl")

AUTO             = False   # If True, player will autonomously navigate using the navigator. If False, player will idle and allow manual control (e.g., via keyboard).

STEP             = 40
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
        #self.passed_nodes = set()        # Track skipped nodes
        self.alignment_history = []      # Track alignment over frames
        self.consecutive_low_alignment = 0  # Counter for confidence monitoring

        # The first-person view (FPV) image from the robot's camera. This will be updated in real-time as the robot explores the environment. The FPV is used for both visualization and for any perception-based decision-making the player might implement.
        self.fpv = None
        self._last_suggestion = "IDLE"
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
            # Only quit on the absolute first frame to skip exploration.
            if frame_num == 0:
                self.last_act = Action.IDLE 
                return Action.QUIT
                
        # ── 2. Handle Pygame Keyboard Events ──
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("[act] Window closed by user.")
                    return Action.QUIT  # <-- RESTORED: Let the user close the window!
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        if not self._goal_pending and self.navigator is not None and self.navigator._goal_node is not None:
                            self.display_next_best_view()
                    elif event.key in self.keymap:
                        self.last_act |= self.keymap[event.key]
                elif event.type == pygame.KEYUP:
                    if event.key in self.keymap:
                        self.last_act ^= self.keymap[event.key]
        except pygame.error as e:
            print(f"[act] Pygame error: {e}")

        # <-- RESTORED: If you press the mapped quit key (usually ESC), let it quit!
        if self.last_act == Action.QUIT:
            return Action.QUIT

        # ── 3. Wait for Setup ──
        if self._goal_pending or self.navigator is None or self.navigator._goal_node is None:
            return Action.IDLE

        # ── 4. Execute Navigation ──
        final_action = self._execute_navigation()
        
        # ── 5. Safety Intercepts ──
        if final_action is None:
            return self.last_act 

        # <-- RESTORED: Let the navigator end the game when it reaches the goal!
        if final_action == Action.QUIT:
            print("[act] 🎯 Navigator has requested QUIT. Goal reached!")
            return Action.QUIT
            
        return final_action
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
        """
        Score each target view image against the FAISS index using DINOv2
        descriptors. Saves each view to /tmp, queries the index, and returns
        the path of the best-matching view.

        Returns the best image path, or None if scoring fails.
        """
        # Set up variables to track the best matching image and its score.
        best_score, best_path = -1.0, None

        # Save target images with labels for debugging and visualization
        view_names = ["front", "left", "back", "right"]

        # Score each view and keep track of the best one
        for i, img in enumerate(target_imgs[:4]):
            tmp = f"/tmp/target_{view_names[i]}.jpg" # Save with view label for clarity
            cv2.imwrite(tmp, img) 
            try: # Add error handling for encoding issues
                q = self.encoder.encode(tmp).reshape(1, -1).astype(np.float32)
                scores, _ = self.index.search(q, 1)
                sim = float(scores[0][0])
            except Exception as e: # Catch any exception during encoding or searching
                print(f"  [{view_names[i]}] encoding error: {e}")
                sim = -1.0

            print(f"  [{view_names[i]:5s}] similarity: {sim:.4f}")
            if sim > best_score:
                best_score = sim
                best_path  = tmp

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
    
    # ── _act_idle ─────────────────────────────────────────────────────────────
    def _act_idle(self):
        """
        IDLE State: Scan 4 directions, compare to route, choose best aligned.
        Returns direction instruction or transitions to MOVING.
        """
        if self.navigator._goal_node is None:
            print("[idle] ⚠️  No goal set.")
            return Action.IDLE

        # 1. Localize current position
        current_node, confidence = self.navigator.localize_robust(self.fpv)
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
        direction_scores = self.navigator.scan_directions(self.fpv)
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

        # 6. Choose best aligned direction
        best_direction = max(
            direction_scores.items(),
            key=lambda x: x[1].get("score", 0)
        )[0] if direction_scores else "forward"

        best_score    = direction_scores[best_direction]["score"]
        best_distance = direction_scores[best_direction]["distance"]
        print(f"[idle] Best direction: {best_direction} "
            f"(score={best_score:.3f}, distance={best_distance})")

        # 7. Transition to MOVING
        self.state = "MOVING"
        self.alignment_history = []
        self.consecutive_low_alignment = 0
        self._last_suggestion = best_direction
        
        return ACTION_STR_TO_ENUM.get(best_direction, Action.FORWARD)

    # ── _act_moving ───────────────────────────────────────────────────────────
    # ── _act_moving ───────────────────────────────────────────────────────────
    def _act_moving(self):
        # 1. Get alignment
        alignment  = self.navigator.get_alignment_scores(self.fpv)
        primary    = alignment.get("primary", 0.0)
        confidence = alignment.get("route_confidence", 0.0)

        self.alignment_history.append(primary)
        if len(self.alignment_history) > 10:
            self.alignment_history.pop(0)

        print(f"[moving] Alignment: primary={primary:.3f}, confidence={confidence:.3f}")

        # 2. Confirm step
        node_after_step, step_score = self.navigator.confirm_step(self.fpv)
        print(f"[moving] Step confirmed: node={node_after_step}, score={step_score:.3f}")

        # 3. Goal reached?
        if self.navigator.current_node == self.navigator._goal_node:
            self.state = "GOAL_REACHED"
            print("[moving] ✓ GOAL REACHED")
            return Action.CHECKIN

        # 4. Alignment drop?
        if primary < 0.3:
            self.consecutive_low_alignment += 1
            if self.consecutive_low_alignment > 3:
                print("[moving] ⚠️  ALIGNMENT DROP detected (3+ frames)")
                self.state = "IDLE"
                return Action.IDLE
        else:
            self.consecutive_low_alignment = 0

        # 5. ONE scan_directions call — reused for ALL remaining checks
        direction_scores = self.navigator.scan_directions(self.fpv) or {}

        # Better node ahead?
        if "front" in direction_scores:
            front_score    = direction_scores["front"].get("score", 0)
            front_distance = direction_scores["front"].get("distance", 0)
            if front_distance > 1 and front_score > 0.8:
                print(f"[moving] Better forward node at distance {front_distance}")
                self.state = "IDLE"
                return Action.IDLE

        # Junction?
        if self.navigator.detect_junction(direction_scores, threshold=0.6):
            print("[moving] ⚠️  JUNCTION detected while moving")
            self.state = "IDLE"
            return Action.IDLE

        # Dead end?
        if self.navigator.detect_dead_end(direction_scores, threshold=0.4):
            print("[moving] ⚠️  DEAD END detected while moving")
            self.state = "IDLE"
            return Action.IDLE

        # 6. Execute Visually-Guided Movement (Closed-Loop Servoing)
        # 6. Continue on planned route (Hybrid memory/radar approach)
        action_str = self.navigator.next_action()
        
        # --- THE WORMHOLE INTERCEPT ---
        if action_str == "use_radar":
            # Keep console clean: only print once when switching
            if action_str != getattr(self, '_last_suggestion', None):
                print("[moving] 🪩 Wormhole shortcut detected! Trusting visual radar.")
            
            if direction_scores:
                best_dir = max(
                    direction_scores.items(),
                    key=lambda x: x[1].get("score", 0)
                )[0]
                
                # TRANSLATE RADAR VISION TO MOTOR COMMANDS
                if best_dir == "left":
                    action_str = "turn_left"
                elif best_dir == "right":
                    action_str = "turn_right"
                elif best_dir == "back":
                    action_str = "turn_left"  # Keep turning to spin around
                else:
                    action_str = "forward"
                    
                if action_str != getattr(self, '_last_suggestion', None):
                    print(f"[moving] Radar steering: {action_str} (Target is {best_dir})")
            else:
                print("[moving] ⚠️ Radar blind. Forcing IDLE state.")
                self.state = "IDLE"
                return Action.IDLE
        else:
            # Normal edge - use the graph's memory!
            if action_str != getattr(self, '_last_suggestion', None):
                print(f"[moving] Following graph memory: {action_str}")

        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)
    
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
        N_PREVIEW = 5

        cur_node   = self.navigator.current_node
        goal_node  = self.navigator._goal_node
        path       = self.navigator.current_path or []
        hops       = max(0, len(path) - 1)

        # Next action from path: Prefer our translated motor command if available!
        saved_action = getattr(self, '_last_suggestion', None)
        if saved_action:
            next_action = saved_action
        else:
            next_action = self.navigator.next_action() if len(path) >= 2 else "stop"
            
        near = hops <= 5

        # ── Info bar ─────────────────────────────────────────────────────────
        panel_w = TW * 3
        bar = np.zeros((40, panel_w, 3), dtype=np.uint8)
        bar[:] = (0, 0, 160) if near else (50, 35, 15)
        txt = (f"Node {cur_node}  |  Goal {goal_node}"
            f"  |  {hops} hops  |  >> {next_action.upper()}")
        cv2.putText(bar, txt, (8, 27), FONT, 0.48, (255, 255, 255), 1, AA)
        if near:
            cv2.putText(bar, "NEAR GOAL — SPACE TO CHECK IN",
                        (panel_w - 280, 27), FONT, 0.45, (0, 255, 255), 1, AA)

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

        # ── Row 2: path preview (next N nodes) ───────────────────────────────
        preview_nodes = path[1:1 + N_PREVIEW]
        cells = []
        for p in range(N_PREVIEW):
            if p < len(preview_nodes):
                node_idx = preview_nodes[p]
                img = None
                if node_idx < len(self.keyframes):
                    img = cv2.imread(self.keyframes[node_idx]["path"])
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))
                cv2.rectangle(img, (0, 0), (PW-1, PH-1), (200, 200, 0), 1)
                cv2.putText(img, f"+{p+1} node {node_idx}", (4, 16),
                            FONT, 0.38, (255, 255, 255), 1, AA)
            else:
                img = np.zeros((PH, PW, 3), dtype=np.uint8)
            cells.append(img)

        row2 = cv2.hconcat(cells)
        if row2.shape[1] < panel_w:
            pad = np.zeros((PH, panel_w - row2.shape[1], 3), dtype=np.uint8)
            row2 = cv2.hconcat([row2, pad])

        panel = cv2.vconcat([bar, row1, row2])
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