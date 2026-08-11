"""
player.py  (refactored state machine)
=======================================
Changes vs previous version
----------------------------

STATE MACHINE
  Previous states : IDLE, MOVING, GOAL_REACHED
  New states added (layered on top, not replacing):
    TURN_COMMIT      — hold a committed turn for N frames; ignore conflicting
                       visual evidence during the turn itself
    CORRIDOR_FOLLOW  — robot is inside a straight corridor; skip branch
                       re-evaluation and only steer for alignment
    LOCAL_ZONE_HOLD  — slight view change detected but robot has not physically
                       moved; preserve current maneuver
    STUCK_RECOVERY   — failed action blocked; escalating recovery sequence

PRIORITY ORDERING  (enforced inside _execute_navigation)
  1. Structural constraint   — is_forced_turn()  → take the only open path
  2. Turn commitment         — TURN_COMMIT state → keep turning until complete
  3. Corridor follow         — CORRIDOR_FOLLOW   → advance, no branch decision
  4. Junction decide         — IDLE / scan       → visual + route evidence
  5. Dead end / stuck        — STUCK_RECOVERY    → block failed action, replan

NEW CONFIG KNOBS (all at the top of the file)
  TURN_COMMIT_FRAMES       — how many frames to hold a turn before reassessing
  CORRIDOR_STRAIGHT_FRAMES — max frames in corridor before forcing a recheck
  LOCAL_ZONE_THRESHOLD     — similarity gap below which we call it "same zone"
  STUCK_BLOCK_FRAMES       — how long a failed action is blocked after failure
  MAX_NODE_REVISITS        — visit count threshold before triggering recovery
  MIN_SCORE_MARGIN         — minimum gap between top two options to commit
"""

from ctypes import alignment
from turtle import fd

from vis_nav_game import Player, Action, Phase
import pygame
import tempfile
import cv2
import os
import numpy as np

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

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(BASE_DIR)

FRAME_DIR  = "/Users/demarcewilliams/Downloads/exploration_data-3/images"
GRAPH_DIR  = "/Users/demarcewilliams/Downloads/exploration_data-3/fusion_output"
GRAPH_PATH = os.path.join(GRAPH_DIR, "maze_graph.pkl")
FAISS_PATH = os.path.join(GRAPH_DIR, "keyframe_index.faiss")
META_PATH  = os.path.join(GRAPH_DIR, "keyframe_meta.pkl")

AUTO            = True
STEP            = 10
ROTATION_THRESH = 15.0
MIN_GAP         = 10
HAMMING         = 8
LOOP_THRESHOLD  = 0.85
LOOP_MIN_GAP    = 50
DINO_SIZE       = "s"

# PD controller
PD_KP     = 1.5
PD_KD     = 0.8
PD_T_TURN = 0.12

# Graph bias applied to direction preferred by the route planner
GRAPH_BIAS = 0.25

# confirm_step call interval while MOVING
CONFIRM_STEP_INTERVAL = 8

# ── NEW: state machine tuning ─────────────────────────────────────────────────

# TURN_COMMIT: how many frames to keep turning before we allow reassessment.
# Intermediate turn frames are visually unstable so we suppress re-decisions.
TURN_COMMIT_FRAMES = 12

# CORRIDOR_FOLLOW: max consecutive corridor frames before forcing a scan.
# Prevents the robot from blindly charging ahead for too long.
CORRIDOR_STRAIGHT_FRAMES = 30

# LOCAL_ZONE_HOLD: if the top scan score drops by less than this between
# consecutive IDLE frames, treat it as "same zone" and preserve the action.
LOCAL_ZONE_THRESHOLD = 0.08

# STUCK_RECOVERY: how many frames a failed direction remains blocked.
STUCK_BLOCK_FRAMES = 20

# STUCK_RECOVERY: how many times the robot may visit a node before we
# declare it stuck and force a replan from a different position.
MAX_NODE_REVISITS = 4

# Minimum score margin between top-2 options before we commit at a junction.
# Below this gap we force another scan rather than guessing.
MIN_SCORE_MARGIN = 0.06

# ─────────────────────────────────────────────────────────────────────────────

ACTION_STR_TO_ENUM = {
    "forward":    Action.FORWARD,
    "turn_left":  Action.LEFT,
    "turn_right": Action.RIGHT,
    "backward":   Action.BACKWARD,
    "stop":       Action.CHECKIN,
}

OPPOSITE = {
    "forward":    "backward",
    "backward":   "forward",
    "turn_left":  "turn_right",
    "turn_right": "turn_left",
}


class KeyboardPlayerPyGame(Player):
    def __init__(self):
        self.nav_frame_idx = 0
        self.cached_localize          = None
        self.cached_direction_scores  = None
        self.cached_alignment         = None

        self.encoder     = DINOv2Descriptor(model_size=DINO_SIZE)
        self.graph       = None
        self.index       = None
        self.descriptors = None
        self.keyframes   = []
        self._frame_count = 0

        self._goal_img_path     = None
        self._goal_pending      = True
        self._target_images_set = None

        # ── Core state machine ────────────────────────────────────────────────
        # States: IDLE | MOVING | GOAL_REACHED |
        #         TURN_COMMIT | CORRIDOR_FOLLOW | LOCAL_ZONE_HOLD | STUCK_RECOVERY
        self.state = "IDLE"

        self.navigator           = None
        self.alignment_history   = []
        self.consecutive_low_alignment = 0

        self._goal_confirm_count    = 0
        self._GOAL_CONFIRM_NEEDED   = 5

        self._prev_fpv_gray  = None
        self._cumulative_yaw = 0.0

        self.fpv      = None
        self.prev_fpv = None
        self._last_suggestion        = "IDLE"
        self._last_suggestion_detail = ""
        self.last_act = Action.IDLE
        self.screen   = None
        self.keymap   = None

        self._prev_e_k = 0.0

        # ── TURN_COMMIT state ─────────────────────────────────────────────────
        self._turn_commit_dir    = None   # "turn_left" or "turn_right"
        self._turn_commit_frames = 0      # frames remaining in commit

        # ── CORRIDOR_FOLLOW state ─────────────────────────────────────────────
        self._corridor_frames = 0         # consecutive corridor frames

        # ── LOCAL_ZONE_HOLD state ─────────────────────────────────────────────
        self._zone_hold_action  = None    # preserved action
        self._zone_hold_frames  = 0       # frames held in zone
        self._zone_prev_score   = 0.0     # best score on entry to zone
        self._zone_entry_yaw    = 0.0     # cumulative yaw when zone was entered
        self._zone_entry_hops   = None    # path length when zone was entered

        # ── STUCK_RECOVERY state ──────────────────────────────────────────────
        self._blocked_action        = None   # direction blocked after failure
        self._blocked_frames_left   = 0      # countdown until block clears
        self._stuck_escape_attempts = 0      # escalation counter
        self._recovery_action       = None   # current recovery maneuver

        super(KeyboardPlayerPyGame, self).__init__()

    # ─────────────────────────────────────────────────────────────────────────
    def reset(self):
        self.fpv      = None
        self.last_act = Action.IDLE
        self.screen   = None
        pygame.init()
        self.keymap = {
            pygame.K_LEFT:   Action.LEFT,
            pygame.K_RIGHT:  Action.RIGHT,
            pygame.K_UP:     Action.FORWARD,
            pygame.K_DOWN:   Action.BACKWARD,
            pygame.K_SPACE:  Action.CHECKIN,
            pygame.K_ESCAPE: Action.QUIT,
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
        print(f"K={K}")
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

        target_imgs             = self.get_target_images()
        self._target_images_set = target_imgs
        self._goal_pending      = True

        target_imgs = self._target_images_set or self.get_target_images()
        if target_imgs is not None and len(target_imgs) > 0:
            self._goal_img_path  = None
            self._goal_pending   = True
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            print(f"[pre_navigation] Using image at {best_path}, "
                  f"node={best_node} (combined={best_score:.4f})")
            self._goal_img_path    = best_path
            self._goal_node_direct = best_node
        else:
            print("[pre_navigation] Target images not available — "
                  "preserving existing goal state.")
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
            shape        = opencv_image.shape[1::-1]
            return pygame.image.frombuffer(opencv_image.tobytes(), shape, 'RGB')

        pygame.display.set_caption("KeyboardPlayer:fpv")
        rgb = convert_opencv_img_to_pygame(fpv)
        self.screen.blit(rgb, (0, 0))
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
            print("[see] Scoring target images (all 4 views combined)…")
            best_path, best_score, best_node = self._score_target_images(target_imgs)
            if best_path is None:
                self.prev_fpv = fpv.copy()
                return
            self._goal_img_path    = best_path
            self._goal_node_direct = best_node
            print(f"[see] Goal image: {best_path}, "
                  f"goal node: {best_node} (combined={best_score:.4f})")

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

    # =========================================================================
    #  CORE NAVIGATION DISPATCHER
    # =========================================================================

    def _execute_navigation(self):
        """
        Priority-ordered state machine dispatcher.

        Call order reflects the behavior policy from the design doc:
          1. Structural constraint  (forced turn)
          2. TURN_COMMIT            (mid-turn, keep going)
          3. GOAL_REACHED           (check in)
          4. STUCK_RECOVERY         (blocked / looping)
          5. CORRIDOR_FOLLOW        (straight corridor)
          6. LOCAL_ZONE_HOLD        (same zone, preserve action)
          7. IDLE / JUNCTION_DECIDE (active branch decision)
          8. MOVING                 (monitoring while en route)
        """
        self.nav_frame_idx += 1

        if self.navigator is None or self.fpv is None:
            return Action.IDLE

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # ── Refresh direction scan cache ──────────────────────────────────────
        if self.nav_frame_idx % 3 == 0 or self.cached_direction_scores is None:
            self.cached_direction_scores = self.navigator.scan_directions(small_fpv)

        direction_scores = self.cached_direction_scores or {}

        # ── PRIORITY 1: STRUCTURAL CONSTRAINT ────────────────────────────────
        # If only one direction is physically open, take it immediately.
        # No visual comparison. No graph bias. Just obey the geometry.
        forced, forced_dir = self.navigator.is_forced_turn(direction_scores)
        if forced and forced_dir is not None:
            action_enum = ACTION_STR_TO_ENUM.get(
                _dir_to_action(forced_dir), Action.FORWARD
            )
            print(f"[P1-STRUCTURE] ⚡ Forced: {forced_dir}")
            self._last_suggestion = _dir_to_action(forced_dir)
            # If the forced direction is a turn, commit to it
            if forced_dir in ("left", "right"):
                self._enter_turn_commit(_dir_to_action(forced_dir))
            return action_enum

        # ── PRIORITY 2: TURN_COMMIT ───────────────────────────────────────────
        if self.state == "TURN_COMMIT":
            return self._act_turn_commit()

        # ── PRIORITY 3: GOAL_REACHED ──────────────────────────────────────────
        if self.state == "GOAL_REACHED":
            print("[execute] ✓ Goal reached!")
            return Action.CHECKIN

        # ── PRIORITY 4: STUCK_RECOVERY ────────────────────────────────────────
        if self.state == "STUCK_RECOVERY":
            return self._act_stuck_recovery(direction_scores)

        # ── Record node visit for loop detection ──────────────────────────────
        if self.navigator.current_node is not None:
            self.navigator.record_visit(self.navigator.current_node)
            visits = self.navigator.get_visit_count(self.navigator.current_node)
            if visits > MAX_NODE_REVISITS and self.state != "STUCK_RECOVERY":
                print(f"[loop-detect] Node {self.navigator.current_node} "
                      f"visited {visits}x — entering STUCK_RECOVERY")
                self._enter_stuck_recovery(direction_scores)
                return self._act_stuck_recovery(direction_scores)

        # ── PRIORITY 5: CORRIDOR_FOLLOW ───────────────────────────────────────
        if self.navigator.is_corridor(direction_scores):
            self._corridor_frames += 1
            if self._corridor_frames <= CORRIDOR_STRAIGHT_FRAMES:
                if self.state != "CORRIDOR_FOLLOW":
                    print(f"[P5-CORRIDOR] Entering corridor-follow mode")
                    self.state = "CORRIDOR_FOLLOW"
                return self._act_corridor_follow(direction_scores)
            else:
                # Force a full re-evaluation after too long in corridor
                print(f"[P5-CORRIDOR] {CORRIDOR_STRAIGHT_FRAMES} frames exceeded — forcing scan")
                self._corridor_frames = 0
                self.cached_direction_scores = None
        else:
            self._corridor_frames = 0
            if self.state == "CORRIDOR_FOLLOW":
                print("[P5-CORRIDOR] Leaving corridor-follow mode")
                self.state = "IDLE"

        # ── PRIORITY 6: LOCAL_ZONE_HOLD ───────────────────────────────────────
        if self.state in ("IDLE", "MOVING", "LOCAL_ZONE_HOLD"):
            zone_action = self._check_local_zone(direction_scores)
            if zone_action is not None:
                return zone_action

        # ── PRIORITY 7 & 8: IDLE / MOVING ────────────────────────────────────
        if self.state in ("IDLE", "LOCAL_ZONE_HOLD"):
            self.state = "IDLE"
            return self._act_idle()
        if self.state == "MOVING":
            return self._act_moving()

        return Action.IDLE

    # =========================================================================
    #  STATE HANDLERS
    # =========================================================================

    # ── TURN_COMMIT ───────────────────────────────────────────────────────────

    def _enter_turn_commit(self, action_str: str):
        """Start a committed turn; suppress re-evaluation for TURN_COMMIT_FRAMES."""
        self.state               = "TURN_COMMIT"
        self._turn_commit_dir    = action_str
        self._turn_commit_frames = TURN_COMMIT_FRAMES
        print(f"[TURN_COMMIT] → {action_str} for {TURN_COMMIT_FRAMES} frames")

    def _act_turn_commit(self):
        """
        Hold the committed turn until the frame budget is exhausted OR
        alignment strongly confirms we have completed the turn.
        """
        self._turn_commit_frames -= 1
        print(f"[TURN_COMMIT] Holding {self._turn_commit_dir} "
              f"({self._turn_commit_frames} frames left)")

        if self._turn_commit_frames <= 0:
            print("[TURN_COMMIT] Turn complete — returning to IDLE")
            self.state = "IDLE"
            self.cached_direction_scores = None
            self.cached_localize         = None

        self._last_suggestion = self._turn_commit_dir
        return ACTION_STR_TO_ENUM.get(self._turn_commit_dir, Action.FORWARD)

    # ── CORRIDOR_FOLLOW ───────────────────────────────────────────────────────

    def _act_corridor_follow(self, direction_scores: dict):
        """
        Corridor mode: advance forward with light PD steering for centering.
        No branch evaluation. Only exits on junction, dead end, or goal.
        """
        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # Goal check
        if self.navigator.current_node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                return Action.CHECKIN

        # Alignment-based centering (PD on left/right visual score)
        r_score = direction_scores.get("right", {}).get("score", 0.0)
        l_score = direction_scores.get("left",  {}).get("score", 0.0)
        e_k     = r_score - l_score
        u_k     = PD_KP * e_k + PD_KD * (e_k - self._prev_e_k)
        self._prev_e_k = e_k

        # In corridor mode the PD dead-band is tighter — only steer if really off
        if u_k > PD_T_TURN * 1.5:
            action_str = "turn_right"
        elif u_k < -PD_T_TURN * 1.5:
            action_str = "turn_left"
        else:
            action_str = "forward"

        print(f"[CORRIDOR] PD u={u_k:+.3f} → {action_str}")
        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)

    # ── LOCAL_ZONE_HOLD ───────────────────────────────────────────────────────

    # Maximum heading change (degrees) allowed before the zone is considered
    # invalid.  A reversal is ~180° but we break earlier (120°) to catch the
    # robot turning back before it has fully reversed.
    _ZONE_YAW_BREAK   = 120.0

    def _check_local_zone(self, direction_scores: dict):
        """
        Detect 'same zone' condition: the best scan score has not shifted by
        more than LOCAL_ZONE_THRESHOLD since the previous IDLE decision.

        If we are still in the same local zone, preserve the previous action
        rather than re-deciding — this prevents oscillation from minor view
        shifts.

        BREAK CONDITIONS (any one triggers zone exit):
          1. Score shift exceeds LOCAL_ZONE_THRESHOLD   (original)
          2. Zone hold frame limit reached              (original)
          3. Heading reversal — yaw delta since zone    (NEW)
             entry exceeds _ZONE_YAW_BREAK degrees.
             Catches the robot turning back while the
             visual scene still looks similar.
          4. Path regression — path length has grown    (NEW)
             since zone entry, meaning the robot moved
             away from goal. Escalates directly to
             STUCK_RECOVERY rather than returning IDLE.

        Returns an Action enum to execute, or None if zone hold does not apply.
        """
        if not direction_scores:
            return None

        cur_best_score = max(
            info.get("score", 0.0) for info in direction_scores.values()
        )

        if self._zone_hold_action is None:
            self._zone_prev_score = cur_best_score
            return None

        # ── Break condition 3: heading reversal ───────────────────────────────
        yaw_delta = abs(self._cumulative_yaw - self._zone_entry_yaw)
        if yaw_delta > self._ZONE_YAW_BREAK:
            print(f"[ZONE_HOLD] ⚠️  HEADING REVERSAL detected "
                  f"(yaw_delta={yaw_delta:.1f}° > {self._ZONE_YAW_BREAK}°) "
                  f"— breaking zone hold on '{self._zone_hold_action}'")
            self._clear_zone_hold()
            return None

        # ── Break condition 4: path regression ───────────────────────────────
        cur_hops = (len(self.navigator.current_path) - 1
                    if self.navigator and self.navigator.current_path else None)
        if (self._zone_entry_hops is not None and
                cur_hops is not None and
                cur_hops > self._zone_entry_hops):
            print(f"[ZONE_HOLD] ⚠️  PATH REGRESSION detected "
                  f"(hops {self._zone_entry_hops} → {cur_hops}) "
                  f"— robot moved away from goal while in ZONE_HOLD. "
                  f"Escalating to STUCK_RECOVERY.")
            self._clear_zone_hold()
            self._enter_stuck_recovery(direction_scores)
            return self._act_stuck_recovery(direction_scores)

        # ── Break conditions 1 & 2: score shift / frame limit ─────────────────
        score_shift = abs(cur_best_score - self._zone_prev_score)

        if score_shift < LOCAL_ZONE_THRESHOLD and self._zone_hold_frames < 8:
            self._zone_hold_frames += 1
            self.state = "LOCAL_ZONE_HOLD"
            print(f"[ZONE_HOLD] Score shift={score_shift:.3f} | "
                  f"yaw_delta={yaw_delta:.1f}° | "
                  f"preserving '{self._zone_hold_action}' "
                  f"(frame {self._zone_hold_frames})")
            return ACTION_STR_TO_ENUM.get(self._zone_hold_action, Action.FORWARD)

        # Score shifted or limit reached — allow re-decision
        self._clear_zone_hold()
        self._zone_prev_score = cur_best_score
        return None

    def _enter_zone_hold(self, action_str: str, score: float):
        """
        Record the chosen action and snapshot entry state for zone-hold
        break-condition tracking.

        Replaces the old _set_zone_action() which did not capture entry yaw
        or path length, making heading-reversal and path-regression detection
        impossible.
        """
        self._zone_hold_action = action_str
        self._zone_prev_score  = score
        self._zone_hold_frames = 0
        # Snapshot the robot's heading and distance-to-goal at the moment
        # the zone is established so breaks can compare against entry state.
        self._zone_entry_yaw  = self._cumulative_yaw
        self._zone_entry_hops = (
            len(self.navigator.current_path) - 1
            if self.navigator and self.navigator.current_path else None
        )
        print(f"[ZONE_HOLD] ↓ Entered zone for '{action_str}' "
              f"(score={score:.3f}, yaw={self._zone_entry_yaw:.1f}°, "
              f"hops={self._zone_entry_hops})")

    # Keep old name as alias so any leftover calls don't crash
    def _set_zone_action(self, action_str: str, score: float):
        self._enter_zone_hold(action_str, score)

    def _clear_zone_hold(self):
        """Reset all zone-hold state cleanly."""
        self._zone_hold_action = None
        self._zone_hold_frames = 0
        self._zone_entry_yaw   = 0.0
        self._zone_entry_hops  = None

    # ── STUCK_RECOVERY ────────────────────────────────────────────────────────

    def _enter_stuck_recovery(self, direction_scores: dict):
        """
        Transition into STUCK_RECOVERY.
        Block the last failed action and choose an escape direction.
        Always clears zone-hold state so a stale held action cannot
        re-engage the moment recovery exits back to IDLE.
        """
        self.state = "STUCK_RECOVERY"

        # Clear any zone hold — the held action was part of the failure
        self._clear_zone_hold()

        # Block the direction we were just trying
        failed = self._last_suggestion
        if failed and failed != "stop":
            self._blocked_action      = failed
            self._blocked_frames_left = STUCK_BLOCK_FRAMES
            print(f"[STUCK] Blocking '{failed}' for {STUCK_BLOCK_FRAMES} frames")

        self._stuck_escape_attempts += 1
        # Choose escape: try the graph's preferred direction first
        graph_action = self.navigator.next_action()
        traversable  = self.navigator.get_traversable_directions(direction_scores)

        # Remove the blocked action from candidates
        candidates = {
            d: info for d, info in traversable.items()
            if _dir_to_action(d) != self._blocked_action
        }

        if candidates:
            best_dir = max(candidates, key=lambda d: candidates[d].get("score", 0))
            self._recovery_action = _dir_to_action(best_dir)
        elif graph_action not in ("stop", "use_radar"):
            self._recovery_action = graph_action
        else:
            self._recovery_action = "turn_right"  # last resort spin

        print(f"[STUCK] Escape attempt {self._stuck_escape_attempts}: "
              f"→ '{self._recovery_action}'")

    def _act_stuck_recovery(self, direction_scores: dict):
        """
        Execute recovery maneuver.
        Tick down the block counter; exit to IDLE when unblocked.
        """
        # Escalation: too many failed attempts → full relocalization
        if self._stuck_escape_attempts > 8:
            print("[STUCK] 🚨 Persistent failure — forcing full relocalization")
            self.cached_localize         = None
            self.cached_direction_scores = None
            self.cached_alignment        = None
            self._stuck_escape_attempts  = 0
            self._blocked_action         = None
            self._blocked_frames_left    = 0
            self._recovery_action        = None
            self.navigator.reset_visit_counts()
            self.state = "IDLE"
            return Action.IDLE

        # Tick block counter
        if self._blocked_frames_left > 0:
            self._blocked_frames_left -= 1
        else:
            # Block expired
            if self._blocked_action is not None:
                print(f"[STUCK] Block on '{self._blocked_action}' expired")
            self._blocked_action  = None
            self._recovery_action = None
            self.state            = "IDLE"
            self._stuck_escape_attempts = 0
            self.navigator.reset_visit_counts()
            print("[STUCK] Recovery complete — returning to IDLE")
            return Action.IDLE

        action_str = self._recovery_action or "turn_right"
        print(f"[STUCK] Recovery: {action_str} "
              f"({self._blocked_frames_left} frames left)")
        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.RIGHT)

    # ── _act_idle ─────────────────────────────────────────────────────────────

    def _act_idle(self):
        """
        IDLE / JUNCTION_DECIDE: full branch evaluation with graph bias and PD.
        Only reached when structural, turn-commit, corridor, and zone checks
        have all cleared.
        """
        if self.navigator._goal_node is None:
            print("[idle] ⚠️  No goal set.")
            return Action.IDLE

        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        # Localize
        if self.nav_frame_idx % 3 == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.localize_robust(small_fpv)

        current_node, confidence = self.cached_localize
        print(f"[idle] Node: {current_node} (conf={confidence:.3f})")

        if confidence > 0.70:
            self.navigator.current_node = current_node
            self.navigator.set_goal(self.navigator._goal_node)

        if not self.navigator.current_path or len(self.navigator.current_path) < 2:
            print("[idle] ⚠️  Path planning failed.")
            return Action.IDLE

        direction_scores = self.cached_direction_scores or {}

        # Optical flow corner check
        if self._is_facing_corner(self.prev_fpv, self.fpv):
            graph_action = self.navigator.next_action()
            print(f"[idle] 🔄 Corner! Map says: '{graph_action}'")
            self.prev_fpv = self.fpv.copy() if self.fpv is not None else None
            turn_str = graph_action if graph_action in ("turn_left", "turn_right") \
                else "turn_right"
            self._enter_turn_commit(turn_str)
            return ACTION_STR_TO_ENUM.get(turn_str, Action.RIGHT)

        # Dead-end / blindness
        is_blind = self.navigator.detect_dead_end(direction_scores, threshold=0.4)
        if is_blind:
            graph_action = self.navigator.next_action()
            print(f"[idle] ⚠️  Blind. Map: '{graph_action}'")
            fallback = graph_action if graph_action in ACTION_STR_TO_ENUM else "turn_right"
            if fallback in ("turn_left", "turn_right"):
                self._enter_turn_commit(fallback)
            return ACTION_STR_TO_ENUM.get(fallback, Action.RIGHT)

        # Apply graph bias to a fresh copy
        raw_scores   = {d: info.copy() for d, info in direction_scores.items()}
        graph_action = self.navigator.next_action()
        graph_dir    = _action_to_dir(graph_action)

        if graph_dir and graph_dir in raw_scores:
            old = raw_scores[graph_dir]["score"]
            raw_scores[graph_dir]["score"] = min(1.0, old + GRAPH_BIAS)
            print(f"[idle] Bias: {graph_dir} {old:.3f}→"
                  f"{raw_scores[graph_dir]['score']:.3f} (map='{graph_action}')")

        # Explicit backward
        if graph_action == "backward":
            back = raw_scores.get("back", {})
            if back and not back.get("blocked", False):
                self._transition_to_moving("backward", back.get("score", 0.0))
                return ACTION_STR_TO_ENUM["backward"]

        feasible = {
            d: info for d, info in raw_scores.items()
            if not info.get("blocked", False)
        }
        if not feasible:
            print("[idle] ⚠️  All directions blocked")
            self._enter_stuck_recovery(direction_scores)
            return self._act_stuck_recovery(direction_scores)

        # PD decision
        r_score = feasible.get("right", {}).get("score", 0.0)
        l_score = feasible.get("left",  {}).get("score", 0.0)
        f_score = feasible.get("front", {}).get("score", 0.0)

        e_k  = r_score - l_score
        u_k  = PD_KP * e_k + PD_KD * (e_k - self._prev_e_k)
        self._prev_e_k = e_k

        front_dominates = f_score > max(r_score, l_score) + 0.10

        if front_dominates:
            best_action    = "forward"
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

        # Minimum margin check at junctions
        if self.navigator.detect_junction(direction_scores):
            sorted_scores = sorted(
                [info.get("score", 0) for info in feasible.values()], reverse=True
            )
            margin = sorted_scores[0] - sorted_scores[1] if len(sorted_scores) > 1 else 1.0
            if margin < MIN_SCORE_MARGIN:
                print(f"[idle] ⚠️  Junction margin too small ({margin:.3f}) — holding")
                return Action.IDLE

        best_score = raw_scores.get(best_direction, {}).get("score", 0.0)
        print(f"[idle] PD u={u_k:+.3f} → {best_action} (score={best_score:.3f})")

        # Commit turn if the decision is a turn
        if best_action in ("turn_left", "turn_right"):
            self._enter_turn_commit(best_action)
            self._set_zone_action(best_action, best_score)
            return ACTION_STR_TO_ENUM.get(best_action, Action.FORWARD)

        self._transition_to_moving(best_action, best_score)
        self._set_zone_action(best_action, best_score)
        return ACTION_STR_TO_ENUM.get(best_action, Action.FORWARD)

    # ── _act_moving ───────────────────────────────────────────────────────────

    def _act_moving(self):
        """
        MOVING: execute chosen direction while monitoring alignment.
        Exits to IDLE (or STUCK_RECOVERY) on alignment drop / junction / dead end.
        """
        small_fpv = self._small_fpv(self.fpv, scale=0.6)

        if self.nav_frame_idx % 3 == 0 or self.cached_alignment is None:
            self.cached_alignment = self.navigator.get_alignment_scores(small_fpv)

        alignment  = self.cached_alignment
        primary    = alignment.get("primary", 0.0)
        confidence = alignment.get("route_confidence", 0.0)

        self.alignment_history.append(primary)
        if len(self.alignment_history) > 10:
            self.alignment_history.pop(0)

        print(f"[moving] Alignment: primary={primary:.3f}, conf={confidence:.3f}")

        if self.nav_frame_idx % CONFIRM_STEP_INTERVAL == 0 or self.cached_localize is None:
            self.cached_localize = self.navigator.confirm_step(small_fpv)

        node_after_step, step_score = self.cached_localize

        # Goal confirmation
        if self.navigator.current_node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
                return Action.CHECKIN
        else:
            self._goal_confirm_count = 0

        # Alignment drop
        if primary < 0.3:
            self.consecutive_low_alignment += 1
            if self.consecutive_low_alignment > 3:
                print("[moving] ⚠️  ALIGNMENT DROP — returning to IDLE")
                self._reset_moving_state()
                return Action.IDLE
        else:
            self.consecutive_low_alignment = 0

        direction_scores = self.cached_direction_scores or {}

        # Better node ahead?
        front_info = direction_scores.get("front", {})
        if front_info.get("distance", 0) > 1 and front_info.get("score", 0) > 0.8:
            print(f"[moving] Better forward node — re-evaluating")
            self._reset_moving_state()
            return Action.IDLE

        # Junction?
        if self.navigator.detect_junction(direction_scores, threshold=0.6):
            print("[moving] ⚠️  JUNCTION detected — returning to IDLE")
            self._reset_moving_state()
            return Action.IDLE

        # Corner?
        if self._is_facing_corner(self.prev_fpv, self.fpv):
            print("[moving] 🔄 Corner while moving — returning to IDLE")
            self.prev_fpv = self.fpv.copy() if self.fpv is not None else None
            self._reset_moving_state()
            return Action.IDLE

        # Dead end?
        if self.navigator.detect_dead_end(direction_scores, threshold=0.4):
            print("[moving] ⚠️  DEAD END detected")
            self._reset_moving_state()
            self._enter_stuck_recovery(direction_scores)
            return self._act_stuck_recovery(direction_scores)

        # Apply graph bias to a fresh copy for steering
        biased = {d: info.copy() for d, info in direction_scores.items()}
        graph_action = self.navigator.next_action()
        graph_dir    = _action_to_dir(graph_action)

        if graph_dir and graph_dir in biased:
            old = biased[graph_dir]["score"]
            biased[graph_dir]["score"] = min(1.0, old + GRAPH_BIAS)

        # Explicit backward
        if graph_action == "backward":
            back = biased.get("back", {})
            if back and not back.get("blocked", False):
                self._last_suggestion = "backward"
                return ACTION_STR_TO_ENUM["backward"]

        feasible = [
            (d, info) for d, info in biased.items()
            if not info.get("blocked", False)
        ]
        if feasible:
            r_score = next((i["score"] for d, i in feasible if d == "right"), 0.0)
            l_score = next((i["score"] for d, i in feasible if d == "left"),  0.0)
            f_score = next((i["score"] for d, i in feasible if d == "front"), 0.0)

            e_k  = r_score - l_score
            u_k  = PD_KP * e_k + PD_KD * (e_k - self._prev_e_k)
            self._prev_e_k = e_k

            if f_score > max(r_score, l_score) + 0.10:
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

        # If MOVING decided to turn, commit it
        if action_str in ("turn_left", "turn_right"):
            self._enter_turn_commit(action_str)
            return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)

        self._last_suggestion = action_str
        return ACTION_STR_TO_ENUM.get(action_str, Action.FORWARD)

    # ─────────────────────────────────────────────────────────────────────────
    #  INTERNAL HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _transition_to_moving(self, action_str: str, score: float):
        self.state                     = "MOVING"
        self.alignment_history         = []
        self.consecutive_low_alignment = 0
        self._last_suggestion          = action_str

    def _reset_moving_state(self):
        self.state                   = "IDLE"
        self.cached_direction_scores = None
        self.cached_alignment        = None

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
                iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
            )
            _, w  = prev_gray.shape
            mid   = w // 2
            left_mean  = float(np.mean(flow[:, :mid, 0]))
            right_mean = float(np.mean(flow[:, mid:, 0]))
            divergence = right_mean - left_mean
            if divergence > divergence_threshold:
                print(f"[flow {divergence:.2f}] 🛑 CORNER detected")
                return True
            return False
        except Exception as e:
            print(f"[flow] Corner detector failed: {e}")
            return False

    # ── _load_existing_map ────────────────────────────────────────────────────
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

    # ── _build_map_from_dataset ───────────────────────────────────────────────
    def _build_map_from_dataset(self):
        print("\n── Stage 1 & 2: Keyframe extraction")
        uniform_kfs = extract_keyframes_uniform(FRAME_DIR, step=STEP)
        turn_kfs    = detect_turns(FRAME_DIR, rotation_threshold=ROTATION_THRESH)
        print("\n── Stage 3: Merge")
        candidates   = merge_and_sort(uniform_kfs + turn_kfs)
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

    # ── _score_target_images ──────────────────────────────────────────────────
    def _score_target_images(self, target_imgs):
        view_names    = ["front", "left", "back", "right"]
        view_encodings = []

        for i, img in enumerate(target_imgs[:4]):
            fd_obj, tmp = tempfile.mkstemp(suffix=f"_{view_names[i]}.jpg")
            os.close(fd_obj)
            try:
                ok = cv2.imwrite(tmp, img)
                if not ok:
                    continue
                q = self.encoder.encode(tmp).reshape(1, -1).astype(np.float32)
                view_encodings.append((view_names[i], q, tmp))
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
            print(f"  [{vname:5s}] best: node {int(idxs[0][0])} "
                  f"(sim={float(scores[0][0]):.4f})")

        if not node_votes:
            return None, -1.0, None

        best_node     = max(node_votes, key=node_votes.get)
        best_combined = node_votes[best_node]
        print(f"  [combined] Goal node: {best_node} (combined={best_combined:.4f})")

        best_view_path, best_view_score = None, -1.0
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

    # ── show_target_images ────────────────────────────────────────────────────
    def show_target_images(self):
        targets = self.get_target_images()
        if targets is None or len(targets) <= 0:
            return
        hor1        = cv2.hconcat(targets[:2])
        hor2        = cv2.hconcat(targets[2:])
        concat_img  = cv2.vconcat([hor1, hor2])
        w, h        = concat_img.shape[:2]
        color       = (0, 0, 0)
        concat_img  = cv2.line(concat_img, (int(h/2), 0), (int(h/2), w), color, 2)
        concat_img  = cv2.line(concat_img, (0, int(w/2)), (h, int(w/2)), color, 2)
        font, line, size, stroke = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA, 0.75, 1
        w_offset, h_offset = 25, 10
        cv2.putText(concat_img, 'Front View', (h_offset, w_offset),                       font, size, color, stroke, line)
        cv2.putText(concat_img, 'Left View',  (int(h/2)+h_offset, w_offset),              font, size, color, stroke, line)
        cv2.putText(concat_img, 'Back View',  (h_offset, int(w/2)+w_offset),              font, size, color, stroke, line)
        cv2.putText(concat_img, 'Right View', (int(h/2)+h_offset, int(w/2)+w_offset),     font, size, color, stroke, line)
        cv2.imshow('KeyboardPlayer:target_images', concat_img)
        cv2.imwrite('target.jpg', concat_img)
        cv2.waitKey(1)

    # ── Manual mode helpers ───────────────────────────────────────────────────

    def _manual_update_and_display(self):
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator._goal_node is None:
            return
        small_fpv  = self._small_fpv(self.fpv, scale=0.6)
        yaw_delta  = self._estimate_heading_change(small_fpv)
        self._cumulative_yaw += yaw_delta

        node, score = self.navigator.localize_robust(small_fpv)
        self.navigator.current_node = node
        self.navigator.set_goal(self.navigator._goal_node)
        path = self.navigator.current_path or []
        hops = max(0, len(path) - 1)

        if node == self.navigator._goal_node:
            self._goal_confirm_count += 1
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                self.state = "GOAL_REACHED"
        else:
            self._goal_confirm_count = 0

        direction_scores = self.navigator.scan_directions(small_fpv)
        self.cached_direction_scores = direction_scores

        raw_action = self.navigator.next_action() if hops >= 1 else "stop"
        resolved_action, detail = self._resolve_action(raw_action, small_fpv, direction_scores)
        self._last_suggestion        = resolved_action
        self._last_suggestion_detail = detail

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
            orb  = cv2.ORB_create(nfeatures=200)
            kp1, des1 = orb.detectAndCompute(prev, None)
            kp2, des2 = orb.detectAndCompute(gray, None)
            if des1 is None or des2 is None or len(kp1) < 10:
                return 0.0
            bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            if len(matches) < 5:
                return 0.0
            dx_list = [kp2[m.trainIdx].pt[0] - kp1[m.queryIdx].pt[0] for m in matches]
            return -float(np.median(dx_list)) * (90.0 / w)
        except Exception:
            return 0.0

    def _resolve_action(self, raw_action, small_fpv, direction_scores=None):
        dir_map = {"front": "forward", "left": "turn_left",
                   "right": "turn_right", "back": "backward"}
        if raw_action == "stop":
            if self._goal_confirm_count >= self._GOAL_CONFIRM_NEEDED:
                return "stop", "GOAL CONFIRMED — press SPACE"
            return "stop", "end of path"

        if direction_scores:
            feasible = [(d, info) for d, info in direction_scores.items()
                        if not info.get("blocked", False)]
            feasible.sort(key=lambda x: -x[1]["score"])
            best_dir     = feasible[0][0] if feasible else None
            detail_parts = []
            for d in ["front", "left", "right", "back"]:
                if d in direction_scores:
                    info   = direction_scores[d]
                    marker = ">>>" if d == best_dir else "   "
                    blk    = " [WALL]" if info.get("blocked") else ""
                    detail_parts.append(f"{marker}{d}={info['score']:.2f}{blk}")
            detail = " | ".join(detail_parts)
            if feasible:
                return dir_map.get(best_dir, "forward"), detail

        return "forward", "no scan data"

    def _simplify_path(self, path):
        if not path or len(path) < 2:
            return []
        G, goal      = self.navigator.G, self.navigator._goal_node
        waypoints    = []
        MAX_STRAIGHT = 5
        last_wp_hop  = 0

        for i in range(1, len(path)):
            node = path[i]
            prev = path[i - 1]
            ed   = G.get_edge_data(prev, node)
            edge_action = (ed or {}).get("action", "forward")
            edge_type   = (ed or {}).get("edge_type", "sequential")
            n_succ      = len(list(G.successors(node)))
            is_turn     = edge_action in ("turn_left", "turn_right")
            is_loop     = edge_type == "loop_closure" or edge_action == "loop"
            is_junction = n_succ > 2
            is_goal_    = node == goal
            hops_since  = i - last_wp_hop

            if is_goal_:
                waypoints.append({"node": node, "hops_away": i, "type": "goal", "label": "GOAL"})
                break
            elif is_loop:
                waypoints.append({"node": node, "hops_away": i, "type": "loop", "label": "WARP"})
            elif is_turn:
                waypoints.append({"node": node, "hops_away": i, "type": "turn",
                                   "label": f"TURN {'LEFT' if edge_action=='turn_left' else 'RIGHT'}"})
            elif is_junction:
                waypoints.append({"node": node, "hops_away": i, "type": "junction", "label": "JUNCTION"})
            elif hops_since >= MAX_STRAIGHT:
                remaining = max(0, len(path) - 1 - i)
                waypoints.append({"node": node, "hops_away": i, "type": "checkpoint",
                                   "label": f"STRAIGHT ({remaining} left)"})
            else:
                last_wp_hop = i
                continue
            last_wp_hop = i
            if len(waypoints) >= 5:
                break

        if not waypoints and goal is not None:
            waypoints.append({"node": goal, "hops_away": max(0, len(path)-1),
                               "type": "goal", "label": "GOAL"})
        return waypoints

    def display_next_best_view(self):
        if self.fpv is None or self.navigator is None:
            return
        if self.navigator.current_node is None or self.navigator._goal_node is None:
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
        bar_h   = 60 if detail_text else 40
        bar     = np.zeros((bar_h, panel_w, 3), dtype=np.uint8)

        if   self.state == "GOAL_REACHED":  bar[:] = (0, 160, 0)
        elif self.state == "STUCK_RECOVERY": bar[:] = (0, 0, 140)
        elif self.state == "TURN_COMMIT":    bar[:] = (0, 100, 200)
        elif self.state == "CORRIDOR_FOLLOW":bar[:] = (20, 60, 20)
        elif near:                           bar[:] = (0, 0, 160)
        else:                                bar[:] = (50, 35, 15)

        heading_str = f"  hdg={self._cumulative_yaw:+.0f}°" if self._cumulative_yaw else ""
        txt = (f"Node {cur_node}  |  Goal {goal_node}"
               f"  |  {hops} hops  |  >> {next_action.upper()}"
               f"  |  [{self.state}]{heading_str}")
        cv2.putText(bar, txt, (8, 22), FONT, 0.44, (255, 255, 255), 1, AA)

        if self.state == "GOAL_REACHED":
            cv2.putText(bar, "GOAL CONFIRMED — PRESS SPACE!",
                        (panel_w-300, 22), FONT, 0.45, (0,255,255), 1, AA)
        elif near:
            cv2.putText(bar, f"NEAR GOAL ({self._goal_confirm_count}/{self._GOAL_CONFIRM_NEEDED})",
                        (panel_w-340, 22), FONT, 0.45, (0,255,255), 1, AA)

        if detail_text:
            cv2.putText(bar, detail_text, (8, 48), FONT, 0.40, (180,180,255), 1, AA)

        def thumb(img, label, color, extra=None):
            t = cv2.resize(img, (TW, TH))
            cv2.rectangle(t, (0,0), (TW-1,TH-1), color, 2)
            cv2.putText(t, label, (6,22), FONT, 0.55, color, 1, AA)
            if extra:
                cv2.putText(t, extra, (6,44), FONT, 0.45, (200,200,200), 1, AA)
            return t

        fpv_t     = thumb(self.fpv, "Live FPV", (255,255,255))
        match_img = (cv2.imread(self.keyframes[cur_node]["path"])
                     if cur_node < len(self.keyframes) else None) or \
                     np.zeros((TH, TW, 3), dtype=np.uint8)
        match_t   = thumb(match_img, f"Match: node {cur_node}", (0,255,0))
        tgt_img   = (cv2.imread(self._goal_img_path)
                     if self._goal_img_path and os.path.exists(self._goal_img_path)
                     else None) or np.zeros((TH, TW, 3), dtype=np.uint8)
        tgt_t     = thumb(tgt_img, "Goal Image", (0,140,255))
        row1      = cv2.hconcat([fpv_t, match_t, tgt_t])

        # Micro-steps row
        N_MICRO, micro_cells = 5, []
        if len(path) >= 2:
            cur_fidx   = self.keyframes[path[0]]["frame_idx"] if path[0] < len(self.keyframes) else 0
            next_fidx  = self.keyframes[path[1]]["frame_idx"] if path[1] < len(self.keyframes) else 0
            ed         = self.navigator.G.get_edge_data(path[0], path[1])
            hop_action = (ed or {}).get("action", "forward")
            if hop_action == "loop":
                hop_action = self._last_suggestion
            step_dir   = 1 if next_fidx > cur_fidx else -1
            all_inter  = list(range(cur_fidx + step_dir, next_fidx, step_dir))
            usable     = all_inter[2:] if len(all_inter) > N_MICRO + 2 else all_inter
            indices    = ([usable[int(i * len(usable) / N_MICRO)] for i in range(N_MICRO)]
                         if usable else [])
            sorted_frames = self._get_sorted_frame_list()
            for i, fidx in enumerate(indices):
                img = (cv2.imread(str(sorted_frames[fidx]))
                       if sorted_frames and 0 <= fidx < len(sorted_frames) else None) or \
                       np.zeros((PH, PW, 3), dtype=np.uint8)
                img = cv2.resize(img, (PW, PH))
                cv2.rectangle(img, (0,0), (PW-1,PH-1), (0,200,200), 1)
                cv2.putText(img, f"f{fidx} ({int(100*(i+1)/max(len(indices),1))}%)",
                            (4,16), FONT, 0.32, (255,255,255), 1, AA)
                if i == 0 and hop_action:
                    cv2.putText(img, hop_action.upper(), (4,PH-8), FONT, 0.40, (0,255,255), 1, AA)
                micro_cells.append(img)

        while len(micro_cells) < N_MICRO:
            micro_cells.append(np.zeros((PH, PW, 3), dtype=np.uint8))

        row2 = cv2.hconcat(micro_cells)
        if row2.shape[1] < panel_w:
            row2 = cv2.hconcat([row2, np.zeros((PH, panel_w-row2.shape[1], 3), dtype=np.uint8)])
        row2_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row2_label, "Micro-steps to next node", (6,14), FONT, 0.38, (0,200,200), 1, AA)

        # Waypoints row
        waypoints, N_WP, cells = self._simplify_path(path), 5, []
        TYPE_COLOR = {"turn": (0,200,255), "loop": (200,100,255), "goal": (0,255,0),
                      "junction": (0,255,255), "checkpoint": (180,180,180)}
        for p in range(N_WP):
            if p < len(waypoints):
                wp       = waypoints[p]
                ni       = wp["node"]
                img      = (cv2.imread(self.keyframes[ni]["path"])
                            if ni < len(self.keyframes) else None) or \
                            np.zeros((PH, PW, 3), dtype=np.uint8)
                img      = cv2.resize(img, (PW, PH))
                bc       = TYPE_COLOR.get(wp["type"], (200,200,0))
                cv2.rectangle(img, (0,0), (PW-1,PH-1), bc, 2)
                cv2.putText(img, f"{wp['hops_away']} hops", (4,16), FONT, 0.35, (255,255,255), 1, AA)
                cv2.putText(img, wp["label"], (4,PH-8), FONT, 0.38, bc, 1, AA)
            else:
                img = np.zeros((PH, PW, 3), dtype=np.uint8)
            cells.append(img)

        row3 = cv2.hconcat(cells)
        if row3.shape[1] < panel_w:
            row3 = cv2.hconcat([row3, np.zeros((PH, panel_w-row3.shape[1], 3), dtype=np.uint8)])
        row3_label = np.zeros((20, panel_w, 3), dtype=np.uint8)
        cv2.putText(row3_label, "Waypoints (turns · junctions · checkpoints)",
                    (6,14), FONT, 0.38, (200,200,0), 1, AA)

        panel = cv2.vconcat([bar, row1, row2_label, row2, row3_label, row3])
        cv2.imshow("Navigation Panel", panel)
        cv2.waitKey(1)
        print(f"── NAV: {next_action:<12} | Node {cur_node} → {goal_node} "
              f"| {hops} hops | [{self.state}]")


# ─────────────────────────────────────────────────────────────────────────────
# MODULE-LEVEL HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _action_to_dir(action_str: str) -> str | None:
    """Map a route action string to a scan direction key."""
    return {
        "forward":    "front",
        "turn_left":  "left",
        "turn_right": "right",
        "backward":   "back",
    }.get(action_str)


def _dir_to_action(direction: str) -> str:
    """Map a scan direction key to an action string."""
    return {
        "front": "forward",
        "left":  "turn_left",
        "right": "turn_right",
        "back":  "backward",
    }.get(direction, "forward")


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import logging
    logging.basicConfig(
        filename='vis_nav_player.log', filemode='w', level=logging.INFO,
        format='%(asctime)s - %(levelname)s: %(message)s', datefmt='%d-%b-%y %H:%M:%S',
    )
    import vis_nav_game as vng
    logging.info(f'player.py is using vis_nav_game {vng.core.__version__}')
    vng.play(the_player=KeyboardPlayerPyGame())