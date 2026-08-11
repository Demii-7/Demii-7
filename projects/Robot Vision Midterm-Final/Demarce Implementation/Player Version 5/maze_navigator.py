"""
maze_navigator.py  (refactored)
================================
Runtime navigation functions imported by player.py.

Changes vs previous version
----------------------------
[N1]  get_traversable_directions()
        Returns only physically open directions from scan_directions results.
        Centralises the "blocked?" check so every caller uses the same logic.

[N2]  is_forced_turn()
        Returns (True, direction_str) when exactly one option is traversable
        and it is not forward.  This is the highest-priority structural rule:
        if only one direction is open, take it — no visual comparison needed.

[N3]  is_corridor()
        Returns True when the traversable set is {front} only, or {front} plus
        at most one side that scores well below the forward score.  Signals
        that the robot is inside a straight corridor and should not re-decide.

[N4]  record_visit() / get_visit_count()
        Lightweight node visit counter for loop / revisit detection.
        Called by player.py's STUCK_RECOVERY logic.

[N5]  reset_visit_counts()
        Clears all visit counts — called when a new goal is set so stale
        loop-detection history does not bleed across navigation targets.

[N6]  scan_directions() — now returns fresh dict copies (was already patched
        in previous version; confirmed here).

[N7]  confirm_step() — lost_candidate fix retained from previous version.

All previous fixes [1]–[10] are preserved unchanged.
"""

import bisect
import math
import os
import pickle
import tempfile
from collections import defaultdict

import cv2
import networkx as nx
import numpy as np
from PIL import Image
from build_graph import load_graph  # single source of truth


# ─────────────────────────────────────────────────────────────────────────────
# 1. ACTION LABELLING
# ─────────────────────────────────────────────────────────────────────────────

def label_actions_from_commands(keyframes: list[dict],
                                command_log: list[tuple]) -> list[dict]:
    """
    Attach an 'action_to_next' field to each keyframe using the command log
    recorded during exploration.

    FIX [10]: O(N²) scan replaced with bisect O(N log N).
    """
    if not command_log:
        for kf in keyframes:
            kf["action_to_next"] = "forward"
        return keyframes

    sorted_log   = sorted(command_log, key=lambda x: x[0])
    sorted_idxs  = [entry[0] for entry in sorted_log]

    for kf in keyframes:
        fi  = kf["frame_idx"]
        pos = bisect.bisect_right(sorted_idxs, fi) - 1
        if pos >= 0:
            kf["action_to_next"] = sorted_log[pos][1]
        else:
            kf["action_to_next"] = "forward"

    return keyframes


# ─────────────────────────────────────────────────────────────────────────────
# 2. GRAPH PERSISTENCE
# ─────────────────────────────────────────────────────────────────────────────

def save_graph(graph: nx.DiGraph, path: str = "maze_graph.pkl") -> None:
    with open(path, "wb") as f:
        pickle.dump(graph, f)
    size = os.path.getsize(path) / 1024
    print(f"[save_graph]  → {path}  ({size:.1f} KB)")


# ─────────────────────────────────────────────────────────────────────────────
# 3. LOCALISATION  (standalone functions)
# ─────────────────────────────────────────────────────────────────────────────

def localize(query_img_path: str,
             index,
             keyframes: list[dict],
             encoder,
             current_node: int | None = None,
             graph: nx.DiGraph | None = None,
             top_k: int = 3) -> tuple[int, float]:
    """FIX [1]: temporal boost was dead code — now applied correctly."""
    q = encoder.encode(query_img_path).reshape(1, -1).astype(np.float32)
    scores, idxs = index.search(q, top_k * 3)
    candidates   = list(zip(idxs[0].tolist(), scores[0].tolist()))

    if current_node is not None and graph is not None:
        neighbors = (
            set(graph.successors(current_node)) |
            set(graph.predecessors(current_node)) |
            {current_node}
        )
        candidates = [
            (idx, sc * 1.15 if idx in neighbors else sc)
            for idx, sc in candidates
        ]
        candidates.sort(key=lambda x: -x[1])

    return int(candidates[0][0]), float(candidates[0][1])


def localize_robust(query_fpv: np.ndarray,
                    index,
                    keyframes: list[dict],
                    encoder,
                    current_node: int | None = None,
                    graph: nx.DiGraph | None = None) -> tuple[int, float]:
    """
    Graph-topology-aware localisation with exponential distance decay.
    FIX [2]: temp file always deleted in finally block.
    """
    pil = Image.fromarray(cv2.cvtColor(query_fpv, cv2.COLOR_BGR2RGB))

    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        tmp_path = tmp.name
    pil.save(tmp_path)
    try:
        q = encoder.encode(tmp_path).reshape(1, -1).astype(np.float32)
    finally:
        os.unlink(tmp_path)

    scores, idxs = index.search(q, 20)
    candidates   = [(int(idx), float(sc)) for idx, sc in zip(idxs[0], scores[0]) if idx >= 0]

    if not candidates:
        return 0, 0.0

    if current_node is not None and graph is not None:
        MAX_DIST = 30
        try:
            distances = nx.single_source_shortest_path_length(
                graph, current_node, cutoff=MAX_DIST
            )
        except Exception:
            distances = {current_node: 0}

        SIGMA      = 8.0
        reweighted = []
        for idx, vis_score in candidates:
            dist = distances.get(idx, MAX_DIST)
            if   dist == 0: topo_weight = 1.25
            elif dist <= 2: topo_weight = 1.15
            else:           topo_weight = math.exp(-dist / SIGMA)
            reweighted.append((idx, vis_score * topo_weight))

        reweighted.sort(key=lambda x: -x[1])
        return reweighted[0][0], float(reweighted[0][1])

    return candidates[0][0], float(candidates[0][1])


# ─────────────────────────────────────────────────────────────────────────────
# 4. MAZE NAVIGATOR CLASS
# ─────────────────────────────────────────────────────────────────────────────

class MazeNavigator:
    """
    Wraps localisation, path planning, and step execution.

    New public API (additions marked [N1]–[N5]):
        get_traversable_directions(direction_scores) → dict   [N1]
        is_forced_turn(direction_scores) → (bool, str|None)  [N2]
        is_corridor(direction_scores) → bool                  [N3]
        record_visit(node)                                    [N4]
        get_visit_count(node) → int                           [N4]
        reset_visit_counts()                                  [N5]
    """

    def __init__(self,
                 graph: nx.DiGraph,
                 index,
                 keyframes: list[dict],
                 descriptors: np.ndarray,
                 encoder):

        self.G           = graph
        self.index       = index
        self.keyframes   = keyframes
        self.descriptors = descriptors
        self.encoder     = encoder

        self.current_node    = None
        self.current_path    = []
        self._goal_node      = None

        self.lost_count      = 0
        self._lost_candidate = None

        # [N4] visit tracking for loop / revisit detection
        self._visit_counts: dict[int, int] = defaultdict(int)

    # ── Internal localise ─────────────────────────────────────────────────────

    def localize(self, query_img_path: str, top_k: int = 3) -> tuple[int, float]:
        q = self.encoder.encode(query_img_path).reshape(1, -1).astype(np.float32)
        scores, idxs = self.index.search(q, top_k * 3)
        candidates   = list(zip(idxs[0].tolist(), scores[0].tolist()))

        if self.current_node is not None:
            neighbors = (
                set(self.G.successors(self.current_node)) |
                set(self.G.predecessors(self.current_node)) |
                {self.current_node}
            )
            candidates = [
                (idx, sc * 1.15 if idx in neighbors else sc)
                for idx, sc in candidates
            ]
            candidates.sort(key=lambda x: -x[1])

        best_node        = int(candidates[0][0])
        best_score       = float(candidates[0][1])
        self.current_node = best_node
        return best_node, best_score

    def localize_robust(self, fpv: np.ndarray) -> tuple[int, float]:
        node, score       = localize_robust(
            fpv, self.index, self.keyframes, self.encoder,
            current_node=self.current_node,
            graph=self.G,
        )
        self.current_node = node
        return node, score

    # ── Goal / planning ───────────────────────────────────────────────────────

    def set_goal(self, goal_node_idx: int) -> list[int] | None:
        """FIX [8]: nx.NodeNotFound added to except tuple."""
        if goal_node_idx is None:
            print("[navigator] Cannot plan: goal_node_idx is None.")
            self.current_path = []
            return None
        if self.current_node is None:
            print("[navigator] Must localise before planning.")
            self.current_path = []
            return None
        try:
            path = nx.shortest_path(
                self.G,
                source=self.current_node,
                target=goal_node_idx,
                weight="weight",
            )
            self.current_path = list(path)
            self._goal_node   = goal_node_idx
            # [N5] reset visit counts when we commit to a new goal
            self.reset_visit_counts()
            print(f"[navigator] Path: {len(self.current_path) - 1} hops "
                  f"to node {goal_node_idx}")
            return self.current_path
        except (nx.NetworkXNoPath, nx.NodeNotFound,
                KeyError, TypeError, ValueError) as e:
            print(f"[navigator] Path planning failed: {type(e).__name__}: {e}")
            self.current_path = []
            return None

    def set_goal_by_image(self, goal_img_path: str) -> list[int] | None:
        q = self.encoder.encode(goal_img_path).reshape(1, -1).astype(np.float32)
        scores, idxs = self.index.search(q, 1)
        goal_node    = int(idxs[0][0])
        print(f"[navigator] Goal → node {goal_node} "
              f"(frame {self.keyframes[goal_node]['frame_idx']}, "
              f"sim={scores[0][0]:.3f})")
        return self.set_goal(goal_node)

    # ── Execution ─────────────────────────────────────────────────────────────

    def next_action(self) -> str:
        if not self.current_path or len(self.current_path) < 2:
            return "stop"
        src  = self.current_path[0]
        dest = self.current_path[1]
        edge = self.G.get_edge_data(src, dest)
        if edge is None:
            return "stop"
        if edge.get("edge_type") == "loop_closure" or edge.get("action") == "loop":
            return "use_radar"
        return edge.get("action", "forward")

    def confirm_step(self, fpv: np.ndarray) -> tuple[int, float]:
        """FIX [7]: replan uses actual localized position."""
        new_node, score = self.localize_robust(fpv)

        if self.current_path and len(self.current_path) >= 2:
            if new_node in self.current_path[1:4]:
                idx               = self.current_path.index(new_node)
                self.current_path = self.current_path[idx:]
                self.current_node = new_node
                self.lost_count   = 0
                self._lost_candidate = None
            elif not self.G.has_edge(self.current_path[0], new_node):
                self.lost_count      += 1
                self._lost_candidate  = new_node
                if self.lost_count > 3:
                    print(f"[navigator] Lost for too long. "
                          f"Forcing replan from {self._lost_candidate}")
                    self.current_node    = self._lost_candidate
                    self.set_goal(self._goal_node)
                    self.lost_count      = 0
                    self._lost_candidate = None
                else:
                    self.current_node = self.current_path[0]
            else:
                self.current_node    = new_node
                self.lost_count      = 0
                self._lost_candidate = None

        return (self.current_node, score)

    # ─────────────────────────────────────────────────────────────────────────
    # [N1]  get_traversable_directions
    # ─────────────────────────────────────────────────────────────────────────

    def get_traversable_directions(self, direction_scores: dict) -> dict:
        """
        [N1] Return only directions that are NOT physically blocked.

        This is the canonical "what can I actually do?" query.  Every caller
        in player.py that previously did its own `not info.get("blocked")`
        filter should use this instead to guarantee consistency.

        Returns a dict with the same structure as direction_scores but
        containing only traversable entries.
        """
        return {
            d: info
            for d, info in direction_scores.items()
            if not info.get("blocked", False)
        }

    # ─────────────────────────────────────────────────────────────────────────
    # [N2]  is_forced_turn
    # ─────────────────────────────────────────────────────────────────────────

    def is_forced_turn(self,
                       direction_scores: dict) -> tuple[bool, str | None]:
        """
        [N2] Structural constraint check — highest priority in the pipeline.

        If exactly ONE direction is traversable (open), the robot has no
        choice: it must take that direction.  Visual scores are irrelevant.

        This correctly handles:
          • T-junctions where one arm is blocked
          • Corridor ends that force a single turn
          • Any geometry where only one exit exists

        Returns:
            (True,  direction_str)  — forced, take this direction
            (False, None)           — more than one option available

        Note: "forward" counts as a valid traversable option.  If forward
        is the only option that is not blocked, this returns (True, "forward")
        so the caller knows it is a forced move, not a free choice.
        """
        traversable = self.get_traversable_directions(direction_scores)
        if len(traversable) == 1:
            direction = next(iter(traversable))
            print(f"[structure] ⚡ Forced direction: '{direction}' "
                  f"(only traversable option)")
            return True, direction
        return False, None

    # ─────────────────────────────────────────────────────────────────────────
    # [N3]  is_corridor
    # ─────────────────────────────────────────────────────────────────────────

    def is_corridor(self,
                    direction_scores: dict,
                    side_score_gap: float = 0.15) -> bool:
        """
        [N3] Detect whether the robot is inside a straight corridor.

        A corridor means: forward is traversable AND side options either do
        not exist or score significantly lower than forward.  In this state
        the robot should NOT re-evaluate branch decisions — it just follows.

        Corridor conditions (ALL must hold):
          1. "front" is in traversable directions
          2. No side direction (left/right) has a score within `side_score_gap`
             of the front score AND also has a distance <= 2 hops
             (i.e. no imminent junction branch is visible ahead)

        side_score_gap: how much lower a side must score vs front to be
                        considered a corridor wall, not a branch.
                        Default 0.15 means the side must score at least 0.15
                        worse than front.

        Returns True = corridor, robot should keep moving without re-deciding.
        """
        traversable = self.get_traversable_directions(direction_scores)

        if "front" not in traversable:
            return False

        front_score = traversable["front"].get("score", 0.0)

        for d in ("left", "right"):
            if d not in traversable:
                continue
            side_info  = traversable[d]
            side_score = side_info.get("score", 0.0)
            side_dist  = side_info.get("distance", 99)

            # A side direction is a genuine branch candidate if:
            #   • it scores close to forward AND
            #   • there is a node relatively close in that direction
            if (side_score >= front_score - side_score_gap and
                    side_dist <= 3):
                return False   # genuine junction / branch visible

        return True

    # ─────────────────────────────────────────────────────────────────────────
    # [N4]  visit tracking
    # ─────────────────────────────────────────────────────────────────────────

    def record_visit(self, node: int) -> None:
        """[N4] Increment the visit counter for a node."""
        if node is not None:
            self._visit_counts[node] += 1

    def get_visit_count(self, node: int) -> int:
        """[N4] Return how many times the robot has visited this node."""
        return self._visit_counts.get(node, 0)

    # ─────────────────────────────────────────────────────────────────────────
    # [N5]  reset_visit_counts
    # ─────────────────────────────────────────────────────────────────────────

    def reset_visit_counts(self) -> None:
        """[N5] Clear all visit counters — called when a new goal is planned."""
        self._visit_counts.clear()

    # ── Direction scanning and advanced detection ─────────────────────────────

    def extract_directional_crops(self, fpv: np.ndarray) -> dict:
        """
        FIX [5]: Image.ROTATE_180 → Image.Transpose.ROTATE_180.
        """
        h, w = fpv.shape[:2]
        pil  = Image.fromarray(cv2.cvtColor(fpv, cv2.COLOR_BGR2RGB))
        return {
            "front": pil.crop((w // 4, h // 4, 3 * w // 4, 3 * h // 4)),
            "left":  pil.crop((0,      h // 4, w // 2,      3 * h // 4)),
            "right": pil.crop((w // 2, h // 4, w,           3 * h // 4)),
            "back":  pil.transpose(Image.Transpose.ROTATE_180),
        }

    def scan_directions(self, fpv: np.ndarray,
                        path_segment: list = None) -> dict:
        """
        Compare directional crops against upcoming path node descriptors.
        Returns fresh dict objects each call (safe to mutate in caller).
        """
        if not isinstance(self.current_path, list):
            self.current_path = []
        if not self.current_path or len(self.current_path) < 2:
            return {}
        if path_segment is None:
            segment_len  = min(7, len(self.current_path))
            path_segment = list(self.current_path[1:segment_len])

        h, w = fpv.shape[:2]
        pil  = Image.fromarray(cv2.cvtColor(fpv, cv2.COLOR_BGR2RGB))

        crops = {
            "front": pil.crop((w // 6, h // 6, 5 * w // 6, 5 * h // 6)),
            "left":  pil.crop((0,      h // 6, w // 2,      5 * h // 6)),
            "right": pil.crop((w // 2, h // 6, w,           5 * h // 6)),
        }

        results = {}
        for direction, crop in crops.items():
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                tmp_path = tmp.name
            crop.save(tmp_path)
            try:
                q = self.encoder.encode(tmp_path).reshape(1, -1).astype(np.float32)
            finally:
                os.unlink(tmp_path)

            best_node, best_score, best_distance = None, -1.0, 0
            for dist, node_idx in enumerate(path_segment, start=1):
                if node_idx < len(self.descriptors):
                    desc       = self.descriptors[node_idx].reshape(1, -1).astype(np.float32)
                    similarity = float(np.dot(q, desc.T)[0, 0])
                    if similarity > best_score:
                        best_score    = similarity
                        best_node     = node_idx
                        best_distance = dist

            if best_node is not None:
                is_blocked = self._check_if_blocked(crop)
                results[direction] = dict(
                    node     = best_node,
                    score    = best_score if not is_blocked else best_score * 0.3,
                    distance = best_distance,
                    blocked  = is_blocked,
                )

        return results

    def _check_if_blocked(self, crop_pil: Image.Image) -> bool:
        try:
            crop_cv  = cv2.cvtColor(np.array(crop_pil), cv2.COLOR_RGB2BGR)
            gray     = cv2.cvtColor(crop_cv, cv2.COLOR_BGR2GRAY)
            grad_x   = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            grad_y   = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            grad_mag = np.sqrt(grad_x**2 + grad_y**2)
            h, w     = gray.shape
            center   = grad_mag[h//4:3*h//4, w//4:3*w//4]
            grad_std  = float(np.std(center))
            grad_mean = float(np.mean(center))
            return grad_std < 12 and grad_mean < 18
        except Exception:
            return False

    def detect_junction(self, direction_scores: dict,
                        threshold: float = 0.6) -> bool:
        """
        Returns True if >= 2 unique destination nodes score above threshold.
        """
        strong_nodes = set()
        for v in direction_scores.values():
            if v.get("score", 0) > threshold:
                node_id = v.get("node")
                if node_id is not None:
                    strong_nodes.add(node_id)
        return len(strong_nodes) >= 2

    def detect_dead_end(self, direction_scores: dict,
                        threshold: float = 0.4) -> bool:
        """Returns True if all directions score below threshold."""
        if not direction_scores:
            return True
        max_score = max(v.get("score", 0) for v in direction_scores.values())
        return max_score < threshold

    def get_alignment_scores(self, fpv: np.ndarray) -> dict:
        """FIX [4]: temp file deleted in finally block."""
        if not self.current_path or len(self.current_path) < 2:
            return {"primary": 0.0, "alternative": 0.0, "route_confidence": 0.0}

        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            tmp_path = tmp.name
            img      = Image.fromarray(cv2.cvtColor(fpv, cv2.COLOR_BGR2RGB))
            img.save(tmp_path)
        try:
            q = self.encoder.encode(tmp_path).reshape(1, -1).astype(np.float32)
        finally:
            os.unlink(tmp_path)

        next_node     = self.current_path[1] if len(self.current_path) > 1 else None
        primary_score = 0.0
        if next_node is not None and next_node < len(self.descriptors):
            next_desc     = self.descriptors[next_node].reshape(1, -1).astype(np.float32)
            primary_score = float(np.dot(q, next_desc.T)[0, 0])

        scores, _         = self.index.search(q, 5)
        alternative_score = float(scores[0][1]) if len(scores[0]) > 1 else 0.0
        confidence        = primary_score if primary_score > alternative_score else 0.5

        return {
            "primary":          primary_score,
            "alternative":      alternative_score,
            "route_confidence": confidence,
        }


# ─────────────────────────────────────────────────────────────────────────────
# 5. NAVIGATE_TO_GOAL  (standalone convenience function)
# ─────────────────────────────────────────────────────────────────────────────

def navigate_to_goal(navigator: MazeNavigator,
                     robot,
                     goal_img_path: str,
                     max_steps: int = 200,
                     replan_interval: int = 10) -> bool:
    """FIX [9]: uses Action enums with graceful fallback."""
    try:
        from vis_nav_game import Action
        action_map = {
            "forward":    Action.FORWARD,
            "turn_left":  Action.LEFT,
            "turn_right": Action.RIGHT,
            "backward":   Action.BACKWARD,
            "stop":       Action.CHECKIN,
        }
    except ImportError:
        print("[navigate]  vis_nav_game not found — using string actions")
        action_map = {
            "forward":    "FORWARD",
            "turn_left":  "LEFT",
            "turn_right": "RIGHT",
            "backward":   "BACKWARD",
            "stop":       "CHECKIN",
        }

    frame       = robot.capture_frame()
    node, score = navigator.localize_robust(frame)
    print(f"[navigate]  Start node {node} (confidence {score:.3f})")

    path = navigator.set_goal_by_image(goal_img_path)
    if path is None:
        print("[navigate]  Cannot reach goal — check graph connectivity.")
        return False

    for step in range(max_steps):
        action_str = navigator.next_action()
        if action_str == "stop":
            print(f"[navigate]  Goal reached in {step} steps.")
            return True

        robot.send_action(action_map.get(action_str, action_map["forward"]))
        frame = robot.capture_frame()
        navigator.confirm_step(frame)

        if step % replan_interval == 0 and navigator._goal_node is not None:
            navigator.set_goal(navigator._goal_node)

    print(f"[navigate]  Max steps ({max_steps}) reached.")
    return False
