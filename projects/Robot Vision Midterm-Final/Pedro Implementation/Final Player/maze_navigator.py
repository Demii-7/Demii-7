"""
maze_navigator.py
=================
Runtime navigation functions imported by player.py.

Contains everything needed to localise and navigate at inference time:
    label_actions_from_commands
    save_graph
    load_graph
    localize
    localize_robust
    BayesianLocalizer              (NEW — replaces exponential decay reweighting)
    MazeNavigator                  (class — wraps localize, plan, execute)
    navigate_to_goal

Changes vs previous version:
    [NEW] BayesianLocalizer        — HMM-based belief filter over all nodes.
                                     Replaces per-frame exponential decay with
                                     accumulated probabilistic evidence. One bad
                                     frame no longer corrupts the position estimate.
    [NEW] MazeNavigator uses Bayes — localize_robust() now runs the Bayesian
                                     filter instead of raw reweighting.
    [KEEP] scan_directions         — kept but ONLY called in manual mode (Q press)
                                     for the human display panel. AUTO mode never
                                     calls it — it encodes crops which don't match
                                     full-frame keyframe descriptors reliably.
    [KEEP] All previous fixes [1]-[10] intact.
"""

import bisect
import math
import os
import pickle
import tempfile

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

    command_log is a list of (frame_idx, action_str) tuples built by
    player.py see() via infer_action_from_flow().

    FIX [10]: O(N²) scan replaced with bisect O(N log N).
    """
    if not command_log:
        for kf in keyframes:
            kf["action_to_next"] = "forward"
        return keyframes

    sorted_log  = sorted(command_log, key=lambda x: x[0])
    sorted_idxs = [entry[0] for entry in sorted_log]

    for kf in keyframes:
        fi  = kf["frame_idx"]
        pos = bisect.bisect_right(sorted_idxs, fi) - 1
        kf["action_to_next"] = sorted_log[pos][1] if pos >= 0 else "forward"

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
# 3. STANDALONE LOCALISATION FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────

def localize(query_img_path: str,
             index,
             keyframes: list[dict],
             encoder,
             current_node: int | None = None,
             graph: nx.DiGraph | None = None,
             top_k: int = 3) -> tuple[int, float]:
    """
    Find which graph node the robot is currently at.
    Kept for backward compatibility with external callers.

    FIX [1]: temporal boost was dead code; now applies neighbor boost correctly.
    """
    q = encoder.encode(query_img_path).reshape(1, -1).astype(np.float32)
    scores, idxs = index.search(q, top_k * 3)
    candidates = list(zip(idxs[0].tolist(), scores[0].tolist()))

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
    Standalone robust localiser (used outside MazeNavigator, e.g. tests).
    Graph-topology-aware: exponential decay on graph distance.

    MazeNavigator.localize_robust() uses the BayesianLocalizer instead —
    this standalone version is kept for backward compatibility.

    FIX [2]: temp files no longer leak on encoder exception.
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
    candidates = [(int(idx), float(sc))
                  for idx, sc in zip(idxs[0], scores[0]) if idx >= 0]

    if not candidates:
        return 0, 0.0

    if current_node is not None and graph is not None:
        MAX_DIST = 30
        try:
            distances = nx.single_source_shortest_path_length(
                graph, current_node, cutoff=MAX_DIST)
        except Exception:
            distances = {current_node: 0}

        SIGMA = 8.0
        reweighted = []
        for idx, vis_score in candidates:
            dist = distances.get(idx, MAX_DIST)
            if dist == 0:
                topo_weight = 1.25
            elif dist <= 2:
                topo_weight = 1.15
            else:
                topo_weight = math.exp(-dist / SIGMA)
            reweighted.append((idx, vis_score * topo_weight))

        reweighted.sort(key=lambda x: -x[1])
        return reweighted[0][0], float(reweighted[0][1])

    return candidates[0][0], float(candidates[0][1])


# ─────────────────────────────────────────────────────────────────────────────
# 4. BAYESIAN LOCALIZER  (NEW)
# ─────────────────────────────────────────────────────────────────────────────

class BayesianLocalizer:
    """
    HMM-based belief filter over all graph nodes.

    Replaces the per-frame exponential decay reweighting with a proper
    probabilistic accumulation. The key advantage: one visually ambiguous
    frame no longer corrupts the position estimate — the belief only shifts
    to a new node when *multiple consecutive frames* agree.

    Algorithm per update():
        1. Observation likelihood  — from FAISS cosine scores for top-K nodes.
                                     Unseen nodes get a small baseline (not 0)
                                     so the filter never hard-locks.
        2. Transition model        — smear current belief to graph neighbours.
                                     The robot can only move to adjacent nodes,
                                     so distant nodes get no probability mass.
        3. Bayesian update         — belief = transition × likelihood, renorm.

    Usage inside MazeNavigator:
        self.bayesian = BayesianLocalizer(n_nodes, graph)
        node, conf = self.bayesian.update(faiss_scores, faiss_indices)
    """

    STAY_PROB     = 0.55   # probability the robot stays at its current node
    MOVE_PROB     = 0.40   # probability distributed across out-neighbours
    BASELINE_OBS  = 0.02   # small non-zero likelihood for unseen nodes
    RESET_THRESH  = 0.01   # if max belief drops below this, soft-reset

    def __init__(self, n_nodes: int, graph: nx.DiGraph):
        self.n     = n_nodes
        self.G     = graph
        # Uniform prior — we don't know where we start
        self.belief = np.ones(n_nodes, dtype=np.float64) / n_nodes

        # Pre-cache out-degree for transition model speed
        self._out_degrees = np.array(
            [max(1, graph.out_degree(i)) for i in range(n_nodes)],
            dtype=np.float64,
        )

    def update(self,
               faiss_scores: np.ndarray,
               faiss_indices: np.ndarray) -> tuple[int, float]:
        """
        Run one Bayesian filter step.

        faiss_scores  : 1-D float array, cosine similarities (top-K)
        faiss_indices : 1-D int   array, matching node indices  (top-K)

        Returns (best_node_id, confidence_score).
        """
        # ── 1. Build observation likelihood ──────────────────────────────────
        likelihood = np.full(self.n, self.BASELINE_OBS, dtype=np.float64)
        for idx, sc in zip(faiss_indices, faiss_scores):
            if 0 <= idx < self.n:
                # Cosine scores can be slightly > 1.0 due to float precision
                likelihood[idx] = max(likelihood[idx], float(sc))

        # ── 2. Transition: smear belief to neighbours ─────────────────────────
        transitioned = np.zeros(self.n, dtype=np.float64)
        for node in range(self.n):
            b = self.belief[node]
            if b < 1e-9:
                continue
            # Stay component
            transitioned[node] += b * self.STAY_PROB
            # Move component — distributed evenly across successors
            move_per_nb = b * self.MOVE_PROB / self._out_degrees[node]
            for nb in self.G.successors(node):
                if 0 <= nb < self.n:
                    transitioned[nb] += move_per_nb
            # Residual probability stays at current node
            transitioned[node] += b * (1.0 - self.STAY_PROB - self.MOVE_PROB)

        # ── 3. Bayesian update ────────────────────────────────────────────────
        self.belief = transitioned * likelihood

        total = self.belief.sum()
        if total < 1e-12 or np.max(self.belief) < self.RESET_THRESH:
            # Filter has collapsed — soft reset to uniform
            print("[Bayes] Belief collapse — resetting to uniform prior.")
            self.belief = np.ones(self.n, dtype=np.float64) / self.n
        else:
            self.belief /= total

        best_node = int(np.argmax(self.belief))
        confidence = float(np.max(self.belief))
        return best_node, confidence

    def force_node(self, node: int, certainty: float = 0.90):
        """
        Force the filter to concentrate belief on a known node.
        Useful after a confirmed arrival or manual relocalization.
        """
        self.belief = np.full(self.n, (1.0 - certainty) / max(1, self.n - 1),
                              dtype=np.float64)
        if 0 <= node < self.n:
            self.belief[node] = certainty
        self.belief /= self.belief.sum()

    def reset(self):
        self.belief = np.ones(self.n, dtype=np.float64) / self.n


# ─────────────────────────────────────────────────────────────────────────────
# 5. MAZE NAVIGATOR CLASS
# ─────────────────────────────────────────────────────────────────────────────

class MazeNavigator:
    """
    Wraps localisation, path planning, and step execution.

    Lifecycle in player.py:
        pre_navigation   — constructed once after graph is built
        act() AUTO       — localize_robust(), next_action(), confirm_step()
        act() MANUAL     — scan_directions() for human display panel only

    Key design change: AUTO mode never calls scan_directions().
    scan_directions() encodes image *crops* with DINOv2, then compares them
    against keyframe descriptors that were encoded as *full frames*. This
    produces unreliable matches. AUTO mode uses:
        • next_action()   — graph edge action (pre-computed, zero compute)
        • localize_robust() — single full-frame encode + Bayesian filter
        • confirm_step()  — dot-product arrival check vs target descriptor
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

        self.current_node = None
        self.current_path = []
        self._goal_node   = None

        self.lost_count      = 0
        self._lost_candidate = None

        # NEW: Bayesian localizer replaces exponential decay reweighting
        self.bayesian = BayesianLocalizer(len(keyframes), graph)

    # ── Encoding helper ───────────────────────────────────────────────────────

    def _encode_fpv(self, fpv: np.ndarray) -> np.ndarray:
        """
        Encode a live FPV frame to a descriptor vector.
        Uses a temp file because encoder.encode() expects a file path.
        Temp file is always cleaned up.
        """
        pil = Image.fromarray(cv2.cvtColor(fpv, cv2.COLOR_BGR2RGB))
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            tmp_path = tmp.name
        pil.save(tmp_path)
        try:
            q = self.encoder.encode(tmp_path).reshape(1, -1).astype(np.float32)
        finally:
            os.unlink(tmp_path)
        return q

    # ── Localisation ──────────────────────────────────────────────────────────

    def localize(self, query_img_path: str, top_k: int = 3) -> tuple[int, float]:
        """
        Localise from a saved image file.
        Updates self.current_node and Bayesian filter.
        """
        q = self.encoder.encode(query_img_path).reshape(1, -1).astype(np.float32)
        scores, idxs = self.index.search(q, 20)
        node, conf = self.bayesian.update(scores[0], idxs[0])
        self.current_node = node
        return node, conf

    def localize_robust(self, fpv: np.ndarray) -> tuple[int, float]:
        """
        Localise from a live camera frame using the Bayesian filter.

        Steps:
          1. Single DINOv2 encode of the full frame (not crops)
          2. FAISS top-20 search
          3. Bayesian filter update — accumulates evidence over time

        Updates self.current_node.
        """
        q = self._encode_fpv(fpv)
        scores, idxs = self.index.search(q, 20)
        node, conf = self.bayesian.update(scores[0], idxs[0])
        self.current_node = node
        return node, conf

    # ── Goal setting / path planning ──────────────────────────────────────────

    def set_goal(self, goal_node_idx: int) -> list[int] | None:
        """
        Plan shortest path from current node to goal_node_idx.
        FIX [8]: nx.NodeNotFound caught explicitly.
        """
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
            print(f"[navigator] Path: {len(self.current_path) - 1} hops "
                  f"to node {goal_node_idx}")
            return self.current_path

        except (nx.NetworkXNoPath, nx.NodeNotFound,
                KeyError, TypeError, ValueError) as e:
            print(f"[navigator] Path planning failed: {type(e).__name__}: {e}")
            self.current_path = []
            return None

    def set_goal_by_image(self, goal_img_path: str) -> list[int] | None:
        """
        Set goal from a reference photo of the destination.
        """
        q = self.encoder.encode(goal_img_path).reshape(1, -1).astype(np.float32)
        scores, idxs = self.index.search(q, 1)
        goal_node = int(idxs[0][0])
        print(f"[navigator] Goal → node {goal_node} "
              f"(frame {self.keyframes[goal_node]['frame_idx']}, "
              f"sim={scores[0][0]:.3f})")
        return self.set_goal(goal_node)

    # ── Execution ─────────────────────────────────────────────────────────────

    def next_action(self) -> str:
        """
        Return the next action string from the graph edge.

        Returns:
            'stop'        — goal reached or no path
            'use_radar'   — loop-closure edge; caller resolves via keyframe flow
            'forward' | 'turn_left' | 'turn_right' | 'backward'
        """
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
        """
        Confirm the robot's position after a move and advance the path.

        Uses Bayesian localizer — the filter's accumulated belief means
        we don't flip to a wrong node on a single ambiguous frame.

        FIX [7]: replan uses actual localized position, not stale node.
        """
        new_node, score = self.localize_robust(fpv)

        if self.current_path and len(self.current_path) >= 2:

            if new_node in self.current_path[1:4]:
                # On track — advance
                idx = self.current_path.index(new_node)
                self.current_path    = self.current_path[idx:]
                self.current_node    = new_node
                self.lost_count      = 0
                self._lost_candidate = None
                # Reinforce the Bayesian filter with high certainty
                self.bayesian.force_node(new_node, certainty=0.90)

            elif not self.G.has_edge(self.current_path[0], new_node):
                self.lost_count     += 1
                self._lost_candidate = new_node

                if self.lost_count > 3:
                    print(f"[navigator] Lost for too long. "
                          f"Forcing replan from {self._lost_candidate}")
                    self.current_node = self._lost_candidate
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

    def arrival_score(self, fpv: np.ndarray, target_node: int) -> float:
        """
        Fast arrival check: encode full FPV and dot-product against a
        specific precomputed target node descriptor.

        Returns cosine similarity in [0, 1]. Threshold 0.82 = arrived.

        This is how AUTO mode confirms arrival — full frame vs full frame,
        apples to apples. No crops, no scan_directions.
        """
        if target_node >= len(self.descriptors):
            return 0.0
        q           = self._encode_fpv(fpv)
        target_desc = self.descriptors[target_node].reshape(1, -1).astype(np.float32)
        return float(np.dot(q, target_desc.T)[0, 0])

    # ── Direction scanning — MANUAL MODE ONLY ────────────────────────────────

    def scan_directions(self, fpv: np.ndarray,
                        path_segment: list = None) -> dict:
        """
        Compare directional crops against upcoming path node descriptors.

        *** IMPORTANT: Do NOT call this in AUTO mode. ***

        Crops are encoded by DINOv2 which was trained on full images —
        a left-strip crop embedding will not reliably match a full-frame
        keyframe descriptor. This function produces unreliable scores in
        ambiguous corridors and caused the wrong-path-matching bug.

        It is kept ONLY for the manual mode (Q press) display panel where
        a human interprets the scores visually. The AUTO pipeline uses
        next_action() + arrival_score() instead.

        FIX [3]: temp files cleaned up in try/finally.
        FIX [5]: Pillow Transpose enum for compatibility.
        FIX [6]: back-direction distance negated.
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
        """
        Heuristic: is this direction blocked by a wall?
        Low gradient variance = flat wall filling the view.
        """
        try:
            crop_cv  = cv2.cvtColor(np.array(crop_pil), cv2.COLOR_RGB2BGR)
            gray     = cv2.cvtColor(crop_cv, cv2.COLOR_BGR2GRAY)
            grad_x   = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            grad_y   = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            grad_mag = np.sqrt(grad_x**2 + grad_y**2)
            h, w     = gray.shape
            center   = grad_mag[h//4:3*h//4, w//4:3*w//4]
            return float(np.std(center)) < 12 and float(np.mean(center)) < 18
        except Exception:
            return False

    def detect_junction(self, direction_scores: dict,
                        threshold: float = 0.6) -> bool:
        """
        MANUAL MODE: detect if multiple paths score above threshold.
        AUTO mode uses graph node degree instead (see bestplayer.py).
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
        """
        MANUAL MODE: detect if no direction has good alignment.
        AUTO mode uses optical flow corner detection instead.
        """
        if not direction_scores:
            return True
        return max(v.get("score", 0) for v in direction_scores.values()) < threshold

    def get_alignment_scores(self, fpv: np.ndarray) -> dict:
        """
        Get alignment scores for current position vs planned route.
        Used in MANUAL mode to monitor quality while moving.

        FIX [4]: temp file deleted in try/finally.
        """
        if not self.current_path or len(self.current_path) < 2:
            return {"primary": 0.0, "alternative": 0.0, "route_confidence": 0.0}

        q = self._encode_fpv(fpv)

        next_node     = self.current_path[1] if len(self.current_path) > 1 else None
        primary_score = 0.0
        if next_node is not None and next_node < len(self.descriptors):
            next_desc     = self.descriptors[next_node].reshape(1, -1).astype(np.float32)
            primary_score = float(np.dot(q, next_desc.T)[0, 0])

        scores, _ = self.index.search(q, 5)
        alternative_score = float(scores[0][1]) if len(scores[0]) > 1 else 0.0
        confidence = primary_score if primary_score > alternative_score else 0.5

        return {
            "primary":          primary_score,
            "alternative":      alternative_score,
            "route_confidence": confidence,
        }


# ─────────────────────────────────────────────────────────────────────────────
# 6. NAVIGATE_TO_GOAL  (standalone convenience function)
# ─────────────────────────────────────────────────────────────────────────────

def navigate_to_goal(navigator: MazeNavigator,
                     robot,
                     goal_img_path: str,
                     max_steps: int = 200,
                     replan_interval: int = 10) -> bool:
    """
    Full navigation loop for use outside the player game loop (e.g. testing).

    FIX [9]: action strings replaced with Action enums; import guarded.
    """
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
        action_map = {k: k.upper() for k in
                      ["forward", "turn_left", "turn_right", "backward", "stop"]}

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
