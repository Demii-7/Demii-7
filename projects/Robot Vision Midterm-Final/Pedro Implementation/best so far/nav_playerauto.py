"""
auto_nav.py
Autonomous navigation player — full pipeline.

Offline map (built once, cached to disk)
─────────────────────────────────────────
  1. build_vlad_system      SIFT (cached) → K-Means vocab → VLAD vectors → FAISS index
  2. build_clusters         group sequential similar frames into anchor nodes
  3. build_topological_graph  VLAD retrieval → geometric verification (area spread +
                              horizontal constraint) → motion estimation (essential
                              matrix) → cluster-level edges with direction labels +
                              automatic bidirectional inverse edges
  4. save_map_system / load_map_system  FAISS native I/O + pickle metadata + JSON graph

Online localisation (background thread, every ~3 s)
────────────────────────────────────────────────────
  localize_image  VLAD → verify_geometry → cluster-level score → best cluster ID
  find_path       Dijkstra on cluster graph (distance_cost = 1/match_count)
  direction label on next edge → CMD_TURN_LEFT / CMD_TURN_RIGHT written to buffer

FSM (main thread, every frame)
────────────────────────────────
  NORMAL → GLOBAL_TURN ↔ EVASIVE (prior_state) → CONFIRMING → CHECKIN
  MANUAL  (Space toggles; C fires CHECKIN manually)

Key bindings
────────────
  Space     toggle MANUAL ↔ NORMAL
  C         CHECKIN (MANUAL only)
  Arrows    move (MANUAL only)
  Escape    quit
"""

import heapq
import json
import os
import pickle
import sys
import threading
import time
from collections import defaultdict
from enum import Enum, auto

import cv2
import faiss
import numpy as np
import pygame
from sklearn.cluster import MiniBatchKMeans
from tqdm import tqdm

import re

from vis_nav_game import Player, Action, Phase


# ══════════════════════════════════════════════════════════════════════════════
# CONFIG
# ══════════════════════════════════════════════════════════════════════════════

# VPR / map
NUM_CLUSTERS         = 64
IMG_SIZE             = (320, 240)
TOP_K                = 30
MIN_MATCH_COUNT      = 10
MAX_Y_DIFF           = 40
PROXIMITY_BLACKLIST_FRAMES = 60   # ~2 sec at 30fps; skip proximity at a
                                   # node for this long after a failed
                                   # CONFIRMING attempt, so the robot
                                   # actually moves away instead of
                                   # re-triggering from the same wall.
MIN_CLUSTER_MATCHES  = 15       # SIFT matches needed to keep two frames in same cluster
MIN_MATCH_AREA_RATIO = 0.05     # verified matches must span >= 5% of image area

# Navigation FSM
FLOOR_V_LOW       = 204         # HSV-V threshold for floor isolation
SCAN_FRAC         = 0.40        # bottom fraction of frame used for floor mask
GOOD_THRESH       = 0.35        # centre zone -> confident FORWARD
WEAK_THRESH       = 0.15        # centre zone -> cautious FORWARD
GLOBAL_FWD_THRESH = 0.40        # centre must reach this to exit GLOBAL_TURN
LOST_THRESH       = 0.05        # total floor below this -> EVASIVE
HYSTERESIS_FRAMES = 8           # FORWARD frames before GLOBAL_TURN -> NORMAL

# Background thread
VPR_STRIDE   = 5                # feed every Nth frame to VPR thread
VPR_INTERVAL = 3.0              # seconds between VPR runs

# Navigation & Graph Configs
DOWNSAMPLE_THRESHOLD = 6000
STRIDE               = 10
TEMPORAL_THRESHOLD   = 10
# (MIN_CLUSTER_MATCHES already defined above with the VPR/map constants)

# Persistence — relative to this file so it runs on any machine. Override
# with environment variables if you keep data elsewhere.
_HERE      = os.path.dirname(os.path.abspath(__file__))
MAP_FOLDER = os.environ.get("AUTONAV_MAP_FOLDER",
                            os.path.join(_HERE, "map_artifacts"))
DATA_DIR   = os.environ.get("AUTONAV_DATA_DIR",
                            os.path.join(_HERE, "trajectory_data"))

# ══════════════════════════════════════════════════════════════════════════════
# FSM STATES & COMMAND TOKENS
# ══════════════════════════════════════════════════════════════════════════════

class State(Enum):
    NORMAL      = auto()
    GLOBAL_TURN = auto()
    GLOBAL_SETTLE = auto()
    EVASIVE     = auto()
    CONFIRMING  = auto()
    MANUAL      = auto()

CMD_TURN_LEFT  = "TURN_LEFT"
CMD_TURN_RIGHT = "TURN_RIGHT"
CMD_CHECKIN    = "CHECKIN"

# ══════════════════════════════════════════════════════════════════════════════
# VPR PIPELINE  (offline + online)
# ══════════════════════════════════════════════════════════════════════════════

_sift         = cv2.SIFT_create()
FEATURE_CACHE = {}              # path -> (kp, des)  RAM cache


def _load_resize(path):
    img = cv2.imread(path)
    return cv2.resize(img, IMG_SIZE) if img is not None else None


def get_frame_idx(file_path):
    """Safely extracts the frame number from the filename."""
    filename = os.path.basename(file_path)
    name, _ = os.path.splitext(filename)
    try:
        return int(''.join(filter(str.isdigit, name)))
    except ValueError:
        return -1
    
def get_cached_features(path_or_img):
    """
    Return (kp, des) for a file path (cached to RAM) or a raw numpy frame (not cached).
    Supports both offline map building and online live-frame queries.
    """
    if isinstance(path_or_img, str):
        if path_or_img in FEATURE_CACHE:
            return FEATURE_CACHE[path_or_img]
        img = _load_resize(path_or_img)
        key = path_or_img
    else:
        img = cv2.resize(path_or_img, IMG_SIZE)
        key = None                              # never cache live frames

    if img is None:
        return None, None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kp, des = _sift.detectAndCompute(gray, None)

    if key is not None:
        FEATURE_CACHE[key] = (kp, des)
    return kp, des

def fast_sift_match(path_a, path_b):
    """Now runs instantly by pulling features from the cache."""
    kp_a, des_a = get_cached_features(path_a)
    kp_b, des_b = get_cached_features(path_b)
    
    if des_a is None or des_b is None: return 0
        
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des_a, des_b, k=2)
    
    # Safe unpacking for knnMatch
    good = []
    for m_n in matches:
        if len(m_n) == 2:
            m, n = m_n
            if m.distance < 0.75 * n.distance:
                good.append(m)
    return len(good)


def compute_vlad(des, kmeans):
    """Power-normalised, L2-normalised VLAD descriptor."""
    centers = kmeans.cluster_centers_
    k       = centers.shape[0]
    vlad    = np.zeros((k, centers.shape[1]), dtype=np.float32)
    labels  = kmeans.predict(des)
    for i, d in enumerate(des):
        vlad[labels[i]] += d - centers[labels[i]]
    vlad = vlad.flatten()
    vlad = np.sign(vlad) * np.sqrt(np.abs(vlad))   # power norm
    norm = np.linalg.norm(vlad)
    if norm > 0:
        vlad /= norm
    return vlad


# ── Offline: map building ──────────────────────────────────────────────────

def build_vlad_system(image_paths):
    """SIFT (cached) -> K-Means vocab -> VLAD vectors -> FAISS index."""
    all_des = []
    for p in tqdm(image_paths, desc="1. Feature extraction"):
        _, des = get_cached_features(p)
        if des is not None:
            all_des.append(des)

    print(f"[MAP] 2. Clustering {len(all_des)} descriptor sets -> {NUM_CLUSTERS} words...")
    kmeans = MiniBatchKMeans(n_clusters=NUM_CLUSTERS, batch_size=10000, n_init="auto")
    kmeans.fit(np.vstack(all_des))

    vectors, valid_paths = [], []
    for p in tqdm(image_paths, desc="3. Building VLAD index"):
        _, des = get_cached_features(p)
        if des is not None:
            vectors.append(compute_vlad(des, kmeans))
            valid_paths.append(p)

    vectors = np.array(vectors, dtype="float32")
    index   = faiss.IndexFlatL2(vectors.shape[1])
    index.add(vectors)
    print(f"[MAP] FAISS index: {len(valid_paths)} images.")
    return kmeans, index, valid_paths


def build_clusters(paths):
    print("\n[INFO] Grouping images into Anchor-Based Neighborhood Clusters...")
    clusters, img_to_cluster = {}, {}
    cluster_idx = 0
    
    paths_by_folder = defaultdict(list)
    for p in paths: 
        paths_by_folder[os.path.dirname(p)].append(p)
        
    for folder, f_paths in paths_by_folder.items():
        f_paths = sorted(f_paths, key=get_frame_idx)
        
        curr_cluster = [f_paths[0]]
        clusters[cluster_idx] = curr_cluster
        img_to_cluster[f_paths[0]] = cluster_idx
        
        for i in tqdm(range(1, len(f_paths)), desc=f"Clustering {os.path.basename(folder)}"):
            curr_path = f_paths[i]
            anchor_path = curr_cluster[0] 
            
            # 1. Temporal Check against Anchor
            if get_frame_idx(curr_path) - get_frame_idx(anchor_path) <= TEMPORAL_THRESHOLD:
                # 2. Visual Check against Anchor
                if fast_sift_match(anchor_path, curr_path) >= MIN_CLUSTER_MATCHES:
                    curr_cluster.append(curr_path)
                    img_to_cluster[curr_path] = cluster_idx
                    continue
            
            cluster_idx += 1
            curr_cluster = [curr_path]
            clusters[cluster_idx] = curr_cluster
            img_to_cluster[curr_path] = cluster_idx
            
        cluster_idx += 1 

    print(f"[INFO] Compressed {len(paths)} images down to {len(clusters)} Anchor Nodes.")
    return clusters, img_to_cluster

def _estimate_motion(kp_q, kp_r, matches):
    """
    Estimate relative camera motion via essential matrix (RANSAC).
    Returns dict with 'direction_str' or None if estimation fails.
    Directions: Forward / Backward / Left / Right / Minimal
    """
    if len(matches) < 5:
        return None

    pts_q = np.float32([kp_q[m.queryIdx].pt for m in matches])
    pts_r = np.float32([kp_r[m.trainIdx].pt for m in matches])

    # Approximate intrinsics for IMG_SIZE (320x240)
    K = np.array([[92, 0, 160],
                  [0,  92, 120],
                  [0,   0,   1]], dtype=np.float32)

    E, _ = cv2.findEssentialMat(pts_q, pts_r, K,
                                 method=cv2.RANSAC, threshold=1.0)
    if E is None or E.shape != (3, 3):
        return None

    _, _, t, _ = cv2.recoverPose(E, pts_q, pts_r, K)
    x, _, z    = t.flatten()

    if np.linalg.norm([x, z]) < 0.15:
        return {"direction_str": "Minimal"}
    if abs(x) > abs(z):
        direction = "Right" if x > 0 else "Left"
    else:
        direction = "Forward" if z > 0 else "Backward"
    return {"direction_str": direction}


def query_vlad(path_or_img, kmeans, index, paths):
    """VLAD query -- accepts a file path or a raw numpy frame."""
    _, des = get_cached_features(path_or_img)
    if des is None:
        return []
    vlad = compute_vlad(des, kmeans).astype("float32").reshape(1, -1)
    D, I = index.search(vlad, TOP_K)
    return [(paths[i], D[0][j]) for j, i in enumerate(I[0])]


def verify_geometry(path_or_img, vlad_results):
    """
    Geometric verification with:
      - Lowe's ratio test
      - Horizontal constraint (MAX_Y_DIFF)
      - Spatial area spread check (MIN_MATCH_AREA_RATIO)

    Accepts a file path or raw numpy frame as query.
    Returns (verified_results, kp_q).
    verified_results items include 'kp_r' and 'good_matches' for motion estimation.
    """
    kp_q, des_q = get_cached_features(path_or_img)
    if des_q is None:
        return [], None

    bf         = cv2.BFMatcher()
    total_area = IMG_SIZE[0] * IMG_SIZE[1]
    verified   = []

    for path, _ in vlad_results:
        kp_r, des_r = get_cached_features(path)
        if des_r is None:
            continue

        matches = bf.knnMatch(des_q, des_r, k=2)
        good    = [m_n[0]  for m_n in matches
                   if len(m_n) == 2 and m_n[0].distance < 0.75 * m_n[1].distance]

        # Horizontal constraint
        valid = [m for m in good
                 if abs(kp_q[m.queryIdx].pt[1] - kp_r[m.trainIdx].pt[1]) <= MAX_Y_DIFF]

        if len(valid) < MIN_MATCH_COUNT:
            continue

        # Area spread check -- reject tiny repetitive textures
        pts_q      = np.float32([kp_q[m.queryIdx].pt for m in valid])
        x_min, y_min = np.min(pts_q, axis=0)
        x_max, y_max = np.max(pts_q, axis=0)
        area_ratio   = (x_max - x_min) * (y_max - y_min) / total_area
        if area_ratio < MIN_MATCH_AREA_RATIO:
            continue

        verified.append({
            "path":         path,
            "match_count":  len(valid),
            "kp_r":         kp_r,
            "good_matches": valid,
        })

    verified.sort(key=lambda x: x["match_count"], reverse=True)
    return verified, kp_q


def build_topological_graph(kmeans, index, paths, action_map, clusters, img_to_cluster):
    print("\n[INFO] Building Spatial-Aware Topological Graph...")
    
    INVERSE = {
        "Forward": "Backward", "Backward": "Forward",
        "Left": "Right", "Right": "Left",
        "Unknown": "Unknown", "Minimal": "Minimal", "Idle": "Idle"
    }
    
    temp_edges = defaultdict(lambda: defaultdict(lambda: {
        "direction_weights": defaultdict(float),
        "total_weight": 0, "vote_count": 0, "is_sequential": False
    }))
    
    for path in tqdm(paths, desc="Mapping Connections"):
        cluster_a = img_to_cluster[path]
        folder_a = os.path.dirname(path)
        idx_a = get_frame_idx(path)
        
        vlad_results = query_vlad(path, kmeans, index, paths)[:20]
        verified_results, kp_q = verify_geometry(path, vlad_results)
        
        for res in verified_results:
            neighbor_path = res["path"]
            cluster_b = img_to_cluster[neighbor_path]
            
            if cluster_a == cluster_b: continue
                
            folder_b = os.path.dirname(neighbor_path)
            idx_b = get_frame_idx(neighbor_path)
            weight = res["match_count"]
            direction = "Unknown"
            
            is_temporally_local = ((folder_a == folder_b) and idx_a != -1 and idx_b != -1 and abs(idx_a - idx_b) <= TEMPORAL_THRESHOLD)

            # Inject Ground Truth or fallback to math
            if is_temporally_local and path in action_map:
                direction = action_map[path]
                if direction.upper() in ["IDLE", "UNKNOWN"]:
                    motion = _estimate_motion(kp_q, res["kp_r"], res["good_matches"])
                    if motion: direction = motion["direction_str"]
            else:
                motion = _estimate_motion(kp_q, res["kp_r"], res["good_matches"])
                if motion: direction = motion["direction_str"]
            
            if is_temporally_local:
                temp_edges[cluster_a][cluster_b]["is_sequential"] = True
                
            temp_edges[cluster_a][cluster_b]["direction_weights"][direction] += weight
            temp_edges[cluster_a][cluster_b]["total_weight"] += weight
            temp_edges[cluster_a][cluster_b]["vote_count"] += 1

    final_graph = defaultdict(dict)
    
    for c_a, neighbors in temp_edges.items():
        for c_b, data in neighbors.items():
            if data["vote_count"] == 0: continue 
                
            valid_dirs = {k: v for k, v in data["direction_weights"].items() if k not in ["Unknown", "Minimal"]}
            if valid_dirs:
                winning_direction = max(valid_dirs, key=valid_dirs.get)
            else:
                winning_direction = max(data["direction_weights"], key=data["direction_weights"].get) 
                
            inverse_dir = INVERSE.get(winning_direction, "Unknown")
            
            avg_weight = data["total_weight"] / data["vote_count"]
            base_cost = 1.0 / (avg_weight + 0.001)
            
            # Loop closure penalty
            distance_cost = base_cost * (1.0 if data["is_sequential"] else 3.0) 
            
            str_ca, str_cb = str(c_a), str(c_b)
            
            final_graph[str_ca][str_cb] = {
                "weight": avg_weight,
                "distance_cost": distance_cost,
                "direction": winning_direction,
                "type": "sequential" if data["is_sequential"] else "loop_closure"
            }
            
            if str_ca not in final_graph.get(str_cb, {}):
                final_graph[str_cb][str_ca] = {
                    "weight": avg_weight,
                    "distance_cost": distance_cost,
                    "direction": inverse_dir,
                    "type": "sequential" if data["is_sequential"] else "loop_closure"
                }
            
    return dict(final_graph)

# ── Persistence ────────────────────────────────────────────────────────────

def save_map_system(kmeans, index, valid_paths, clusters, img_to_cluster,
                    graph, folder=MAP_FOLDER):
    """
    Save all pipeline components.
    - metadata.pkl      : KMeans + paths + clusters (must stay in sync)
    - vlad.index        : FAISS native format (faster/smaller than pickle)
    - topological_graph.json : human-readable graph for debugging
    """
    os.makedirs(folder, exist_ok=True)

    metadata = {
        "kmeans":         kmeans,
        "valid_paths":    valid_paths,
        "clusters":       clusters,
        "img_to_cluster": img_to_cluster,
    }
    with open(os.path.join(folder, "metadata.pkl"), "wb") as f:
        pickle.dump(metadata, f)

    faiss.write_index(index, os.path.join(folder, "vlad.index"))

    with open(os.path.join(folder, "topological_graph.json"), "w") as f:
        json.dump(graph, f, indent=4)

    print(f"[MAP] Saved to '{folder}/'")


def load_map_system(folder=MAP_FOLDER):
    """Load all pipeline components from disk."""
    print(f"[MAP] Loading from '{folder}'...")

    with open(os.path.join(folder, "metadata.pkl"), "rb") as f:
        meta = pickle.load(f)

    index = faiss.read_index(os.path.join(folder, "vlad.index"))

    with open(os.path.join(folder, "topological_graph.json"), "r") as f:
        graph = json.load(f)

    print(f"[MAP] Loaded: {len(meta['valid_paths'])} images, "
          f"{len(meta['clusters'])} clusters, {len(graph)} graph nodes.")
    return (
        meta["kmeans"],
        index,
        meta["valid_paths"],
        meta["clusters"],
        meta["img_to_cluster"],
        graph,
    )


# ── Online localisation ────────────────────────────────────────────────────

def localize_image(cv2_img, kmeans, index, paths, img_to_cluster,
                    verbose=False, label="loc"):
    """
    Real-time localisation entry point.
    Accepts a raw BGR numpy frame. Returns best cluster ID (int) or None.

    Two-tier strategy:
      (1) VLAD retrieval + geometric verification. When verify_geometry
          returns matches, we trust them — they're high-confidence.
      (2) When it returns nothing (common for goal-target images that don't
          match the training distribution exactly), fall back to top-K VLAD
          voting alone. This is less precise but produces *some* answer.

    The original was strict-only and returned None on any match shortfall,
    which silently prevented target_node and start localization from ever
    succeeding. Better to take a noisy answer than no answer.
    """
    vlad_res = query_vlad(cv2_img, kmeans, index, paths)
    if not vlad_res:
        if verbose:
            print(f"[{label}] no VLAD candidates")
        return None

    # Tier 1: geometric verification
    verified, _ = verify_geometry(cv2_img, vlad_res)
    if verified:
        scores = defaultdict(float)
        for res in verified:
            c_id          = img_to_cluster[res["path"]]
            scores[c_id] += res["match_count"]
        best = max(scores, key=scores.get)
        if verbose:
            print(f"[{label}] verified -> cluster {best} "
                  f"(n_verified={len(verified)}, score={scores[best]:.1f})")
        return best

    # Tier 2: VLAD fallback. Use top-K nearest neighbors and vote on cluster.
    # Lower L2 distance = more similar.
    K_FALLBACK = 10
    cluster_votes = defaultdict(float)
    for path, dist in vlad_res[:K_FALLBACK]:
        c_id = img_to_cluster.get(path)
        if c_id is None:
            continue
        # Inverse-distance vote so closer matches count more.
        cluster_votes[c_id] += 1.0 / (dist + 1e-6)

    if not cluster_votes:
        if verbose:
            print(f"[{label}] VLAD fallback found no clusterable paths")
        return None

    best = max(cluster_votes, key=cluster_votes.get)
    if verbose:
        top3 = sorted(cluster_votes.items(),
                      key=lambda x: x[1], reverse=True)[:3]
        top3_str = ", ".join(f"c{c}={s:.2f}" for c, s in top3)
        print(f"[{label}] VLAD fallback -> cluster {best}  ({top3_str})")
    return best


def find_path(graph, start_node, target_node):
    """
    Dijkstra on the cluster graph.
    Nodes are string keys; uses 'distance_cost' (= 1/match_count) as edge weight.

    Returns:
      []   if start == target (already at goal)
      None if target is unreachable from start (graph disconnected)
      [start, ..., target] otherwise
    """
    start, target = str(start_node), str(target_node)
    if start == target:
        return []

    dist = {start: 0.0}
    prev = {}
    pq   = [(0.0, start)]

    target_reached = False
    while pq:
        d, curr = heapq.heappop(pq)
        if curr == target:
            target_reached = True
            break
        if d > dist.get(curr, float("inf")):
            continue
        for neighbour, data in graph.get(curr, {}).items():
            nd = d + data["distance_cost"]
            if nd < dist.get(neighbour, float("inf")):
                dist[neighbour] = nd
                prev[neighbour] = curr
                heapq.heappush(pq, (nd, neighbour))

    if not target_reached:
        return None  # disconnected

    path, curr = [], target
    while curr in prev:
        path.append(curr)
        curr = prev[curr]
    path.append(start)
    path.reverse()
    return path   # [start, ..., target]

# ──────────────────────────────────────────────────────────────────────────────
#  SHARED MAP STATE
# ──────────────────────────────────────────────────────────────────────────────
class MapState:
    def __init__(self):
        self._lock       = threading.Lock()
        self.node        = None
        self.path        = None
        self.target_done = False

    def update(self, node, path, target_done):
        with self._lock:
            self.node        = node
            self.path        = path
            self.target_done = target_done

    def snapshot(self):
        with self._lock:
            return {
                "node":        self.node,
                "path":        list(self.path) if self.path else None,
                "target_done": self.target_done,
            }

class DeadReckoner:
    def __init__(self):
        self.reset()

    def reset(self):
        self.forward_steps  = 0
        self.age_seconds    = 0.0
        self._last_tick     = time.time()

    def tick(self, action):
        now = time.time()
        self.age_seconds  += now - self._last_tick
        self._last_tick    = now
        if action == Action.FORWARD:
            self.forward_steps += 1
        elif action == Action.BACKWARD:
            self.forward_steps -= 1

    @property
    def confidence(self):
        time_decay = max(0.0, 1.0 - self.age_seconds / 15.0)
        step_decay = max(0.0, 1.0 - abs(self.forward_steps) / 20.0)
        return time_decay * step_decay

def sliding_cursor(path, current_node):
    if not path or current_node is None:
        return None
    for i, node in enumerate(path):
        if node == str(current_node):
            return i
    return None

# ══════════════════════════════════════════════════════════════════════════════
# AUTONOMOUS NAVIGATION PLAYER
# ══════════════════════════════════════════════════════════════════════════════

class AutoNavPlayer(Player):
    """
    Full autonomous navigation player.

    FSM: NORMAL | GLOBAL_TURN | EVASIVE | CONFIRMING | MANUAL
    Keys: Space = MANUAL toggle, C = CHECKIN (MANUAL), Arrows = move (MANUAL), Esc = quit
    """

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def __init__(self):
        # ── Visual & Threading ──
        self.fpv = None
        self.screen = None
        self._mode = "auto"
        self._quit = False
        self.last_act = Action.IDLE
        self._frame_count = 0
        self._manual_held = None

        # ── FSM & Navigation ──
        self.fsm_state = State.NORMAL
        self._prior_state = State.NORMAL
        self.global_cmd = None
        self._settle_frames = 0
        self._local_turning = None
        self._target_front_img = None
        self._target_view_descs = []
        self._proximity_best = 0
        self._proximity_blacklist = {}  # node_str -> frame_count_until
        self._near_target_hint = False  # set by VPR thread when cluster=target
        # Rolling history of recent match scores during CONFIRMING. CHECKIN
        # requires SUSTAINED strong matches across multiple frames — a real
        # arrival keeps matching strongly as the camera moves slightly,
        # while a coincidental texture match will flicker. Each entry is
        # the front_count for that frame.
        self._match_history = []  # list of recent front_count values
        self._stuck_counter = 0
        self._escape_remaining = 0
        self._turn_frames_executed = 0
        # CONFIRMING escape hatch
        self._confirm_attempts = 0

        # ── Tracking & Dead Reckoning ──
        self._map_state = MapState()
        self._dr = DeadReckoner()
        self._snap = {"node": None, "path": None, "target_done": False}
        self._edge_consumed = None
        self._same_node_repeats = 0
        
        # ── Background Communication ──
        self._frame_queue = None
        self._fq_lock = threading.Lock()

        # Map artefacts placeholders
        self._kmeans = None
        self._faiss_idx = None
        self._map_paths = None
        self._clusters = None
        self._img_to_cluster = None
        self._graph = None

        super().__init__()
        print("[AutoNav] Refined v2-Architecture Initialised.")

    def reset(self):
        pygame.init()
        self.fsm_state = State.NORMAL
        self.global_cmd = None
        self._settle_frames = 0
        self._local_turning = None
        self._mode = "auto"
        self._dr.reset()
        self._edge_consumed = None
        self._same_node_repeats = 0
        self._turn_frames_executed = 0
        self._confirm_attempts = 0
        self._proximity_best = 0
        self._proximity_blacklist = {}
        self._near_target_hint = False
        self._match_history = []
        self._stuck_counter = 0
        self._escape_remaining = 0
        self._manual_held = None
        self.last_act = Action.IDLE
        print("[AutoNav] Reset complete: FSM cleared.")
        self.fpv      = None
        self.screen   = None
        self.last_act = Action.IDLE
        pygame.init()
        print("[AutoNav] Reset complete.")

    # ── pre_exploration ───────────────────────────────────────────────────────

    def pre_exploration(self):
        """
        Build or load the map before the game starts.
        act() returns QUIT immediately when the engine enters Phase.EXPLORATION.
        """
        print("[AutoNav] pre_exploration: building / loading map...")
        self._build_or_load_map()
        print("[AutoNav] pre_exploration done. Will QUIT exploration phase immediately.")

    def _build_or_load_map(self):
        map_exists = (
            os.path.exists(MAP_FOLDER)
            and os.path.exists(os.path.join(MAP_FOLDER, "metadata.pkl"))
            and os.path.exists(os.path.join(MAP_FOLDER, "vlad.index"))
            and os.path.exists(os.path.join(MAP_FOLDER, "topological_graph.json"))
        )

        if map_exists:
            (self._kmeans, self._faiss_idx, self._map_paths,
             self._clusters, self._img_to_cluster, self._graph) = load_map_system()
            return

        all_paths = []
        action_map = {}

        if os.path.isdir(DATA_DIR):
            subdirs = sorted([os.path.join(DATA_DIR, d) for d in os.listdir(DATA_DIR) 
                              if os.path.isdir(os.path.join(DATA_DIR, d))])
            
            for folder in subdirs:
                folder_name = os.path.basename(folder)

                # 1. Load Actions (JSON) — accept either filename:
                #    - "trajectory.json"  (what player_traj.py actually writes)
                #    - "{folder_name}.json"  (legacy/alternative)
                json_candidates = [
                    os.path.join(folder, "trajectory.json"),
                    os.path.join(folder, f"{folder_name}.json"),
                ]
                json_path = next((p for p in json_candidates if os.path.exists(p)),
                                 None)
                if json_path is not None:
                    print(f"[INFO] Loading actions from {os.path.basename(json_path)}")
                    with open(json_path, 'r') as f:
                        data = json.load(f)
                        for item in data:
                            img_name = item.get("image")
                            action = item.get("action", ["UNKNOWN"])[0].title()
                            full_path = os.path.abspath(os.path.join(folder, img_name))
                            action_map[full_path] = action
                else:
                    print(f"[WARN] No action JSON found in {folder_name} — "
                          f"edge directions will rely on motion estimation only")

                # 2. Load Images with Dynamic Downsampling
                paths = sorted([
                    os.path.abspath(os.path.join(folder, f))
                    for f in os.listdir(folder)
                    if f.lower().endswith(('.png', '.jpg', '.jpeg'))
                ])
                
                num_images = len(paths)
                if num_images > DOWNSAMPLE_THRESHOLD:
                    print(f"[INFO] {folder_name} has {num_images} images. Downsampling (Stride {STRIDE})...")
                    sampled_paths = paths[::STRIDE]
                else:
                    sampled_paths = paths
                    
                all_paths.extend(sampled_paths)
        else:
            print(f"[AutoNav] WARNING: DATA_DIR '{DATA_DIR}' not found.")
            return

        print(f"[AutoNav] Building map from {len(all_paths)} carefully sampled images...")
        self._map_paths = all_paths
        
        # Build Pipeline
        self._kmeans, self._faiss_idx, self._map_paths = build_vlad_system(self._map_paths)
        
        self._clusters, self._img_to_cluster = build_clusters(self._map_paths)
        
        self._graph = build_topological_graph(
            self._kmeans, self._faiss_idx, self._map_paths, 
            action_map, self._clusters, self._img_to_cluster
        )
        
        save_map_system(
            self._kmeans, self._faiss_idx, self._map_paths,
            self._clusters, self._img_to_cluster, self._graph
        )
        
        # Global calculation speed optimisation: Free RAM for gameplay
        global FEATURE_CACHE
        FEATURE_CACHE.clear()
        print("[AutoNav] Map built. Feature Cache cleared.")

    # ── pre_navigation ────────────────────────────────────────────────────────

    def pre_navigation(self):
        print("[AutoNav] pre_navigation...")

        # 1. Target images
        images = self.get_target_images()
        self.flag = images
        if not images:
            print("[AutoNav] WARNING: no target images.")
            return
        self.show_target_images()
        self._target_front_img = images[0].copy()

        # 1b. Pre-compute SIFT descriptors for all 4 target views so we can
        #     check live FPV against each cardinal direction at the goal,
        #     not just the front. The four views are a panorama of the goal
        #     area — Left/Back/Right matches mean we're at the goal but
        #     facing a different direction, which is just as useful as
        #     matching the front.
        self._target_view_descs = []
        view_names = ["front", "right", "back", "left"]
        for i, img in enumerate(images[:4]):
            kp, des = get_cached_features(img)
            label = view_names[i] if i < 4 else f"v{i}"
            if des is None:
                print(f"[AutoNav] WARN: no SIFT descriptors for {label} target view")
                self._target_view_descs.append((label, None, None))
            else:
                self._target_view_descs.append((label, kp, des))
                print(f"[AutoNav] cached {len(des)} SIFT descriptors "
                      f"for {label} view")

        # 2. Identify target_node — vote across all 4 target views, not just
        #    the front one. The front view alone can match poorly (different
        #    rendering pose / lighting from training frames), but the
        #    accumulated evidence from all 4 cardinal views is much stronger.
        if self._kmeans is not None:
            target_votes = defaultdict(float)
            for i, img in enumerate(images[:4]):
                view_label = ["front", "right", "back", "left"][i] if i < 4 else f"v{i}"
                cluster = localize_image(
                    img, self._kmeans, self._faiss_idx,
                    self._map_paths, self._img_to_cluster,
                    verbose=True, label=f"target-{view_label}")
                if cluster is not None:
                    # Each successful view contributes one vote
                    target_votes[cluster] += 1.0

            if target_votes:
                self._target_node = max(target_votes, key=target_votes.get)
                top3 = sorted(target_votes.items(),
                              key=lambda x: x[1], reverse=True)[:3]
                top3_str = ", ".join(f"c{c}={v:.0f}" for c, v in top3)
                print(f"[AutoNav] target_node = cluster {self._target_node} "
                      f"(votes: {top3_str})")

                # Connectivity diagnostic: how many graph nodes can reach the
                # target? If this is small, Dijkstra will fail from most
                # spawn locations and the navigator falls back to wander.
                target_str = str(self._target_node)
                if target_str not in self._graph:
                    print(f"[AutoNav] CRITICAL: target cluster "
                          f"{self._target_node} is NOT in the graph! "
                          f"({len(self._graph)} graph nodes total). "
                          f"Path planning will always fail.")
                else:
                    # Count nodes from which target is reachable (reverse BFS)
                    reachable = self._count_reachable_to(target_str)
                    n_total = len(self._graph)
                    pct = 100.0 * reachable / max(n_total, 1)
                    print(f"[AutoNav] graph: target reachable from "
                          f"{reachable}/{n_total} nodes ({pct:.1f}%)")
                    if pct < 50:
                        print(f"[AutoNav] WARNING: target is poorly connected. "
                              f"Spawning in a disconnected region will leave "
                              f"the robot wandering until it stumbles into a "
                              f"connected region.")
            else:
                self._target_node = None
                print("[AutoNav] WARNING: target_node = None — "
                      "no view localized. Robot will wander only.")

        # 3. Initialise FSM
        self.fsm_state = State.NORMAL
        self._heading = 0
        self._dr.reset()

        # 4. Localise start + initial Dijkstra path
        if self.fpv is not None and self._kmeans is not None and self._target_node is not None:
            start = localize_image(
                self.fpv, self._kmeans, self._faiss_idx,
                self._map_paths, self._img_to_cluster,
                verbose=True, label="start")

            if start is not None:
                path = find_path(self._graph, start, self._target_node)
                print(f"[AutoNav] start = cluster {start}, "
                      f"path length = {len(path)} nodes")
                self._map_state.update(start, path, False)
            else:
                print("[AutoNav] WARNING: start = None — initial path empty. "
                      "VPR thread will retry every 3s.")
        elif self._target_node is None:
            print("[AutoNav] Skipping initial path: no target_node.")

        # 5. Launch background thread
        t = threading.Thread(target=self._vpr_loop, daemon=True)
        t.start()
        print("[AutoNav] Background VPR thread started.")

    # ── Background VPR / planning thread ─────────────────────────────────────

    def _vpr_loop(self):
        """Every VPR_INTERVAL seconds: localise -> re-plan Dijkstra -> write to MapState.

        The whole loop body is wrapped in a try/except so silent crashes
        become visible. Without this, an exception inside the thread (e.g.,
        an OpenCV error on a degenerate frame) just stops the thread without
        any indication, which makes the robot appear to wander purposelessly.
        """
        cycle = 0
        print("[VPR] thread alive, first cycle in 3s...")
        while True:
            try:
                time.sleep(VPR_INTERVAL)
                cycle += 1

                with self._fq_lock:
                    frame = self._frame_queue
                if frame is None or self._kmeans is None:
                    print(f"[VPR cycle {cycle}] no frame yet, skipping")
                    continue

                # Retry target_node if we never got one
                if self._target_node is None and self.flag is not None:
                    target_votes = defaultdict(float)
                    for img in self.flag[:4]:
                        c = localize_image(img, self._kmeans, self._faiss_idx,
                                            self._map_paths, self._img_to_cluster)
                        if c is not None:
                            target_votes[c] += 1.0
                    if target_votes:
                        self._target_node = max(target_votes, key=target_votes.get)
                        print(f"[VPR cycle {cycle}] target_node recovered = "
                              f"cluster {self._target_node}")

                cluster = localize_image(
                    frame, self._kmeans, self._faiss_idx,
                    self._map_paths, self._img_to_cluster)

                if cluster is None:
                    print(f"[VPR cycle {cycle}] localize FAILED — robot is in "
                          f"uncharted-looking territory")
                    continue

                # Reached goal cluster (visual similarity match)
                # IMPORTANT: cluster identity alone is too weak to fire
                # CHECKIN. The live FPV's VLAD code matched cluster
                # `target_node` — but in a maze with repeated wall textures,
                # this happens far from the goal too. Don't set
                # target_done=True. Instead set a softer flag "near_target"
                # that the act() loop can use as a hint to re-check via
                # strict multi-view SIFT (which is much stronger evidence).
                if self._target_node is not None and cluster == self._target_node:
                    print(f"[VPR cycle {cycle}] CLUSTER MATCH {cluster} == goal "
                          f"(hint only — needs SIFT verification)")
                    self._map_state.update(cluster, None, False)
                    self._near_target_hint = True
                    continue
                else:
                    self._near_target_hint = False

                # Re-plan path
                if self._target_node is not None:
                    path = find_path(self._graph, cluster, self._target_node)
                    if path is None or len(path) == 0:
                        print(f"[VPR cycle {cycle}] node={cluster} -> "
                              f"target={self._target_node}: NO PATH "
                              f"(graph disconnected)")
                        # Push the cluster anyway so HUD shows we're localized
                        self._map_state.update(cluster, None, False)
                    else:
                        print(f"[VPR cycle {cycle}] node={cluster} -> "
                              f"target={self._target_node}, path={len(path)} nodes")
                        self._map_state.update(cluster, path, False)
                else:
                    print(f"[VPR cycle {cycle}] node={cluster}, "
                          f"but target_node still None — wandering")
            except Exception as e:
                # Don't let an exception kill the thread silently.
                import traceback
                print(f"[VPR cycle {cycle}] EXCEPTION: {type(e).__name__}: {e}")
                traceback.print_exc()
                # Continue the loop; transient frame errors shouldn't be fatal.
                continue

    def _next_edge_direction(self):
        """Reads MapState to determine next turn. Handles drift and consumed edges."""
        snap = self._snap
        if not snap["path"] or snap["node"] is None:
            return None

        cursor = sliding_cursor(snap["path"], snap["node"])
        if cursor is None or cursor >= len(snap["path"]) - 1:
            return None

        src, dst = snap["path"][cursor], snap["path"][cursor + 1]
        edge_key = (src, dst)

        # Gate by consumed edges or low dead-reckoning confidence
        if self._edge_consumed == edge_key or self._dr.confidence < 0.20:
            return None

        edge = self._graph.get(str(src), {}).get(str(dst), {})
        direction = edge.get("direction", "Unknown")
        self._edge_consumed = edge_key

        if direction == "Left": return Action.LEFT
        if direction == "Right": return Action.RIGHT
        return None

    # ── see() ─────────────────────────────────────────────────────────────────

    def see(self, fpv):
        if fpv is None or fpv.ndim < 3:
            return

        self.fpv = fpv

        if self.screen is None:
            h, w, _ = fpv.shape
            self.screen = pygame.display.set_mode((w, h))
            pygame.display.set_caption("AutoNav: FPV")

        rgb  = fpv[:, :, ::-1]
        surf = pygame.image.frombuffer(rgb.tobytes(), (fpv.shape[1], fpv.shape[0]), "RGB")
        self.screen.blit(surf, (0, 0))
        pygame.display.update()

        self._floor_mask = self._get_floor_mask(fpv)

        self._frame_count += 1
        # Prime queue on frame 1 so the VPR thread doesn't wait for VPR_STRIDE
        # frames before its first localization. After that, throttle to every
        # VPR_STRIDE frames as before.
        if self._frame_count == 1 or self._frame_count % VPR_STRIDE == 0:
            with self._fq_lock:
                self._frame_queue = fpv.copy()

    # ── Perception helpers ────────────────────────────────────────────────────

    def _get_floor_mask(self, frame):
        """HSV-V threshold -> morphological clean -> binary mask (bottom SCAN_FRAC)."""
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask   = (hsv[:, :, 2] > FLOOR_V_LOW).astype(np.uint8) * 255
        h      = mask.shape[0]
        crop_y = int(h * (1.0 - SCAN_FRAC))
        roi    = mask[crop_y:, :]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        roi    = cv2.morphologyEx(roi, cv2.MORPH_OPEN,  kernel)
        roi    = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        result          = np.zeros_like(mask)
        result[crop_y:] = roi
        return result

    def _compute_zone_scores(self, mask):
        """Split floor mask into L / C / R thirds -> fraction of white pixels each."""
        h, w   = mask.shape
        crop_y = int(h * (1.0 - SCAN_FRAC))
        roi    = mask[crop_y:, :]
        if roi.size == 0:
            return 0.0, 0.0, 0.0
        third = w // 3
        rh    = roi.shape[0]
        L = np.count_nonzero(roi[:, :third])         / (rh * third)
        C = np.count_nonzero(roi[:, third:2*third])  / (rh * third)
        R = np.count_nonzero(roi[:, 2*third:])       / (rh * (w - 2*third))
        return L, C, R

    # ── act() — FSM main loop ─────────────────────────────────────────────────

    def act(self):
        # 1. THE SKIP TRIGGER (With the frame=0 safety catch)
        if hasattr(self, '_state') and self._state is not None:
            current_phase = self._state[1]
            frame_num = self._state[2] # Extract the current frame number
            
            if current_phase == Phase.EXPLORATION:
                # ONLY quit on the very first frame to prevent double-quitting
                if frame_num == 0:
                    print("[v1] Skipping Exploration: Map already loaded from dataset.")
                    return Action.QUIT
                else:
                    # If the engine is lagging on the transition, just wait safely.
                    return Action.IDLE

        if self.fpv is None: 
            return Action.IDLE

        # 1. Handle UI and Manual Mode
        pump_res = self._pump_events()
        if self._quit: return Action.QUIT
        if pump_res == Action.CHECKIN: return Action.CHECKIN
        
        if self._mode == "manual":
            return self._manual_held if self._manual_held else Action.IDLE

        # 2. Localize & Map Update (Snapshots)
        new_snap = self._map_state.snapshot()
        if new_snap["node"] != self._snap["node"] and new_snap["node"] is not None:
            print(f"[NAV] Node {self._snap['node']} -> {new_snap['node']}. Resetting DR.")
            self._dr.reset()
            self._edge_consumed = None
            self._same_node_repeats = 0
        elif (new_snap["node"] is not None
                and new_snap["node"] == self._snap["node"]
                and self._edge_consumed is not None):
            # Same node reported again after we already consumed an edge from
            # it. The turn didn't move us. After a few repeats, assume the
            # turn was lost (e.g., aborted before any rotation actually
            # happened) and clear the consumed flag so the planner can retry.
            self._same_node_repeats += 1
            if self._same_node_repeats >= 2:
                print(f"[NAV] Stuck at node {new_snap['node']} after edge "
                      f"{self._edge_consumed} consumed — clearing for retry")
                self._edge_consumed = None
                self._same_node_repeats = 0
        self._snap = new_snap

        # 3. Floor Analysis
        mask = self._get_floor_mask(self.fpv)
        L, C, R = self._compute_zone_scores(mask)

        # Corner-stuck detection. Track how many recent frames had very
        # low total floor. If it persists, force a backup+spin sequence
        # to physically break out instead of oscillating in place.
        STUCK_THRESH      = 0.35   # avg(L,C,R) below this counts as stuck
        STUCK_NEEDED      = 25     # this many recent stuck frames -> force escape
        ESCAPE_DURATION   = 12     # frames of escape behavior
        avg_floor = (L + C + R) / 3.0
        if avg_floor < STUCK_THRESH:
            self._stuck_counter = getattr(self, "_stuck_counter", 0) + 1
        else:
            self._stuck_counter = 0
        if (getattr(self, "_escape_remaining", 0) == 0
                and self._stuck_counter >= STUCK_NEEDED):
            print(f"--- [STUCK ESCAPE] {self._stuck_counter} stuck frames "
                  f"(avg={avg_floor:.2f}) — forcing backup+spin ---")
            self._escape_remaining = ESCAPE_DURATION
            self._stuck_counter = 0
        if getattr(self, "_escape_remaining", 0) > 0:
            self._escape_remaining -= 1
            # First half: back up. Second half: spin.
            if self._escape_remaining > ESCAPE_DURATION // 2:
                return Action.BACKWARD
            return Action.LEFT if (self._escape_remaining % 2 == 0) else Action.RIGHT

        # 4. State Transitions (The 'Robust' logic)

        # Throttled Heartbeat (every 10 frames)
        if self._frame_count % 10 == 0:
            path_len = len(self._snap['path']) if self._snap['path'] else 0
            conf = self._dr.confidence
            prox_best = getattr(self, "_proximity_best", 0)
            print(f"[STATUS] State: {self.fsm_state.name} | Sensors: L:{L:.2f} C:{C:.2f} R:{R:.2f} | "
                  f"Node: {self._snap['node']} | Path: {path_len} steps | DR-Conf: {conf:.2f} | "
                  f"Prox-best: {prox_best}")
        
        # Priority 1: Obstacle Safety
        if (L + C + R) < LOST_THRESH:
            if self.fsm_state != State.EVASIVE:
                print("[FSM] OBSTACLE -> EVASIVE")
                self._prior_state = self.fsm_state
                self.fsm_state = State.EVASIVE

        # Priority 2: Reached Target (v1 SIFT Endgame)
        if self._snap["target_done"]:
            if self.fsm_state != State.CONFIRMING:
                self._confirm_attempts = 0
                print("[FSM] -> CONFIRMING (target node reached)")
            self.fsm_state = State.CONFIRMING
            return self._confirming_act()

        # Priority 2.5: PROXIMITY OVERRIDE — even if the cluster localizer
        # didn't say we're at the goal yet, periodically score the live FPV
        # against ALL 4 target views. Switch to CONFIRMING only if we have
        # GOOD evidence we're at the goal:
        #   (a) one view crosses the strict MIN_MATCH_COUNT, OR
        #   (b) multiple views corroborate each other (total >= 14)
        # 
        # The previous version triggered on a single 7-point front match —
        # that's just below MIN_MATCH_COUNT and turned out to be a false
        # positive (any high-texture wall can hit 7 SIFT matches). True
        # goal proximity should light up multiple cardinal views since the
        # views are a panorama — if right/back/left are all stuck at 2,
        # we're not near the goal.
        PROXIMITY_CHECK_EVERY = 15
        PROXIMITY_STRICT_SINGLE = MIN_MATCH_COUNT     # 10 — strong single
        PROXIMITY_CORROB_TOTAL  = 14                  # combined evidence
        if (self.fsm_state == State.NORMAL
                and getattr(self, "_target_view_descs", None)
                and self._frame_count % PROXIMITY_CHECK_EVERY == 0):
            # Skip proximity if current cluster is blacklisted from a
            # recent CONFIRMING failure.
            current_node = self._snap.get("node")
            blacklist_until = self._proximity_blacklist.get(
                str(current_node) if current_node is not None else "",
                0)
            if self._frame_count < blacklist_until:
                # Quietly skip; no log spam
                pass
            else:
                view, count, all_counts = self._score_target_views(self.fpv)
                total_evidence = sum(all_counts.values()) if all_counts else 0
                # Track warmth so HUD/logs show how close we are
                self._proximity_best = max(getattr(self, "_proximity_best", 0),
                                            count)
                strong_single   = count >= PROXIMITY_STRICT_SINGLE
                corroborated    = (count >= 5 and
                                    total_evidence >= PROXIMITY_CORROB_TOTAL)
                # VPR-hint path: if the background thread says our current
                # cluster matches the target cluster AND we have at least
                # weak SIFT evidence for the front view, take a closer
                # look. Cluster match alone is too weak to fire CHECKIN
                # but is good enough to enter CONFIRMING for verification.
                vpr_hinted = (getattr(self, "_near_target_hint", False)
                              and count >= 5)
                if strong_single or corroborated or vpr_hinted:
                    if strong_single:
                        reason = "strong single"
                    elif corroborated:
                        reason = "corroborated"
                    else:
                        reason = "VPR cluster hint"
                    print(f"!!! [PROXIMITY] {reason}: {view}={count}, "
                          f"total={total_evidence}, all={all_counts} — "
                          f"switching to CONFIRMING !!!")
                    self._confirm_attempts = 0
                    self._match_history = []  # fresh evidence accumulation
                    self.fsm_state = State.CONFIRMING
                    return self._confirming_act()
                elif count >= 5:
                    # Warm but not hot — log occasionally
                    if self._frame_count % (PROXIMITY_CHECK_EVERY * 4) == 0:
                        print(f"--- [WARM] best={view} {count} pts "
                              f"total={total_evidence} (need single>="
                              f"{PROXIMITY_STRICT_SINGLE} or total>="
                              f"{PROXIMITY_CORROB_TOTAL}) ---")



        # Priority 2.6: CONFIRMING state dispatch.
        # If we got here via the proximity trigger above (and not via
        # Priority 2 / target_done), we still need to run _confirming_act
        # every frame so the timeout can tick and we can either CHECKIN
        # or fall back to NORMAL. Without this, the state is "set" but
        # wander runs underneath it.
        if self.fsm_state == State.CONFIRMING:
            return self._confirming_act()

        # Priority 3: Global Navigation
        if self.fsm_state == State.NORMAL:
            cmd = self._next_edge_direction()
            if cmd in [Action.LEFT, Action.RIGHT]:
                self.fsm_state = State.GLOBAL_TURN
                self.global_cmd = cmd
                self._local_turning = None  # Clear local wander state
                self._turn_frames_executed = 0  # Track actual rotation frames
                # Clear and meaningful message
                print(f"!!! [PLANNER] New Direction Needed: {cmd.name} !!!")
                print(f"[FSM] NORMAL -> GLOBAL_TURN")

       
        # 5. Behavior Dispatch
        # 5. Behavior Dispatch
        if self.fsm_state == State.EVASIVE:
            # If we finally see floor, recover
            if (L + C + R) > WEAK_THRESH:
                print(f"--- [RECOVERY] Floor found ({L+C+R:.2f})! Returning to {self._prior_state.name} ---")
                self.fsm_state = self._prior_state
                return Action.FORWARD

            # If totally blind (like in your log), BACK UP or SPIN HARD
            if (L + C + R) < 0.02:
                # Every 5 frames, try to backup to find perspective, otherwise spin
                if self._frame_count % 5 == 0:
                    return Action.BACKWARD
                return Action.LEFT # Hard spin to find floor

            # Otherwise, turn toward the "least bad" side
            return Action.LEFT if L >= R else Action.RIGHT
        
        if self.fsm_state == State.GLOBAL_TURN:
            # Require actual rotation before exiting on center-floor threshold.
            # Without this the turn aborts immediately when entered from a
            # corridor that already has a clear center, leaving us pointing
            # the wrong way.
            MIN_TURN_FRAMES = 4
            self._turn_frames_executed += 1
            if (self._turn_frames_executed >= MIN_TURN_FRAMES
                    and C >= GLOBAL_FWD_THRESH):
                self.fsm_state = State.GLOBAL_SETTLE
                self._settle_frames = HYSTERESIS_FRAMES
                print(f"[FSM] TURN DONE ({self._turn_frames_executed} frames)"
                      f" -> SETTLING")
                return Action.FORWARD
            # Safety: if we've been turning for a very long time without ever
            # reaching center-clear (e.g., spinning in a tight spot), bail out
            # and let NORMAL/wander handle it. Prevents infinite turning.
            if self._turn_frames_executed > 30:
                print(f"[FSM] GLOBAL_TURN timeout ({self._turn_frames_executed}"
                      f" frames) -> NORMAL")
                self.fsm_state = State.NORMAL
                self._local_turning = None
                # Don't clear _edge_consumed — it will be retried on next
                # localization if the node is still active.
                return self.global_cmd
            self._update_heading(self.global_cmd)
            return self.global_cmd

        if self.fsm_state == State.GLOBAL_SETTLE:
            self._settle_frames -= 1
            if self._settle_frames <= 0:
                self.fsm_state = State.NORMAL
                print("[FSM] SETTLE DONE -> NORMAL")
            return Action.FORWARD if C > WEAK_THRESH else (Action.LEFT if L > R else Action.RIGHT)

        # Default Behavior: NORMAL (Wander)
        # Fetch the bias from the global map
        current_bias = self._get_path_bias()
        
        # Pass it into the local planner
        action = self._wander(L, C, R, mask, global_bias=current_bias)
        
        self._dr.tick(action)
        return action
    # ── Event pump ────────────────────────────────────────────────────────────

    def _pump_events(self):
        """v2-style event pump for mode switching and manual override.
        MANUAL movement uses held-key tracking (KEYDOWN sets, KEYUP clears)
        so holding UP moves continuously instead of requiring repeated taps."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._quit = True
                return Action.QUIT

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self._quit = True
                    return Action.QUIT

                if event.key == pygame.K_SPACE:
                    self._mode = "manual" if self._mode == "auto" else "auto"
                    self.last_act = Action.IDLE
                    self._manual_held = None
                    self._local_turning = None # Clear AI memory on toggle
                    print(f"--- MODE: {self._mode.upper()} ---")

                if self._mode == "manual":
                    if event.key == pygame.K_UP:    self._manual_held = Action.FORWARD
                    if event.key == pygame.K_DOWN:  self._manual_held = Action.BACKWARD
                    if event.key == pygame.K_LEFT:  self._manual_held = Action.LEFT
                    if event.key == pygame.K_RIGHT: self._manual_held = Action.RIGHT
                    if event.key == pygame.K_c:     return Action.CHECKIN

            if event.type == pygame.KEYUP and self._mode == "manual":
                if event.key in (pygame.K_UP, pygame.K_DOWN,
                                 pygame.K_LEFT, pygame.K_RIGHT):
                    self._manual_held = None

        return None
    """
    # ── Obstacle check ────────────────────────────────────────────────────────

    def _check_obstacle(self, total_floor):
        Enter EVASIVE (saving prior_state) if floor coverage drops below threshold.
        if total_floor < LOST_THRESH and self.fsm_state  != State.EVASIVE:
            self._prior_state = self.fsm_state 
            self.fsm_state        = State.EVASIVE
            return True
        return self.fsm_state  == State.EVASIVE

    # ── Global command reader (Fix 4: atomic read+clear) ─────────────────────

    def _check_global_command(self):
        Atomically read and clear the buffer at the moment of read -- not after.
        Transitions state to GLOBAL_TURN or CONFIRMING.
        Returns True if a command was consumed.
        
        with self._buf_lock:
            cmd              = self._cmd_buffer
            self._cmd_buffer = None     # atomic clear -- Fix 4

        if cmd == CMD_TURN_LEFT:
            self._turn_dir  = Action.LEFT
            self._fwd_count = 0
            self.fsm_state      = State.GLOBAL_TURN
            return True

        if cmd == CMD_TURN_RIGHT:
            self._turn_dir  = Action.RIGHT
            self._fwd_count = 0
            self.fsm_state      = State.GLOBAL_TURN
            return True

        if cmd == CMD_CHECKIN:
            self.fsm_state  = State.CONFIRMING
            return True

        return False
    """
    # ── Local planner (NORMAL) ────────────────────────────────────────────────
        
    def _do_settle(self, L, C, R):
        """Hysteresis dwell after a global turn to prevent oscillation."""
        self._settle_frames -= 1
        if self._settle_frames <= 0:
            self.fsm_state = State.NORMAL # Changed from self.fsm_state 
            self._local_turning = None

        if C > GOOD_THRESH or abs(L - R) < 0.05:
            action = Action.FORWARD
        else:
            action = Action.LEFT if L > R else Action.RIGHT
            
        self._update_heading(action)
        return action
   
    def _wander(self, L, C, R, mask, global_bias=None):
        #total = np.count_nonzero(mask) / mask.size
        total = (L + C + R) / 3.0
        # ── THE MAGIC: APPLY SOFT GLOBAL BIAS ──
        BIAS_STRENGTH = 0.15
        forward_commit = False
        if global_bias == "Left":
            L += BIAS_STRENGTH
        elif global_bias == "Right":
            R += BIAS_STRENGTH
        elif global_bias == "Forward":
            # Path says "go straight through this edge". The previous version
            # gave no signal here because the original assumption was that
            # wander naturally prefers forward — but in practice the wander
            # pivots aggressively on small L/R differences, so a corridor
            # walk turns into oscillation. Boost C so the standard reactive
            # path picks FORWARD instead of pivoting.
            C += BIAS_STRENGTH
            forward_commit = True

        # 1. Emergency Recovery (Dead End)
        if total < LOST_THRESH:
            self._local_turning = "LEFT"
            return Action.LEFT

        # 2. STICKY LOGIC (Finish turns we already started)
        if self._local_turning is not None:
            if C > 0.60: 
                self._local_turning = None
                return Action.FORWARD
            return Action.LEFT if self._local_turning == "LEFT" else Action.RIGHT

        # 3. PRE-EMPTIVE TURN
        # When the path says Forward, raise the threshold for triggering a
        # pre-emptive turn — corridors with C around 0.4 should keep going,
        # not pivot into a wall.
        preempt_thresh = 0.30 if forward_commit else 0.40
        if total < preempt_thresh:
            self._local_turning = "LEFT" if L > R else "RIGHT"
            print(f"--- [WANDER] Pre-emptive {self._local_turning} (Floor: {total:.2f}, Bias: {global_bias}) ---")
            return Action.LEFT if self._local_turning == "LEFT" else Action.RIGHT

        # 4. Standard Reactive Navigation
        if C > GOOD_THRESH:
            return Action.FORWARD

        # When path says Forward and we have any reasonable center, COMMIT.
        # Without this, tiny L/R asymmetries (e.g. L=0.78 R=0.65 C=0.55)
        # cause us to pivot away from the corridor we should be walking.
        if forward_commit and C > 0.30:
            return Action.FORWARD

        # Dead zone to prevent jitter
        if abs(L - R) < 0.05:
            return Action.FORWARD
            
        # The ultimate choice, now heavily influenced by the map!
        if L > R:
            self._local_turning = "LEFT"
            return Action.LEFT
        else:
            self._local_turning = "RIGHT"
            return Action.RIGHT
        
    def _get_path_bias(self):
        """Peek at the Dijkstra path to figure out the next direction.
        Returns one of: 'Left', 'Right', 'Forward', or None.

        Robust to the race condition where VPR thread updated `node` to a
        cluster that isn't in the current `path` — in that case we look up
        the most recent path entry that does match and use its outgoing
        edge.
        """
        snap = self._snap
        path = snap["path"]
        node = snap["node"]
        if not path or node is None:
            return None

        node_str = str(node)
        # Search path for current node OR any path entry matching node str.
        idx = None
        for i, n in enumerate(path):
            if str(n) == node_str:
                idx = i
                break
        if idx is None:
            # Current node isn't in path — path was computed from elsewhere.
            # Use the FIRST edge of the path as a best-effort hint.
            if len(path) >= 2:
                src, dst = path[0], path[1]
            else:
                return None
        else:
            if idx >= len(path) - 1:
                return None
            src, dst = path[idx], path[idx + 1]

        edge = self._graph.get(str(src), {}).get(str(dst), {})
        return edge.get("direction", None)

    def _count_reachable_to(self, target_str):
        """Count graph nodes from which target_str is reachable.
        BFS on the reverse graph. Used as a connectivity diagnostic."""
        # Build reverse adjacency on the fly
        reverse_adj = defaultdict(list)
        for u, neighbours in self._graph.items():
            for v in neighbours:
                reverse_adj[v].append(u)

        visited = {target_str}
        queue = [target_str]
        while queue:
            curr = queue.pop()
            for pred in reverse_adj.get(curr, []):
                if pred not in visited:
                    visited.add(pred)
                    queue.append(pred)
        return len(visited)
        
    """    
    def _decide(self, L, C, R):
        if C >= GOOD_THRESH:
            action = Action.FORWARD
        elif C >= WEAK_THRESH:
            action = Action.FORWARD
        elif L >= R:
            action = Action.LEFT
        elif R > L:
            action = Action.RIGHT
        else:
            action = Action.BACKWARD
        self._update_heading(action)    # Fix 2: only here and in _global_turn_decide
        return action

    # ── GLOBAL_TURN planner ───────────────────────────────────────────────────

    def _global_turn_decide(self, C, turn_dir):
        Execute commanded turn until center floor clears, then settle.
        if C >= GLOBAL_FWD_THRESH:
            self.fsm_state  = State.GLOBAL_SETTLE
            self._settle_frames = 8
            return Action.FORWARD
        
        self._update_heading(turn_dir)
        return turn_dir
   # ── EVASIVE planner ───────────────────────────────────────────────────────

    def _evasive_decide(self, L, C, R):
        total = L + C + R
        if total >= WEAK_THRESH:
            self._restore_prior_state()
            # Fix 2: _update_heading NOT called here
            return Action.FORWARD
        if L >= R:
            return Action.LEFT
        if R > L:
            return Action.RIGHT
        return Action.BACKWARD
    """
    def _restore_prior_state(self):
        self.fsm_state = self._prior_state

    # ── CONFIRMING -- two-stage CHECKIN (Fix 3) ───────────────────────────────

    def _score_target_views(self, fpv):
        """
        Match live FPV against ALL 4 cached target views and return:
            (best_view_name, best_match_count, all_counts_dict)

        The teammate's insight: the four target views form a panorama of the
        goal area. If we're at the goal but facing the wrong way, the front
        view won't match — but Left/Back/Right will. Any strong match across
        ANY view is evidence we're at the goal.

        Returns ("none", 0, {}) when there's nothing to compare against.
        """
        if fpv is None or not getattr(self, "_target_view_descs", None):
            return "none", 0, {}

        kp_q, des_q = get_cached_features(fpv)
        if des_q is None:
            return "none", 0, {}

        bf = cv2.BFMatcher()
        all_counts = {}
        best_name, best_count = "none", 0

        for label, kp_t, des_t in self._target_view_descs:
            if des_t is None:
                all_counts[label] = 0
                continue
            try:
                raw = bf.knnMatch(des_q, des_t, k=2)
            except cv2.error:
                all_counts[label] = 0
                continue
            good = [m_n[0] for m_n in raw
                    if len(m_n) == 2
                    and m_n[0].distance < 0.75 * m_n[1].distance
                    and abs(kp_q[m_n[0].queryIdx].pt[1] -
                            kp_t[m_n[0].trainIdx].pt[1]) <= MAX_Y_DIFF]
            n = len(good)
            all_counts[label] = n
            if n > best_count:
                best_count = n
                best_name = label

        return best_name, best_count, all_counts

    def _confirming_act(self):
        """
        CHECKIN gate is intentionally strict because false CHECKINs cost
        game points and the visual matching alone is unreliable in mazes
        with repeating wall textures. Strategy:

        1. Track a rolling history of recent frame-by-frame match scores.
        2. Fire CHECKIN ONLY when:
              - Median of recent front-view matches >= STRONG threshold
                (sustained — not a single flicker), AND
              - Latest frame shows total evidence across all 4 views >=
                CORROB threshold (panorama actually visible from here).
        3. While accumulating evidence, hold position (creep slowly) so
           the camera samples slightly different angles — a real goal
           location keeps matching as we move; a false positive doesn't.
        4. If we never accumulate enough evidence within
           CONFIRM_MAX_ROTATIONS frames, blacklist this cluster from
           proximity triggers and fall back to NORMAL navigation.

        For non-front view matches (right/back/left strong), rotate
        toward the front view so we can verify. But do not CHECKIN from
        a sideways match alone — those views share textures with too
        many other parts of the maze.
        """
        CONFIRM_MAX_ROTATIONS = 24
        HISTORY_SIZE          = 6     # frames to consider for sustained match
        CHECKIN_MEDIAN_FRONT  = 14    # median front_count across history
        CHECKIN_TOTAL_NEEDED  = 24    # current-frame total across 4 views
        CHECKIN_OTHER_NEEDED  = 5     # at least one non-front view this frame

        if self.fpv is None:
            return Action.FORWARD

        best_view, best_count, all_counts = self._score_target_views(self.fpv)
        front_count = all_counts.get("front", 0)
        right_count = all_counts.get("right", 0)
        back_count  = all_counts.get("back",  0)
        left_count  = all_counts.get("left",  0)
        total_count = front_count + right_count + back_count + left_count

        # Update rolling history of front-view scores
        self._match_history.append(front_count)
        if len(self._match_history) > HISTORY_SIZE:
            self._match_history.pop(0)

        # Sustained evidence check: median (not max!) of recent frames
        # must be high. Median resists single-frame spikes.
        sorted_h = sorted(self._match_history)
        median_front = (sorted_h[len(sorted_h) // 2]
                         if len(sorted_h) >= 3 else 0)
        max_other = max(right_count, back_count, left_count)

        # Strict CHECKIN gate: all three conditions
        sustained_strong  = median_front >= CHECKIN_MEDIAN_FRONT
        total_corroborate = total_count >= CHECKIN_TOTAL_NEEDED
        other_visible     = max_other >= CHECKIN_OTHER_NEEDED

        if sustained_strong and total_corroborate and other_visible:
            print(f"!!! [CHECKIN] sustained-strong + corroborated  "
                  f"median_front={median_front} total={total_count} "
                  f"max_other={max_other}  history={self._match_history} !!!")
            return Action.CHECKIN

        # Diagnostic: explain what's blocking CHECKIN
        if self._frame_count % 15 == 0:
            blockers = []
            if not sustained_strong:
                blockers.append(f"median_front={median_front}<{CHECKIN_MEDIAN_FRONT}")
            if not total_corroborate:
                blockers.append(f"total={total_count}<{CHECKIN_TOTAL_NEEDED}")
            if not other_visible:
                blockers.append(f"max_other={max_other}<{CHECKIN_OTHER_NEEDED}")
            print(f"--- [GATE] blocked by: {', '.join(blockers)} | "
                  f"all={all_counts} attempt={self._confirm_attempts}/"
                  f"{CONFIRM_MAX_ROTATIONS} ---")

        # Decide what to do while gathering more evidence.
        # If front is the best AND it's at least moderately matching, creep
        # forward to gather evidence from a slightly different vantage —
        # don't rotate away from a possibly correct heading.
        if best_view == "front" and front_count >= MIN_MATCH_COUNT:
            self._confirm_attempts += 1
            return Action.FORWARD

        # If a non-front view matches strongly, rotate toward front.
        if best_count >= MIN_MATCH_COUNT and best_view != "front":
            self._confirm_attempts += 1
            if best_view == "right":
                return Action.LEFT      # see right -> turn LEFT to face front
            if best_view == "left":
                return Action.RIGHT     # see left  -> turn RIGHT to face front
            return Action.RIGHT         # back: 180°, either dir works

        # Nothing matched strongly: rotate slowly and keep looking
        self._confirm_attempts += 1

        if self._confirm_attempts >= CONFIRM_MAX_ROTATIONS:
            print(f"--- [CONFIRM TIMEOUT] {self._confirm_attempts} rotations, "
                  f"never accumulated CHECKIN evidence. "
                  f"Last counts: {all_counts}. Falling back to NORMAL ---")
            self._confirm_attempts = 0
            self._match_history = []  # discard stale history
            self.fsm_state = State.NORMAL
            current_node = self._snap.get("node")
            if current_node is not None:
                self._proximity_blacklist[str(current_node)] = (
                    self._frame_count + PROXIMITY_BLACKLIST_FRAMES)
                print(f"--- [BLACKLIST] node {current_node} blocked from "
                      f"proximity for {PROXIMITY_BLACKLIST_FRAMES} frames ---")
            self._map_state.update(self._snap.get("node"),
                                    self._snap.get("path"), False)
            return Action.FORWARD

        return Action.RIGHT
        
    # ── Heading tracker (Fix 2) ───────────────────────────────────────────────

    def _update_heading(self, action):
        """±90 per intentional LEFT/RIGHT. NOT called from _evasive_decide."""
        if action == Action.LEFT:
            self._heading = (self._heading - 90) % 360
        elif action == Action.RIGHT:
            self._heading = (self._heading + 90) % 360

    # ── UI helpers ────────────────────────────────────────────────────────────

    def show_target_images(self):
        targets = self.get_target_images()
        if not targets:
            return
        while len(targets) < 4:
            targets.append(np.zeros_like(targets[0]))

        hor1       = cv2.hconcat(targets[:2])
        hor2       = cv2.hconcat(targets[2:4])
        concat_img = cv2.vconcat([hor1, hor2])
        h, w       = concat_img.shape[:2]
        col        = (0, 0, 0)
        font, sz, st, ln = cv2.FONT_HERSHEY_SIMPLEX, 0.75, 1, cv2.LINE_AA

        concat_img = cv2.line(concat_img, (w//2, 0),  (w//2, h),  col, 2)
        concat_img = cv2.line(concat_img, (0, h//2),  (w, h//2),  col, 2)
        cv2.putText(concat_img, "Front View",  (10, 25),           font, sz, col, st, ln)
        cv2.putText(concat_img, "Left View",   (w//2+10, 25),      font, sz, col, st, ln)
        cv2.putText(concat_img, "Back View",   (10, h//2+25),      font, sz, col, st, ln)
        cv2.putText(concat_img, "Right View",  (w//2+10, h//2+25), font, sz, col, st, ln)

        cv2.imshow("AutoNav: target_images", concat_img)
        cv2.imwrite("target.jpg", concat_img)
        cv2.waitKey(1)

    def set_target_images(self, images):
        super().set_target_images(images)
        self.show_target_images()

    # ── Phase-guard state hook ────────────────────────────────────────────────

    def _set_game_state(self, state):
        self._game_state = state


# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import logging
    import vis_nav_game as vng

    logging.basicConfig(
        filename="auto_nav.log",
        filemode="w",
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s: %(message)s",
        datefmt="%d-%b-%y %H:%M:%S",
    )
    logging.info(f"auto_nav.py using vis_nav_game {vng.core.__version__}")
    vng.play(the_player=AutoNavPlayer())