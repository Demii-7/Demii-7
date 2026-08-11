"""
Baseline Level-1: VLAD-based visual navigation.

Pipeline: RootSIFT → KMeans codebook → VLAD → cosine similarity graph → Dijkstra
"""

from vis_nav_game import Player, Action, Phase
import pygame
import cv2
import numpy as np
import os
import json
import pickle
import networkx as nx
from sklearn.cluster import KMeans
from tqdm import tqdm

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CACHE_DIR = "cache"
DATA_DIR = "data/exploration_data"

# Graph construction
TEMPORAL_WEIGHT = 1.0
VISUAL_WEIGHT_BASE = 2.0
VISUAL_WEIGHT_SCALE = 3.0
MIN_SHORTCUT_GAP = 50

os.makedirs(CACHE_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# VLAD Feature Extraction
# ---------------------------------------------------------------------------
class VLADExtractor:
    """RootSIFT + VLAD with intra-normalization and power normalization."""

    def __init__(self, n_clusters: int = 128):
        self.n_clusters = n_clusters
        self.sift = cv2.SIFT_create()
        self.codebook = None
        self._sift_cache: dict[str, np.ndarray] = {}

    @property
    def dim(self) -> int:
        return self.n_clusters * 128

    @property
    def descriptor_dim(self) -> int:
        return 128

    def _sift_cache_file(self, subsample_rate: int) -> str:
        return os.path.join(
            CACHE_DIR,
            f"rootsift_cache_dim{self.descriptor_dim}_ss{subsample_rate}.pkl"
        )

    def _codebook_cache_file(self) -> str:
        return os.path.join(
            CACHE_DIR,
            f"rootsift_codebook_k{self.n_clusters}_dim{self.descriptor_dim}.pkl"
        )

    # --- Internal helpers ---

    @staticmethod
    def _root_sift(des: np.ndarray) -> np.ndarray:
        """L1-normalize then sqrt (Hellinger kernel approximation)."""
        eps = 1e-12
        des = des.astype(np.float32)
        des = des / (np.sum(des, axis=1, keepdims=True) + eps)
        return np.sqrt(des)

    def _is_valid_descriptor_array(self, arr) -> bool:
        return (
            isinstance(arr, np.ndarray)
            and arr.ndim == 2
            and arr.shape[1] == self.descriptor_dim
        )

    def _validate_sift_cache_dict(self, cache_obj, file_list: list[str]) -> bool:
        if not isinstance(cache_obj, dict):
            return False

        for fname in file_list:
            if fname not in cache_obj:
                return False
            if not self._is_valid_descriptor_array(cache_obj[fname]):
                return False
        return True

    def _validate_codebook(self, codebook) -> bool:
        if codebook is None:
            return False
        if not hasattr(codebook, "cluster_centers_"):
            return False
        centers = codebook.cluster_centers_
        if not isinstance(centers, np.ndarray):
            return False
        if centers.ndim != 2:
            return False
        if centers.shape[1] != self.descriptor_dim:
            return False
        if centers.shape[0] != self.n_clusters:
            return False
        return True

    def _des_to_vlad(self, des: np.ndarray) -> np.ndarray:
        """Aggregate local descriptors into a single VLAD vector."""
        if des is None or len(des) == 0:
            return np.zeros(self.dim, dtype=np.float32)

        if des.ndim != 2 or des.shape[1] != self.descriptor_dim:
            raise ValueError(
                f"Descriptor dimension mismatch: expected (?, {self.descriptor_dim}), "
                f"got {des.shape}"
            )

        if self.codebook is None:
            raise RuntimeError("Codebook is not initialized.")

        if not self._validate_codebook(self.codebook):
            raise ValueError(
                "Loaded codebook is incompatible with current descriptor settings. "
                f"Expected {self.n_clusters} clusters and descriptor dim "
                f"{self.descriptor_dim}, got centers shape "
                f"{getattr(self.codebook, 'cluster_centers_', np.array([])).shape}."
            )

        labels = self.codebook.predict(des)
        centers = self.codebook.cluster_centers_
        k = self.codebook.n_clusters

        vlad = np.zeros((k, des.shape[1]), dtype=np.float32)

        for i in range(k):
            mask = labels == i
            if np.any(mask):
                vlad[i] = np.sum(des[mask] - centers[i], axis=0)
                norm = np.linalg.norm(vlad[i])
                if norm > 0:
                    vlad[i] /= norm

        vlad = vlad.ravel()
        vlad = np.sign(vlad) * np.sqrt(np.abs(vlad))
        norm = np.linalg.norm(vlad)
        if norm > 0:
            vlad /= norm

        return vlad.astype(np.float32)

    # --- Public API ---

    def load_sift_cache(self, file_list: list[str], subsample_rate: int):
        """Load or compute RootSIFT descriptors for all images."""
        cache_file = self._sift_cache_file(subsample_rate)

        if os.path.exists(cache_file):
            print(f"Loading cached SIFT from {cache_file}")
            try:
                with open(cache_file, "rb") as f:
                    loaded_cache = pickle.load(f)

                if self._validate_sift_cache_dict(loaded_cache, file_list):
                    self._sift_cache = loaded_cache
                    print("  Cached RootSIFT descriptors are valid.")
                    return
                else:
                    print("  Cached SIFT is incompatible or incomplete. Re-extracting...")
            except Exception as e:
                print(f"  Failed to load SIFT cache ({e}). Re-extracting...")

        print(f"Extracting SIFT for {len(file_list)} images...")
        self._sift_cache = {}

        for fname in tqdm(file_list, desc="SIFT"):
            img = cv2.imread(fname)
            if img is None:
                continue
            _, des = self.sift.detectAndCompute(img, None)
            if des is not None and len(des) > 0:
                self._sift_cache[fname] = self._root_sift(des)

        with open(cache_file, "wb") as f:
            pickle.dump(self._sift_cache, f)

        print(f"  Saved {len(self._sift_cache)} descriptors -> {cache_file}")

    def build_vocabulary(self, file_list: list[str]):
        """Fit KMeans codebook on cached SIFT descriptors."""
        cache_file = self._codebook_cache_file()

        if os.path.exists(cache_file):
            print(f"Loading cached codebook from {cache_file}")
            try:
                with open(cache_file, "rb") as f:
                    loaded_codebook = pickle.load(f)

                if self._validate_codebook(loaded_codebook):
                    self.codebook = loaded_codebook
                    print("  Cached codebook is valid.")
                    return
                else:
                    print("  Cached codebook is incompatible. Rebuilding...")
            except Exception as e:
                print(f"  Failed to load codebook cache ({e}). Rebuilding...")

        valid_descriptors = [
            self._sift_cache[f]
            for f in file_list
            if f in self._sift_cache and self._is_valid_descriptor_array(self._sift_cache[f])
        ]

        if not valid_descriptors:
            raise RuntimeError("No valid SIFT descriptors found to build vocabulary.")

        all_des = np.vstack(valid_descriptors).astype(np.float32)

        print(f"Fitting KMeans (k={self.n_clusters}) on {len(all_des)} descriptors...")
        self.codebook = KMeans(
            n_clusters=self.n_clusters,
            init='k-means++',
            n_init=3,
            max_iter=300,
            tol=1e-4,
            verbose=1,
            random_state=42,
        ).fit(all_des)

        print(f"  {self.codebook.n_iter_} iters, inertia={self.codebook.inertia_:.0f}")

        with open(cache_file, "wb") as f:
            pickle.dump(self.codebook, f)

        print(f"  Saved codebook -> {cache_file}")

    def extract(self, img: np.ndarray) -> np.ndarray:
        """Compute VLAD for a single BGR image."""
        if img is None:
            return np.zeros(self.dim, dtype=np.float32)

        _, des = self.sift.detectAndCompute(img, None)
        if des is None or len(des) == 0:
            return np.zeros(self.dim, dtype=np.float32)

        return self._des_to_vlad(self._root_sift(des))

    def extract_batch(self, file_list: list[str]) -> np.ndarray:
        """Compute VLAD for all images using cached SIFT. Returns (N, dim)."""
        vectors = []

        for fname in tqdm(file_list, desc="VLAD"):
            if fname in self._sift_cache and len(self._sift_cache[fname]) > 0:
                vectors.append(self._des_to_vlad(self._sift_cache[fname]))
            else:
                vectors.append(np.zeros(self.dim, dtype=np.float32))

        return np.array(vectors, dtype=np.float32)


# ---------------------------------------------------------------------------
# Player
# ---------------------------------------------------------------------------
class KeyboardPlayerPyGame(Player):

    def __init__(self, n_clusters: int = 128, subsample_rate: int = 5,
                 top_k_shortcuts: int = 30):
        self.fpv = None
        self.last_act = Action.IDLE
        self.screen = None
        self.keymap = None
        super().__init__()

        self.subsample_rate = subsample_rate
        self.top_k_shortcuts = top_k_shortcuts

        self.motion_frames = []
        self.file_list = []
        self.traj_boundaries = []

        traj_dirs = sorted([
            d for d in os.listdir(DATA_DIR)
            if d.startswith('traj_') and os.path.isdir(os.path.join(DATA_DIR, d))
        ])

        if traj_dirs:
            all_motion = []
            for traj_dir_name in traj_dirs:
                traj_path = os.path.join(DATA_DIR, traj_dir_name)
                info_path = os.path.join(traj_path, 'data_info.json')
                if not os.path.exists(info_path):
                    continue

                with open(info_path) as f:
                    raw = json.load(f)

                traj_id = traj_dir_name
                pure = {'FORWARD', 'LEFT', 'RIGHT', 'BACKWARD'}

                traj_motion = [
                    {
                        'step': d['step'],
                        'image': d['image'],
                        'action': d['action'][0],
                        'traj_id': traj_id,
                        'image_path': os.path.join(traj_path, d['image'])
                    }
                    for d in raw
                    if len(d['action']) == 1 and d['action'][0] in pure
                ]

                start_idx = len(all_motion)
                all_motion.extend(traj_motion)
                end_idx = len(all_motion)
                self.traj_boundaries.append((start_idx, end_idx))
                print(f"  {traj_dir_name}: {len(traj_motion)} motion frames")

            self.motion_frames = all_motion[::subsample_rate]

            self.traj_boundaries = []
            prev_traj = None
            for idx, m in enumerate(self.motion_frames):
                if m['traj_id'] != prev_traj:
                    if prev_traj is not None:
                        self.traj_boundaries[-1] = (self.traj_boundaries[-1][0], idx)
                    self.traj_boundaries.append((idx, len(self.motion_frames)))
                    prev_traj = m['traj_id']

            if self.traj_boundaries:
                self.traj_boundaries[-1] = (
                    self.traj_boundaries[-1][0],
                    len(self.motion_frames)
                )

            self.file_list = [m['image_path'] for m in self.motion_frames]
            print(
                f"Frames: {len(all_motion)} total, "
                f"{len(self.motion_frames)} after {subsample_rate}x subsample, "
                f"{len(self.traj_boundaries)} trajectories"
            )

        else:
            legacy_info = os.path.join(DATA_DIR, 'data_info.json')
            legacy_img_dir = os.path.join(DATA_DIR, 'images')

            if os.path.exists(legacy_info):
                with open(legacy_info) as f:
                    raw = json.load(f)

                pure = {'FORWARD', 'LEFT', 'RIGHT', 'BACKWARD'}
                all_motion = [
                    {
                        'step': d['step'],
                        'image': d['image'],
                        'action': d['action'][0],
                        'traj_id': 'traj_0',
                        'image_path': os.path.join(legacy_img_dir, d['image'])
                    }
                    for d in raw
                    if len(d['action']) == 1 and d['action'][0] in pure
                ]

                self.motion_frames = all_motion[::subsample_rate]
                self.file_list = [m['image_path'] for m in self.motion_frames]
                self.traj_boundaries = [(0, len(self.motion_frames))]

                print(
                    f"Frames (legacy): {len(all_motion)} total, "
                    f"{len(self.motion_frames)} after {subsample_rate}x subsample"
                )

        self.extractor = VLADExtractor(n_clusters=n_clusters)
        self.database = None
        self.G = None
        self.goal_node = None

    # --- Game engine hooks ---
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
            pygame.K_ESCAPE: Action.QUIT,
        }

    def act(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.last_act = Action.QUIT
                return Action.QUIT
            if event.type == pygame.KEYDOWN:
                if event.key in self.keymap:
                    self.last_act |= self.keymap[event.key]
                else:
                    self.show_target_images()
            if event.type == pygame.KEYUP:
                if event.key in self.keymap:
                    self.last_act ^= self.keymap[event.key]
        return self.last_act

    def see(self, fpv):
        if fpv is None or len(fpv.shape) < 3:
            return

        self.fpv = fpv

        if self.screen is None:
            h, w, _ = fpv.shape
            self.screen = pygame.display.set_mode((w, h))

        pygame.display.set_caption("KeyboardPlayer:fpv")

        if self._state and self._state[1] == Phase.NAVIGATION:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_q]:
                self.display_next_best_view()

        rgb = fpv[:, :, ::-1]
        surface = pygame.image.frombuffer(rgb.tobytes(), rgb.shape[1::-1], 'RGB')
        self.screen.blit(surface, (0, 0))
        pygame.display.update()

    def set_target_images(self, images):
        super().set_target_images(images)
        self.show_target_images()

    def pre_navigation(self):
        super().pre_navigation()
        self._build_database()
        self._build_graph()
        self._setup_goal()

    # --- VLAD database ---
    def _build_database(self):
        """Compute VLAD database."""
        if self.database is not None:
            print("Database already computed, skipping.")
            return

        self.extractor.load_sift_cache(self.file_list, self.subsample_rate)
        self.extractor.build_vocabulary(self.file_list)
        self.database = self.extractor.extract_batch(self.file_list)

        print(f"Database: {self.database.shape}")

    # --- Navigation graph ---
    def _build_graph(self):
        """Build graph with temporal + visual shortcut edges."""
        if self.G is not None:
            print("Graph already built, skipping.")
            return

        n = len(self.database)
        self.G = nx.Graph()
        self.G.add_nodes_from(range(n))

        for start, end in self.traj_boundaries:
            for i in range(start, end - 1):
                self.G.add_edge(i, i + 1, weight=TEMPORAL_WEIGHT, edge_type="temporal")

        print("Computing similarity matrix...")
        sim = self.database @ self.database.T
        np.fill_diagonal(sim, -2)

        for i in range(n):
            lo = max(0, i - MIN_SHORTCUT_GAP)
            hi = min(n, i + MIN_SHORTCUT_GAP + 1)
            sim[i, lo:hi] = -2

        sim[~np.triu(np.ones((n, n), dtype=bool), k=1)] = -2

        flat = sim.ravel()
        top_k = self.top_k_shortcuts
        top_idx = np.argpartition(flat, -top_k)[-top_k:]
        top_idx = top_idx[np.argsort(-flat[top_idx])]

        dists = []
        print(f"Top-{top_k} shortcuts (min_gap={MIN_SHORTCUT_GAP}):")

        for rank, fi in enumerate(top_idx):
            i, j = divmod(int(fi), n)
            s = float(flat[fi])
            d = float(np.sqrt(max(0, 2 - 2 * s)))
            self.G.add_edge(
                i, j,
                weight=VISUAL_WEIGHT_BASE + VISUAL_WEIGHT_SCALE * d,
                edge_type="visual"
            )
            dists.append(d)
            if rank < 5:
                print(f"  #{rank+1}: {i}<->{j} gap={abs(j-i)} d={d:.4f}")

        kd = np.array(dists)
        print(f"  {top_k} visual edges, dist: [{kd.min():.3f}, {kd.max():.3f}]")
        print(f"Graph: {self.G.number_of_nodes()} nodes, {self.G.number_of_edges()} edges")

    # --- Goal ---
    def _setup_goal(self):
        """Set goal node from front-view target image."""
        if self.goal_node is not None:
            print("Goal already set, skipping.")
            return

        targets = self.get_target_images()
        if not targets:
            return

        goal_feat = self.extractor.extract(targets[0])
        sims = self.database @ goal_feat
        self.goal_node = int(np.argmax(sims))
        d = float(np.sqrt(max(0, 2 - 2 * sims[self.goal_node])))

        print(f"Goal: node {self.goal_node} (d={d:.4f})")

    # --- Helpers ---
    def _load_img(self, idx: int) -> np.ndarray | None:
        """Load image by database index."""
        if 0 <= idx < len(self.file_list):
            return cv2.imread(self.file_list[idx])
        return None

    def _get_current_node(self) -> int:
        """Find best-matching database node for current FPV."""
        feat = self.extractor.extract(self.fpv)
        return int(np.argmax(self.database @ feat))

    def _get_path(self, start: int) -> list[int]:
        """Shortest path from start to goal_node."""
        try:
            return nx.shortest_path(self.G, start, self.goal_node, weight="weight")
        except nx.NetworkXNoPath:
            return [start]

    def _edge_action(self, a: int, b: int) -> str:
        """Get the action label for traversing edge a->b."""
        reverse = {
            'FORWARD': 'BACKWARD',
            'BACKWARD': 'FORWARD',
            'LEFT': 'RIGHT',
            'RIGHT': 'LEFT'
        }

        if b == a + 1 and a < len(self.motion_frames):
            return self.motion_frames[a]['action']
        elif b == a - 1 and b < len(self.motion_frames):
            return reverse.get(self.motion_frames[b]['action'], '?')

        return '?'

    # --- Display ---
    def show_target_images(self):
        targets = self.get_target_images()
        if not targets:
            return

        top = cv2.hconcat(targets[:2])
        bot = cv2.hconcat(targets[2:])
        img = cv2.vconcat([top, bot])

        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 0, 0), 2)
        cv2.line(img, (0, h // 2), (w, h // 2), (0, 0, 0), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        for label, pos in [
            ('Front', (10, 25)),
            ('Right', (w // 2 + 10, 25)),
            ('Back', (10, h // 2 + 25)),
            ('Left', (w // 2 + 10, h // 2 + 25))
        ]:
            cv2.putText(img, label, pos, font, 0.75, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('Target Images', img)
        cv2.waitKey(1)

    def display_next_best_view(self):
        """
        Navigation panel:
            Info bar: current node | goal | hops | next action
            Row 1:    [Live FPV] [Best match] [Target (front)]
            Row 2:    Path preview (next 5 nodes)
        """
        act_map = {
            'FORWARD': 'FWD',
            'BACKWARD': 'BACK',
            'LEFT': 'LEFT',
            'RIGHT': 'RIGHT'
        }

        font = cv2.FONT_HERSHEY_SIMPLEX
        aa = cv2.LINE_AA
        TW, TH = 260, 195
        PW, PH = TW * 3 // 5, TH * 3 // 5
        N_PREVIEW = 5

        cur = self._get_current_node()
        cur_feat = self.extractor.extract(self.fpv)
        cur_sim = float(self.database[cur] @ cur_feat)
        cur_d = float(np.sqrt(max(0, 2 - 2 * cur_sim)))
        path = self._get_path(cur)
        hops = len(path) - 1

        edge_info = []
        for a, b in zip(path[:-1], path[1:]):
            et = self.G[a][b].get("edge_type", "temporal")
            if et == "temporal":
                act = act_map.get(self._edge_action(a, b), '?')
                edge_info.append(("seq", act, b == a + 1))
            else:
                edge_info.append(("vis", None, None))

        t_steps = sum(1 for e in edge_info if e[0] == "seq")
        v_jumps = len(edge_info) - t_steps

        if edge_info:
            etype, act, _ = edge_info[0]
            hint = act if etype == "seq" else "VISUAL JUMP"
        else:
            hint = "AT GOAL"

        near = hops <= 5

        panel_w = TW * 3
        bar = np.zeros((40, panel_w, 3), dtype=np.uint8)
        bar[:] = (0, 0, 160) if near else (50, 35, 15)

        txt = (
            f"Node {cur} (d={cur_d:.3f})"
            f"  |  Goal {self.goal_node}"
            f"  |  {hops} hops ({t_steps}s+{v_jumps}v)"
            f"  |  >> {hint}"
        )

        cv2.putText(bar, txt, (8, 27), font, 0.48, (255, 255, 255), 1, aa)

        if near:
            cv2.putText(
                bar,
                "NEAR TARGET — SPACE",
                (panel_w - 220, 27),
                font,
                0.48,
                (0, 255, 255),
                1,
                aa
            )

        def thumb(img, label, color, extra=None):
            t = cv2.resize(img, (TW, TH))
            cv2.rectangle(t, (0, 0), (TW - 1, TH - 1), color, 2)
            cv2.putText(t, label, (6, 22), font, 0.55, color, 1, aa)
            if extra:
                cv2.putText(t, extra, (6, 44), font, 0.45, (200, 200, 200), 1, aa)
            return t

        fpv_t = thumb(self.fpv, "Live FPV", (255, 255, 255))

        match_img = self._load_img(cur)
        if match_img is None:
            match_img = np.zeros((TH, TW, 3), dtype=np.uint8)

        match_t = thumb(match_img, f"Match: node {cur}", (0, 255, 0), f"d={cur_d:.3f}")

        targets = self.get_target_images()
        tgt = targets[0] if targets else np.zeros((TH, TW, 3), dtype=np.uint8)
        tgt_t = thumb(tgt, "Target (front)", (0, 140, 255))
        row1 = cv2.hconcat([fpv_t, match_t, tgt_t])

        preview = path[1:1 + N_PREVIEW]
        cells = []

        for p in range(N_PREVIEW):
            if p < len(preview):
                img = self._load_img(preview[p])
                if img is None:
                    img = np.zeros((PH, PW, 3), dtype=np.uint8)

                img = cv2.resize(img, (PW, PH))
                etype, act, is_fwd = edge_info[p]

                if etype == "seq":
                    lbl = f"{'>' if is_fwd else '<'} {act}"
                    clr = (200, 200, 0)
                else:
                    lbl = "~ VISUAL"
                    clr = (200, 100, 255)

                cv2.rectangle(img, (0, 0), (PW - 1, PH - 1), clr, 1)
                cv2.putText(
                    img,
                    f"+{p+1} node {preview[p]}",
                    (4, 16),
                    font,
                    0.38,
                    (255, 255, 255),
                    1,
                    aa
                )
                cv2.putText(img, lbl, (4, 34), font, 0.38, clr, 1, aa)
            else:
                img = np.zeros((PH, PW, 3), dtype=np.uint8)

            cells.append(img)

        row2 = cv2.hconcat(cells)

        if row2.shape[1] < panel_w:
            pad = np.zeros((PH, panel_w - row2.shape[1], 3), dtype=np.uint8)
            row2 = cv2.hconcat([row2, pad])

        panel = cv2.vconcat([bar, row1, row2])
        cv2.imshow("Navigation", panel)
        cv2.waitKey(1)

        print(
            f"Node {cur} -> Goal {self.goal_node} | "
            f"{hops} hops ({t_steps}s+{v_jumps}v) | >> {hint}"
        )


if __name__ == "__main__":
    import argparse
    import vis_nav_game

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--subsample",
        type=int,
        default=5,
        help="Take every Nth motion frame (default: 5)"
    )
    parser.add_argument(
        "--n-clusters",
        type=int,
        default=128,
        help="VLAD codebook size (default: 128)"
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=30,
        help="Number of global visual shortcut edges (default: 30)"
    )
    args = parser.parse_args()

    vis_nav_game.play(
        the_player=KeyboardPlayerPyGame(
            n_clusters=args.n_clusters,
            subsample_rate=args.subsample,
            top_k_shortcuts=args.top_k,
        )
    )