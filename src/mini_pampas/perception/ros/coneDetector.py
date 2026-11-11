import math
from dataclasses import dataclass
from typing import List, Tuple, Sequence

Point2D = Tuple[float, float]
Cluster = List[Point2D]


@dataclass
class DetectorConfig:
    # Basic filtering & clustering
    min_r: float = 0.15
    max_r: float = 2
    min_a: float = math.pi / 2
    max_a: float = -math.pi / 2
    cluster_thresh: float = 0.20
    min_pts: int = 2
    max_pts: int = 35
    max_w: float = 0.30

    # 1. Surface Centroid Bias & 2. Distance-Adaptive Min Points
    cone_radius: float = 0.045          # Standard small traffic cone base radius (~7.5 cm)
    expected_beam_ratio: float = 0.35   # Require at least 35% of theoretical beam hits

    # 3. Geometric Shape Check
    max_depth: float = 0.18             # Max radial depth of a cluster (m)
    convexity_tol: float = 0.025        # Tolerance (m) for middle points protruding toward sensor

    # 4. Temporal Filtering / Multi-Frame Tracking
    enable_tracking: bool = True
    assoc_dist: float = 0.30            # Max distance (m) to match cone between frames
    min_hits: int = 2                   # Frames required to confirm and publish a cone (must be adjusted for LiDAR frame rate)
    max_misses: int = 1                 # Consecutive missed frames before deleting track (must be adjusted for LiDAR frame rate)
    ema_alpha: float = 0.85             # Position smoothing factor (1.0 = no smoothing)


@dataclass
class TrackedCone:
    x: float
    y: float
    hits: int = 1
    misses: int = 0
    confirmed: bool = False


class ConeDetector:
    """Stateful 2D LiDAR cone detector with shape validation and multi-frame tracking."""

    def __init__(self) -> None:
        self._tracks: List[TrackedCone] = []

    def process_scan(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        cfg: DetectorConfig
    ) -> List[Point2D]:
        """Full perception pipeline: returns confirmed, bias-corrected (x, y) cone centers."""
        # Step 1: Filter & project polar scan to Cartesian (x, y)
        points = self._filter_and_project(ranges, angle_min, angle_increment, cfg)

        # Step 2: Distance-gap clustering
        clusters = self._cluster_points(points, cfg.cluster_thresh)

        # Step 3: Adaptive count check, geometric shape filter, and surface bias correction
        raw_centroids = self._extract_and_validate_cones(clusters, abs(angle_increment), cfg)

        # Step 4: Multi-frame temporal tracking
        if cfg.enable_tracking:
            return self._update_tracker(raw_centroids, cfg)
        return raw_centroids

    # =========================================================================
    # Pipeline Stages
    # =========================================================================

    def _filter_and_project(
        self, ranges: Sequence[float], angle_min: float, angle_inc: float, cfg: DetectorConfig
    ) -> List[Point2D]:
        points: List[Point2D] = []
        angle = angle_min

        for r in ranges:
            if self._is_angle_valid(angle, cfg.min_a, cfg.max_a) and self._is_range_valid(r, cfg.min_r, cfg.max_r):
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += angle_inc
        return points

    def _cluster_points(self, points: List[Point2D], thresh: float) -> List[Cluster]:
        if not points:
            return []

        clusters: List[Cluster] = []
        current: Cluster = [points[0]]

        for pt in points[1:]:
            if self._dist(pt, current[-1]) <= thresh:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]

        if current:
            clusters.append(current)

        # Merge across -pi / +pi boundary
        if len(clusters) > 1 and self._dist(clusters[0][0], clusters[-1][-1]) <= thresh:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

        return clusters

    def _extract_and_validate_cones(
        self, clusters: List[Cluster], angle_inc: float, cfg: DetectorConfig
    ) -> List[Point2D]:
        valid_cones: List[Point2D] = []

        for cluster in clusters:
            num_pts = len(cluster)
            if num_pts > cfg.max_pts or num_pts < cfg.min_pts:
                continue

            # Raw surface centroid
            cx = sum(p[0] for p in cluster) / num_pts
            cy = sum(p[1] for p in cluster) / num_pts
            dist_to_sensor = math.hypot(cx, cy)

            if dist_to_sensor < 1e-3:
                continue

            # Upgrade 2: Distance-Adaptive Minimum Points
            if not self._passes_adaptive_min_pts(num_pts, dist_to_sensor, angle_inc, cfg):
                continue

            # Upgrade 3: Geometric Shape Check (Width, Depth, and Convexity)
            if not self._passes_shape_check(cluster, cx, cy, dist_to_sensor, cfg):
                continue

            # Upgrade 1: Surface Centroid Bias (Range Offset)
            true_cx, true_cy = self._apply_surface_bias(cx, cy, dist_to_sensor, cfg.cone_radius)
            valid_cones.append((true_cx, true_cy))

        return valid_cones

    # =========================================================================
    # Upgrade Helpers (1, 2, 3, 4)
    # =========================================================================

    @staticmethod
    def _apply_surface_bias(
        cx: float, cy: float, dist: float, cone_radius: float
    ) -> Point2D:
        """Pushes the surface centroid away from the sensor toward the cone's cylindrical center."""
        # Illuminated arc centroid sits roughly 0.7 * radius in front of true center
        offset = cone_radius * 0.70
        ux = cx / dist
        uy = cy / dist
        return (cx + ux * offset, cy + uy * offset)

    @staticmethod
    def _passes_adaptive_min_pts(
        num_pts: int, dist: float, angle_inc: float, cfg: DetectorConfig
    ) -> bool:
        """Computes expected beam hits at distance `dist` and rejects sparse close-range noise."""
        if angle_inc <= 1e-6:
            return num_pts >= cfg.min_pts

        cone_diameter = 2.0 * cfg.cone_radius
        subtended_angle = cone_diameter / dist
        expected_beams = subtended_angle / angle_inc

        adaptive_min = max(cfg.min_pts, int(math.floor(expected_beams * cfg.expected_beam_ratio)))
        # Cap adaptive_min at max_pts so very close cones aren't impossibly strict
        adaptive_min = min(adaptive_min, cfg.max_pts)

        return num_pts >= adaptive_min

    def _passes_shape_check(
        self, cluster: Cluster, cx: float, cy: float, dist: float, cfg: DetectorConfig
    ) -> bool:
        """Rejects short walls, flat boxes, and concave corners using ray-aligned geometry."""
        # 1. End-to-end span check
        span = self._dist(cluster[0], cluster[-1])
        if span > cfg.max_w:
            return False

        # Unit vector from sensor to cluster centroid (radial direction)
        ux, uy = cx / dist, cy / dist
        # Perpendicular unit vector (tangential / cross-range direction)
        vx, vy = -uy, ux

        radial_coords = []
        tangential_coords = []

        for px, py in cluster:
            # Project points into local (radial, tangential) frame
            radial_coords.append(px * ux + py * uy)
            tangential_coords.append(px * vx + py * vy)

        width = max(tangential_coords) - min(tangential_coords)
        depth = max(radial_coords) - min(radial_coords)

        if width > cfg.max_w or depth > cfg.max_depth:
            return False

        # 2. Convexity & Wall Rejection (for clusters with 3+ beams)
        if len(cluster) >= 3:
            r_first = radial_coords[0]
            r_last = radial_coords[-1]
            r_edges_mean = 0.5 * (r_first + r_last)

            mid_idx = len(cluster) // 2
            r_mid = radial_coords[mid_idx]

            # On a cylinder facing the LiDAR, the center beam is closer than the side beams.
            # If r_mid is significantly further away than the edges, it's a concave corner.
            if r_mid > r_edges_mean + cfg.convexity_tol:
                return False

            # Reject flat oblique walls (wide span with virtually zero curvature/depth)
            if len(cluster) >= 5 and width > 0.12 and depth < 0.008:
                return False

        return True

    def _update_tracker(
        self, detections: List[Point2D], cfg: DetectorConfig
    ) -> List[Point2D]:
        """Associates new detections with existing tracks using greedy nearest-neighbor matching."""
        unmatched_tracks = set(range(len(self._tracks)))
        unmatched_dets = set(range(len(detections)))

        # Build all candidate pairs within assoc_dist
        pairs: List[Tuple[float, int, int]] = []
        for t_idx, track in enumerate(self._tracks):
            for d_idx, det in enumerate(detections):
                d = math.hypot(track.x - det[0], track.y - det[1])
                if d <= cfg.assoc_dist:
                    pairs.append((d, t_idx, d_idx))

        # Sort by shortest distance first
        pairs.sort(key=lambda item: item[0])

        for _, t_idx, d_idx in pairs:
            if t_idx in unmatched_tracks and d_idx in unmatched_dets:
                track = self._tracks[t_idx]
                det_x, det_y = detections[d_idx]

                # Exponential Moving Average (EMA) position update
                a = cfg.ema_alpha
                track.x = a * det_x + (1.0 - a) * track.x
                track.y = a * det_y + (1.0 - a) * track.y
                track.hits += 1
                track.misses = 0
                if track.hits >= cfg.min_hits:
                    track.confirmed = True

                unmatched_tracks.remove(t_idx)
                unmatched_dets.remove(d_idx)

        # Increment misses on unmatched existing tracks and prune dead tracks
        surviving_tracks: List[TrackedCone] = []
        for t_idx, track in enumerate(self._tracks):
            if t_idx in unmatched_tracks:
                track.misses += 1
            if track.misses <= cfg.max_misses:
                surviving_tracks.append(track)

        # Spawn tentative tracks for unmatched new detections
        for d_idx in unmatched_dets:
            det_x, det_y = detections[d_idx]
            surviving_tracks.append(
                TrackedCone(
                    x=det_x,
                    y=det_y,
                    hits=1,
                    misses=0,
                    confirmed=(cfg.min_hits <= 1)
                )
            )

        self._tracks = surviving_tracks

        # Return only confirmed tracks
        return [(t.x, t.y) for t in self._tracks if t.confirmed]

    # =========================================================================
    # Low-Level Static Math Helpers
    # =========================================================================

    @staticmethod
    def _is_angle_valid(angle: float, min_a: float, max_a: float) -> bool:
        return (min_a <= angle <= max_a) if (min_a <= max_a) else (angle >= min_a or angle <= max_a)

    @staticmethod
    def _is_range_valid(r: float, min_r: float, max_r: float) -> bool:
        return not (math.isinf(r) or math.isnan(r)) and (min_r <= r <= max_r)

    @staticmethod
    def _dist(p1: Point2D, p2: Point2D) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])