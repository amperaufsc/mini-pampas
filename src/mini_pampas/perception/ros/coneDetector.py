import math
from dataclasses import dataclass
from typing import List, Tuple

Point2D = Tuple[float, float]
Cluster = List[Point2D]


@dataclass
class DetectorConfig:
    min_r: float = 0.15
    max_r: float = 4.00
    min_a: float = math.pi / 2
    max_a: float = -math.pi / 2
    cluster_thresh: float = 0.20
    min_pts: int = 2
    max_pts: int = 35
    max_w: float = 0.30


class ConeDetector:
    """Pure Python class for 2D LiDAR cone detection."""

    def process_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        cfg: DetectorConfig
    ) -> List[Point2D]:
        """Full pipeline: returns a list of (x, y) cone centroids."""
        points = self._filter_and_project(ranges, angle_min, angle_increment, cfg)
        clusters = self._cluster_points(points, cfg.cluster_thresh)
        return self._extract_centroids(clusters, cfg)

    def _filter_and_project(
        self, ranges: List[float], angle_min: float, angle_inc: float, cfg: DetectorConfig
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

        if len(clusters) > 1 and self._dist(clusters[0][0], clusters[-1][-1]) <= thresh:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

        return clusters

    def _extract_centroids(self, clusters: List[Cluster], cfg: DetectorConfig) -> List[Point2D]:
        centroids: List[Point2D] = []
        for c in clusters:
            if not (cfg.min_pts <= len(c) <= cfg.max_pts):
                continue
            if self._dist(c[0], c[-1]) > cfg.max_w:
                continue
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            centroids.append((cx, cy))
        return centroids

    @staticmethod
    def _is_angle_valid(angle: float, min_a: float, max_a: float) -> bool:
        return (min_a <= angle <= max_a) if (min_a <= max_a) else (angle >= min_a or angle <= max_a)

    @staticmethod
    def _is_range_valid(r: float, min_r: float, max_r: float) -> bool:
        return not (math.isinf(r) or math.isnan(r)) and (min_r <= r <= max_r)

    @staticmethod
    def _dist(p1: Point2D, p2: Point2D) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])