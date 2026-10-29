# Copyright 2026 Manuel Beschi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Simple geometric grasp planner (stand-in for AnyGrasp). No ROS here.

Used by ``mock_grasp_server``.

1. Fit the support plane (largest plane, RANSAC).
2. Points higher than ``min_height`` above it form blobs (connected
   components in the image).
3. One top-down grasp per blob: center on the top surface, approach along
   the plane normal (into the plane), fingers closing across the shorter
   horizontal side. Blobs wider than the gripper are skipped.

Grasp convention (same as grasp_detection_msgs/Grasp): x = approach,
y = closing direction, z = x cross y.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class PlannedGrasp:
    """A grasp in the frame of the input points."""

    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    score: float
    width: float
    height: float
    depth: float


def fit_plane(points: np.ndarray, threshold: float = 0.01, iterations: int = 150,
              sample: int = 4000, seed: int = 0) -> Optional[Tuple[np.ndarray, float]]:
    """
    Fit the dominant plane with RANSAC.

    :param points: N x 3 finite points.
    :return: ``(normal, d)`` with ``normal . p + d = 0`` and the normal pointing
        towards the origin (the camera), or None.
    """
    if points.shape[0] < 3:
        return None
    rng = np.random.default_rng(seed)
    pts = points[rng.choice(points.shape[0], min(sample, points.shape[0]), replace=False)]
    best, best_count = None, 0
    for _ in range(iterations):
        a, b, c = pts[rng.choice(pts.shape[0], 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = -float(n @ a)
        count = int(np.sum(np.abs(pts @ n + d) < threshold))
        if count > best_count:
            best, best_count = (n, d), count
    if best is None:
        return None
    n, d = best
    # Refine with least squares on the inliers.
    inliers = pts[np.abs(pts @ n + d) < threshold]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid)
    n = vt[2]
    d = -float(n @ centroid)
    if d < 0:   # make the camera side positive: n . 0 + d > 0
        n, d = -n, -d
    return n, d


def rotation_to_quaternion(m: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 rotation matrix to (x, y, z, w)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def plan_grasps(points_img: np.ndarray, min_height: float = 0.01, max_width: float = 0.1,
                min_points: int = 100, finger_clearance: float = 0.02,
                finger_height: float = 0.02, plane_threshold: float = 0.01
                ) -> List[PlannedGrasp]:
    """
    Plan top-down grasps on the objects resting on the dominant plane.

    :param points_img: H x W x 3 points image (camera frame), NaN = invalid.
    :param min_height: Points this high above the plane belong to objects (m).
    :param max_width: Maximum gripper opening (m).
    :param min_points: Minimum points of a blob.
    :param finger_clearance: Added to the object width to get the opening (m).
    :return: Grasps sorted by decreasing score.
    """
    valid = np.isfinite(points_img).all(axis=2)
    plane = fit_plane(points_img[valid], threshold=plane_threshold)
    if plane is None:
        return []
    n, d = plane
    height = np.where(valid, points_img @ n + d, 0.0)
    mask = (valid & (height > min_height)).astype(np.uint8)
    count, labels = cv2.connectedComponents(mask, connectivity=8)

    # Two horizontal axes of the plane, for the blob footprint.
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(n, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)

    grasps = []
    for label in range(1, count):
        sel = labels == label
        if int(sel.sum()) < min_points:
            continue
        pts, h = points_img[sel], height[sel]
        top = float(np.percentile(h, 90))
        flat = np.column_stack([pts @ e1, pts @ e2])
        mean = flat.mean(axis=0)
        cov = np.cov((flat - mean).T)
        evals, evecs = np.linalg.eigh(cov)
        minor = evecs[:, 0]                 # eigh: ascending eigenvalues
        proj = (flat - mean) @ minor
        extent = float(np.percentile(proj, 98) - np.percentile(proj, 2))
        width = extent + finger_clearance
        if width > max_width:
            continue
        closing = minor[0] * e1 + minor[1] * e2
        approach = -n                       # towards the plane
        z_axis = np.cross(approach, closing)
        rot = np.column_stack([approach, closing, z_axis])
        # Center: footprint center, on the top surface.
        center_on_plane = mean[0] * e1 + mean[1] * e2 - d * n
        position = center_on_plane + top * n
        grasp_depth = min(finger_height, max(top - min_height, 0.005))
        score = float(np.clip(1.0 - (width / max_width) ** 2, 0.05, 1.0))
        grasps.append(PlannedGrasp(tuple(float(v) for v in position),
                                   rotation_to_quaternion(rot), score, float(width),
                                   float(finger_height), float(grasp_depth)))
    return sorted(grasps, key=lambda g: -g.score)
