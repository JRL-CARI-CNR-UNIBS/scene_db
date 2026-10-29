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
Logic of the scene updater, without ROS.

* :func:`match_grasps`: assign each grasp to the object whose box (expanded
  by a margin) contains the grasp center.
* :func:`associate`: match the detections of an acquisition to the objects
  already in the database (same class, Hungarian assignment on distance).
* :func:`transform_pose`: move poses between frames.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scene_db.relations import Box
from scipy.optimize import linear_sum_assignment

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

_BIG = 1e9


# ----------------------------------------------------------------------------
# Poses
# ----------------------------------------------------------------------------
def quat_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product ``a * b`` of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate a vector by a unit quaternion."""
    x, y, z, w = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)),
                               (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def transform_pose(translation: Vec3, rotation: Quat, position: Vec3,
                   orientation: Quat) -> Tuple[Vec3, Quat]:
    """Apply the transform (translation, rotation) to a pose."""
    p = rotate(rotation, position)
    return ((p[0] + translation[0], p[1] + translation[1], p[2] + translation[2]),
            quat_multiply(rotation, orientation))


# ----------------------------------------------------------------------------
# Grasp to object
# ----------------------------------------------------------------------------
def containment_depth(point: Vec3, box: Box, margin: float) -> float:
    """
    How deep a point lies inside a box expanded by ``margin``.

    :return: Minimum distance to the faces of the expanded box, along the box
        axes (positive inside, negative outside).
    """
    yaw = box.yaw
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy, dz = (point[i] - box.center[i] for i in range(3))
    local = (c * dx + s * dy, -s * dx + c * dy, dz)
    return min(0.5 * box.size[i] + margin - abs(local[i]) for i in range(3))


def match_grasps(points: Sequence[Vec3], boxes: Sequence[Box], margin: float,
                 eligible: Optional[Sequence[bool]] = None) -> List[int]:
    """
    Assign grasp centers to boxes.

    A grasp belongs to a box when its center is inside the box expanded by
    ``margin``. With several candidates the deepest containment wins; on a
    tie (within 1 mm) the nearest box center.

    :param eligible: per box, whether it can receive grasps (default: all).
        Non-eligible boxes (e.g. the table) are ignored, so the grasps they
        contain go to another box or are discarded.
    :return: Box index for each point, -1 for unmatched points (discarded).
    """
    out = []
    for p in points:
        best, best_key = -1, None
        for i, b in enumerate(boxes):
            if eligible is not None and not eligible[i]:
                continue
            depth = containment_depth(p, b, margin)
            if depth < 0.0:
                continue
            key = (round(depth / 0.001), -math.dist(p, b.center))
            if best_key is None or key > best_key:
                best, best_key = i, key
        out.append(best)
    return out


# ----------------------------------------------------------------------------
# Detections to database objects
# ----------------------------------------------------------------------------
@dataclass
class Candidate:
    """An object for the association (detection or stored object)."""

    class_name: str
    center: Vec3
    size: Vec3
    color: str = ''


@dataclass
class AssociationConfig:
    """Gates of the association."""

    distance: float = 0.15
    """Minimum gate on the distance between centers (m)."""
    size_factor: float = 0.5
    """Gate = max(distance, size_factor * larger horizontal side)."""
    move_threshold: float = 0.03
    """Minimum displacement considered a move (m)."""
    move_size_factor: float = 0.2
    """Move gate = max(move_threshold, move_size_factor * larger horizontal side)."""
    match_color: bool = False
    """Require the same color name."""


def _side(c: Candidate) -> float:
    return max(c.size[0], c.size[1])


def associate(detections: Sequence[Candidate], objects: Sequence[Candidate],
              cfg: AssociationConfig) -> List[Tuple[int, bool]]:
    """
    Match detections to stored objects.

    Pairs need the same class (and color, if enabled) and a center distance
    within the gate. The assignment minimizes the total distance (Hungarian).

    :return: For each detection ``(object index or -1, moved)``.
    """
    n, m = len(detections), len(objects)
    out = [(-1, False)] * n
    if n == 0 or m == 0:
        return out
    cost = np.full((n, m), _BIG)
    for i, d in enumerate(detections):
        for j, o in enumerate(objects):
            if d.class_name != o.class_name:
                continue
            if cfg.match_color and d.color and o.color and d.color != o.color:
                continue
            dist = math.dist(d.center, o.center)
            if dist <= max(cfg.distance, cfg.size_factor * max(_side(d), _side(o))):
                cost[i, j] = dist
    rows, cols = linear_sum_assignment(cost)
    for i, j in zip(rows, cols):
        if cost[i, j] >= _BIG:
            continue
        gate = max(cfg.move_threshold,
                   cfg.move_size_factor * max(_side(detections[i]), _side(objects[j])))
        out[i] = (int(j), bool(cost[i, j] > gate))
    return out


def best_grasp_index(scores: Sequence[float]) -> Optional[int]:
    """Index of the highest score, None if empty."""
    return int(np.argmax(scores)) if len(scores) else None
