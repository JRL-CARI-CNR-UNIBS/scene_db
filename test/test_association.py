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

"""Unit tests for grasp matching, association and pose transforms."""

import math

import numpy as np
import pytest

from scene_db.association import (
    associate,
    AssociationConfig,
    Candidate,
    containment_depth,
    grasp_volume,
    match_grasps,
    rotate,
    transform_pose,
)
from scene_db.grasp_planner import plan_grasps
from scene_db.relations import Box

YAW45 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def test_transform_pose():
    q90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert rotate(q90, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    p, q = transform_pose((1.0, 0.0, 0.0), q90, (1.0, 0.0, 0.0), q90)
    assert p == pytest.approx((1.0, 1.0, 0.0))
    assert rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((-1.0, 0.0, 0.0))


def test_containment_rotated_box():
    box = Box((0.0, 0.0, 0.0), YAW45, (0.4, 0.04, 0.1))
    assert containment_depth((0.1, 0.1, 0.0), box, 0.0) > 0      # along the long axis
    assert containment_depth((0.1, -0.1, 0.0), box, 0.0) < 0     # across it
    assert containment_depth((0.0, 0.0, 0.06), box, 0.0) < 0
    assert containment_depth((0.0, 0.0, 0.06), box, 0.02) > 0    # margin


def test_match_grasps():
    boxes = [Box((0.0, 0.0, 0.0), (0, 0, 0, 1), (0.2, 0.2, 0.2)),
             Box((0.0, 0.0, 0.0), (0, 0, 0, 1), (0.06, 0.06, 0.06)),   # small, inside
             Box((1.0, 0.0, 0.0), (0, 0, 0, 1), (0.1, 0.1, 0.1))]
    pts = [(0.0, 0.0, 0.0),        # in both 0 and 1: deeper in 0
           (0.09, 0.0, 0.0),       # only 0
           (1.0, 0.0, 0.06),       # 2 thanks to the margin
           (0.5, 0.0, 0.0)]        # none
    assert match_grasps(pts, boxes, margin=0.015) == [0, 0, 2, -1]


def test_match_grasps_skips_non_eligible_boxes():
    table = Box((0.0, 0.0, 0.2), (0, 0, 0, 1), (1.0, 1.0, 0.6))   # box up to the objects
    cup = Box((0.2, 0.0, 0.25), (0, 0, 0, 1), (0.08, 0.08, 0.1))
    pts = [(0.2, 0.0, 0.29),    # on the cup (inside the table box too)
           (-0.3, 0.1, 0.19)]   # only in the table box: an undetected object
    assert match_grasps(pts, [table, cup], 0.015) == [0, 0]   # the table box is deeper
    assert match_grasps(pts, [table, cup], 0.015, eligible=[False, True]) == [1, -1]


def test_grasp_volume():
    yaw45 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))
    box = Box((1.0, 0.0, 0.85), yaw45, (0.1, 0.1, 0.1))
    lo, hi = grasp_volume(box, margin=0.05, below=0.05, height=0.3)
    r = 0.05 * math.sqrt(2.0)          # half diagonal of the rotated footprint
    assert lo == pytest.approx((1.0 - r - 0.05, -r - 0.05, 0.75))
    assert hi == pytest.approx((1.0 + r + 0.05, r + 0.05, 1.05))
    tall = Box((0.0, 0.0, 0.3), (0, 0, 0, 1), (0.1, 0.1, 0.6))
    lo, hi = grasp_volume(tall, margin=0.05, below=0.05, height=0.3)
    assert lo[2] == pytest.approx(-0.05) and hi[2] == pytest.approx(0.65)


def test_associate_hungarian_and_moves():
    cfg = AssociationConfig(distance=0.15, size_factor=0.5, move_threshold=0.03)
    objects = [Candidate('cup', (0.0, 0.0, 0.8), (0.08, 0.08, 0.1)),
               Candidate('cup', (0.2, 0.0, 0.8), (0.08, 0.08, 0.1)),
               Candidate('bowl', (0.5, 0.0, 0.8), (0.15, 0.15, 0.1))]
    dets = [Candidate('cup', (0.12, 0.0, 0.8), (0.08, 0.08, 0.1)),   # nearer cup_1 than cup_0
            Candidate('cup', (0.01, 0.0, 0.8), (0.08, 0.08, 0.1)),
            Candidate('bowl', (1.5, 0.0, 0.8), (0.15, 0.15, 0.1)),   # too far: new
            Candidate('mug', (0.5, 0.0, 0.8), (0.1, 0.1, 0.1))]      # other class: new
    out = associate(dets, objects, cfg)
    assert out == [(1, True), (0, False), (-1, False), (-1, False)]
    assert associate([], objects, cfg) == []
    assert associate(dets[:1], [], cfg) == [(-1, False)]


def test_associate_color_gate():
    cfg = AssociationConfig(match_color=True)
    out = associate([Candidate('cup', (0, 0, 0), (0.1, 0.1, 0.1), 'red')],
                    [Candidate('cup', (0, 0, 0), (0.1, 0.1, 0.1), 'blue')], cfg)
    assert out == [(-1, False)]


def tabletop(objects, shape=(240, 320)):
    """Points image of a camera looking down at a table 1 m away, with boxes on it."""
    h, w = shape
    f, cx, cy = 300.0, w / 2, h / 2
    v, u = np.mgrid[0:h, 0:w]
    z = np.full(shape, 1.0)
    for (u1, v1, u2, v2, height) in objects:
        z[v1:v2, u1:u2] = 1.0 - height
    return np.dstack([(u - cx) * z / f, (v - cy) * z / f, z])


def test_plan_grasps_top_down():
    pts = tabletop([(100, 100, 120, 160, 0.08)])   # 20 x 60 px block, 8 cm high
    grasps = plan_grasps(pts, min_height=0.01, max_width=0.1, min_points=50)
    assert len(grasps) == 1
    g = grasps[0]
    assert g.position[2] == pytest.approx(0.92, abs=0.01)    # top of the block
    approach = rotate(g.quaternion, (1.0, 0.0, 0.0))
    assert approach == pytest.approx((0.0, 0.0, 1.0), abs=0.05)   # into the table
    closing = rotate(g.quaternion, (0.0, 1.0, 0.0))
    assert abs(closing[0]) > 0.95                            # across the short side (u)
    assert g.width == pytest.approx(20 * 0.92 / 300 + 0.02, abs=0.01)
    assert 0.0 < g.score <= 1.0
    assert plan_grasps(tabletop([(100, 100, 300, 200, 0.08)]), max_width=0.1) == []
