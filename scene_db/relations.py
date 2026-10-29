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
Scene relations and blocking logic. No ROS and no SQL here.

Geometric relations are derived from the boxes; the blocking logic is used
for picking and disassembly.

Relation direction: ``(a, type, b)`` reads "a <type> b", e.g.
``cup_1 on_top_of table_1`` or ``lid_1 screwed_to box_1``.

Blocking rules (what prevents picking object X):

* every object Y with ``Y on_top_of X`` (Y must be removed first);
* every *tightened* mechanical relation ``X <type> B``:

  - ``type`` in ``blocking_types`` (default ``closed_by``, ``held_by``):
    the object B must be removed first;
  - any other mechanical type (``screwed_to``, ``bracketed_to``, ...): the
    fastening must be released (its state changed) first.

Unconfirmed inferred relations block too: a missed fastening is expensive.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

GEOMETRIC = 'geometric'
MECHANICAL = 'mechanical'
A_PRIORI = 'a_priori'
INFERRED = 'inferred'

ACTIVE = 'active'
TIGHTENED = 'tightened'
LOOSENED = 'loosened'
REMOVED = 'removed'
MECHANICAL_STATES = (TIGHTENED, LOOSENED, REMOVED)

ON_TOP_OF = 'on_top_of'
INSIDE = 'inside'
NEAR = 'near'
LEFT_OF = 'left_of'
RIGHT_OF = 'right_of'
GEOMETRIC_TYPES = (ON_TOP_OF, INSIDE, NEAR, LEFT_OF, RIGHT_OF)


class SceneError(ValueError):
    """Invalid request (unknown id, bad state, cycle, ...)."""


@dataclass
class GeometricConfig:
    """Thresholds of the geometric relations (meters)."""

    on_top_tolerance: float = 0.03
    """Max penetration of A's bottom into B's top; also groups equal-height supports."""
    on_top_max_gap: float = 0.15
    """Max gap between the bottom of A and the top of its support. Single-view
    boxes often cover only the visible top surface, so the gap can be as large
    as the hidden height of A."""
    on_top_min_overlap: float = 0.3
    """Min fraction of A's footprint over B's footprint."""
    inside_tolerance: float = 0.02
    near_distance: float = 0.3
    """Max distance between centers for near / left_of / right_of."""
    lateral_tolerance: float = 0.05
    left_axis: Vec3 = (0.0, 1.0, 0.0)
    """Direction of "left" in the fixed frame."""


@dataclass
class Box:
    """Box of an object: center, rotation about z, size along its own axes."""

    center: Vec3
    quaternion: Quat
    size: Vec3

    @property
    def yaw(self) -> float:
        """Rotation about z (the quaternion is assumed to be yaw-only)."""
        x, y, z, w = self.quaternion
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def half_extents(self) -> Vec3:
        """Half sizes of the axis-aligned box enclosing the rotated box."""
        c, s = abs(math.cos(self.yaw)), abs(math.sin(self.yaw))
        sx, sy, sz = self.size
        return (0.5 * (c * sx + s * sy), 0.5 * (s * sx + c * sy), 0.5 * sz)

    def aabb(self) -> Tuple[Vec3, Vec3]:
        """``(min, max)`` corners of the enclosing axis-aligned box."""
        h = self.half_extents()
        lo = tuple(self.center[i] - h[i] for i in range(3))
        hi = tuple(self.center[i] + h[i] for i in range(3))
        return lo, hi


@dataclass
class RelationRecord:
    """A relation of the database."""

    object_a: str
    object_b: str
    type: str  # noqa: A003
    family: str = GEOMETRIC
    state: str = ACTIVE
    source: str = GEOMETRIC
    fastener: str = ''
    confidence: float = 1.0
    confirmed: bool = True
    observations: int = 1
    stamp_ns: int = 0
    id: int = 0  # noqa: A003


# ----------------------------------------------------------------------------
# Geometric relations
# ----------------------------------------------------------------------------
def _overlap_1d(lo1, hi1, lo2, hi2) -> float:
    return max(0.0, min(hi1, hi2) - max(lo1, lo2))


def _area(lo, hi) -> float:
    return max((hi[0] - lo[0]) * (hi[1] - lo[1]), 1e-9)


def _footprint_overlap(lo1, hi1, lo2, hi2) -> float:
    return _overlap_1d(lo1[0], hi1[0], lo2[0], hi2[0]) * \
        _overlap_1d(lo1[1], hi1[1], lo2[1], hi2[1])


def geometric_relations(a_id: str, a: Box, b_id: str, b: Box,
                        cfg: GeometricConfig) -> List[Tuple[str, str, str]]:
    """
    Return the pairwise geometric relations (inside, near, left_of, right_of).

    ``on_top_of`` depends on the whole scene: see :func:`support_relations`.

    :return: list of ``(object_a, type, object_b)`` triples.
    """
    out = []
    (alo, ahi), (blo, bhi) = a.aabb(), b.aabb()

    def volume(lo, hi) -> float:
        return _area(lo, hi) * max(hi[2] - lo[2], 1e-9)

    for (x, xlo, xhi), (y, ylo, yhi) in (((a_id, alo, ahi), (b_id, blo, bhi)),
                                         ((b_id, blo, bhi), (a_id, alo, ahi))):
        tol = cfg.inside_tolerance
        # Horizontally within the container (with tolerance), vertically below
        # its rim: an object lying on top of a flat box is on_top_of, not inside.
        inside = (all(xlo[i] >= ylo[i] - tol and xhi[i] <= yhi[i] + tol for i in range(2))
                  and xlo[2] >= ylo[2] - tol and xhi[2] <= yhi[2])
        if inside and volume(xlo, xhi) < volume(ylo, yhi):
            out.append((x, INSIDE, y))

    delta = [a.center[i] - b.center[i] for i in range(3)]
    if math.sqrt(sum(d * d for d in delta)) <= cfg.near_distance:
        out += [(a_id, NEAR, b_id), (b_id, NEAR, a_id)]
        lateral = sum(delta[i] * cfg.left_axis[i] for i in range(3))
        if lateral > cfg.lateral_tolerance:
            out += [(a_id, LEFT_OF, b_id), (b_id, RIGHT_OF, a_id)]
        elif lateral < -cfg.lateral_tolerance:
            out += [(b_id, LEFT_OF, a_id), (a_id, RIGHT_OF, b_id)]
    return out


def support_relations(boxes: Dict[str, Box], cfg: GeometricConfig
                      ) -> Set[Tuple[str, str, str]]:
    """
    Return the ``on_top_of`` relations of the whole scene.

    The support of A is the highest object B below A whose footprint covers
    at least ``on_top_min_overlap`` of A's footprint, with B's top not above
    A's bottom (``on_top_tolerance``) and not more than ``on_top_max_gap``
    below it. Objects with tops within ``on_top_tolerance`` of the highest
    one are all supports (an object resting on two others), except those
    covered by another candidate (a box under a thin lid).
    """
    aabbs = {k: b.aabb() for k, b in boxes.items()}
    out = set()
    for a, (alo, ahi) in aabbs.items():
        candidates = []
        for b, (blo, bhi) in aabbs.items():
            if b == a:
                continue
            gap = alo[2] - bhi[2]
            if not -cfg.on_top_tolerance <= gap <= cfg.on_top_max_gap:
                continue
            if 0.5 * (alo[2] + ahi[2]) <= 0.5 * (blo[2] + bhi[2]):
                continue
            if _footprint_overlap(alo, ahi, blo, bhi) / _area(alo, ahi) < cfg.on_top_min_overlap:
                continue
            candidates.append((bhi[2], b))
        if not candidates:
            continue
        top = max(c[0] for c in candidates)
        near_top = [b for z, b in candidates if z >= top - cfg.on_top_tolerance]
        for b in near_top:
            (blo, bhi) = aabbs[b]
            # A candidate covered by another one (a box under its thin lid)
            # is not a support.
            covered = any(
                c != b and aabbs[c][0][2] >= bhi[2] - cfg.on_top_tolerance
                and aabbs[c][1][2] > bhi[2]
                and _footprint_overlap(aabbs[c][0], aabbs[c][1], blo, bhi) > 0.0
                for c in near_top)
            if not covered:
                out.add((a, ON_TOP_OF, b))
    return out


def compute_geometric(boxes: Dict[str, Box], changed: Iterable[str],
                      cfg: GeometricConfig) -> Set[Tuple[str, str, str]]:
    """Pairwise relations of every pair that involves at least one changed object."""
    changed = [c for c in changed if c in boxes]
    done, out = set(), set()
    for c in changed:
        for other, box in boxes.items():
            if other == c or frozenset((c, other)) in done:
                continue
            done.add(frozenset((c, other)))
            out.update(geometric_relations(c, boxes[c], other, box, cfg))
    return out


# ----------------------------------------------------------------------------
# Blocking, free objects, disassembly order
# ----------------------------------------------------------------------------
@dataclass
class BlockingModel:
    """Relations relevant for picking, indexed for fast lookup."""

    relations: Sequence[RelationRecord]
    blocking_types: Sequence[str] = ('closed_by', 'held_by')
    _on_top: Dict[str, List[str]] = field(default_factory=dict, init=False)
    _mech: Dict[str, List[RelationRecord]] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        for r in self.relations:
            if r.family == GEOMETRIC and r.type == ON_TOP_OF:
                self._on_top.setdefault(r.object_b, []).append(r.object_a)
            elif r.family == MECHANICAL and r.state == TIGHTENED:
                self._mech.setdefault(r.object_a, []).append(r)

    def blockers_of(self, object_id: str) -> List[dict]:
        """Return what prevents picking the object (empty list = free)."""
        out = [{'kind': 'object_on_top', 'object_id': y, 'relation_id': None,
                'type': ON_TOP_OF, 'state': ACTIVE}
               for y in sorted(self._on_top.get(object_id, []))]
        for r in self._mech.get(object_id, []):
            kind = 'blocking_object' if r.type in self.blocking_types else 'fastening'
            out.append({'kind': kind, 'object_id': r.object_b, 'relation_id': r.id,
                        'type': r.type, 'state': r.state, 'confirmed': r.confirmed})
        return out

    def is_free(self, object_id: str) -> bool:
        """Return True if nothing blocks the object."""
        return not self.blockers_of(object_id)

    def supported_by(self, object_id: str) -> List[str]:
        """Objects resting on the object, directly or through other objects."""
        out, stack = [], list(self._on_top.get(object_id, []))
        while stack:
            y = stack.pop()
            if y in out or y == object_id:
                continue
            out.append(y)
            stack.extend(self._on_top.get(y, []))
        return sorted(out)

    def disassembly_order(self, targets: Sequence[str]) -> List[dict]:
        """
        Ordered steps to remove the targets (and whatever blocks them).

        Steps are ``{"action": "release", "relation_id", "object_id", "type"}``
        (change a fastening state) and ``{"action": "remove", "object_id"}``.

        :raises SceneError: On a cycle of blocking relations.
        """
        steps: List[dict] = []
        removed: Set[str] = set()
        released: Set[int] = set()
        visiting: Set[str] = set()

        def visit(x: str, path: List[str]) -> None:
            if x in removed:
                return
            if x in visiting:
                raise SceneError('Cycle in blocking relations: ' + ' -> '.join(path + [x]))
            visiting.add(x)
            for b in self.blockers_of(x):
                if b['kind'] in ('object_on_top', 'blocking_object'):
                    visit(b['object_id'], path + [x])
                elif b['relation_id'] not in released:
                    released.add(b['relation_id'])
                    steps.append({'action': 'release', 'relation_id': b['relation_id'],
                                  'object_id': x, 'other_id': b['object_id'],
                                  'type': b['type']})
            visiting.discard(x)
            removed.add(x)
            steps.append({'action': 'remove', 'object_id': x})

        for t in targets:
            visit(t, [])
        for i, s in enumerate(steps):
            s['step'] = i + 1
        return steps


def resolve_reference(ref: str, request_ids: Sequence[str]) -> Optional[str]:
    """Resolve ``#<n>`` (n-th object of an UpdateScene request) to an id."""
    if ref.startswith('#'):
        try:
            return request_ids[int(ref[1:])]
        except (ValueError, IndexError):
            return None
    return ref
