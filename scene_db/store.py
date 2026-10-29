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

r"""
SQLite scene store: the single source of truth of the scene. No ROS here.

Every write runs in one transaction: either all of it is applied, or none.
Every successful write increments ``scene_version``.

Schema (version 2)::

    meta(key TEXT PRIMARY KEY, value TEXT)
        -- schema_version, frame_id, scene_version, counter.<class>
    objects(id TEXT PRIMARY KEY, class_name TEXT, color TEXT, r REAL, g REAL, b REAL,
            px REAL, py REAL, pz REAL, qx REAL, qy REAL, qz REAL, qw REAL,
            sx REAL, sy REAL, sz REAL, state TEXT, confidence REAL,
            observations INTEGER, first_seen_ns INTEGER, last_seen_ns INTEGER,
            grasps_stamp_ns INTEGER)
    properties(object_id TEXT REFERENCES objects ON DELETE CASCADE,
               key TEXT, value TEXT, PRIMARY KEY (object_id, key))
    grasps(id INTEGER PRIMARY KEY AUTOINCREMENT,
           object_id TEXT REFERENCES objects ON DELETE CASCADE,
           px, py, pz, qx, qy, qz, qw REAL, score REAL, width REAL, height REAL,
           depth REAL, stamp_ns INTEGER)
    relations(id INTEGER PRIMARY KEY AUTOINCREMENT,
              object_a TEXT REFERENCES objects ON DELETE CASCADE,
              object_b TEXT REFERENCES objects ON DELETE CASCADE,
              type TEXT, family TEXT, fastener TEXT, state TEXT, source TEXT,
              confidence REAL, confirmed INTEGER, observations INTEGER, stamp_ns INTEGER,
              UNIQUE (object_a, object_b, type))

The file can be inspected with any SQLite tool, e.g.::

    sqlite3 ~/.ros/scene_db.sqlite "SELECT id, class_name, px, py, pz FROM objects"
"""

from contextlib import contextmanager
from dataclasses import dataclass, field
import math
import re
import sqlite3
import threading
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

from scene_db.relations import (
    A_PRIORI,
    ACTIVE,
    BlockingModel,
    Box,
    compute_geometric,
    GEOMETRIC,
    GeometricConfig,
    INFERRED,
    MECHANICAL,
    MECHANICAL_STATES,
    ON_TOP_OF,
    RelationRecord,
    resolve_reference,
    SceneError,
    support_relations,
)

SCHEMA_VERSION = '2'
NS_PER_S = 1_000_000_000

STATE_OK = 'ok'
STATE_TO_VERIFY = 'to_verify'

# Event kinds (same strings as scene_msgs/SceneEvent).
OBJECT_ADDED = 'object_added'
OBJECT_MOVED = 'object_moved'
OBJECT_UPDATED = 'object_updated'
OBJECT_REMOVED = 'object_removed'
RELATION_CHANGED = 'relation_changed'
SCENE_CLEARED = 'scene_cleared'

_SCHEMA = """
CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS objects (
    id TEXT PRIMARY KEY, class_name TEXT NOT NULL, color TEXT, r REAL, g REAL, b REAL,
    px REAL, py REAL, pz REAL, qx REAL, qy REAL, qz REAL, qw REAL,
    sx REAL, sy REAL, sz REAL, state TEXT, confidence REAL, observations INTEGER,
    first_seen_ns INTEGER, last_seen_ns INTEGER, grasps_stamp_ns INTEGER);
CREATE TABLE IF NOT EXISTS properties (
    object_id TEXT NOT NULL REFERENCES objects(id) ON DELETE CASCADE,
    key TEXT NOT NULL, value TEXT, PRIMARY KEY (object_id, key));
CREATE TABLE IF NOT EXISTS grasps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_id TEXT NOT NULL REFERENCES objects(id) ON DELETE CASCADE,
    px REAL, py REAL, pz REAL, qx REAL, qy REAL, qz REAL, qw REAL,
    score REAL, width REAL, height REAL, depth REAL, stamp_ns INTEGER);
CREATE TABLE IF NOT EXISTS relations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_a TEXT NOT NULL REFERENCES objects(id) ON DELETE CASCADE,
    object_b TEXT NOT NULL REFERENCES objects(id) ON DELETE CASCADE,
    type TEXT NOT NULL, family TEXT, fastener TEXT, state TEXT, source TEXT,
    confidence REAL, confirmed INTEGER, observations INTEGER, stamp_ns INTEGER,
    UNIQUE (object_a, object_b, type));
CREATE INDEX IF NOT EXISTS objects_class ON objects(class_name);
CREATE INDEX IF NOT EXISTS grasps_object ON grasps(object_id);
CREATE INDEX IF NOT EXISTS relations_a ON relations(object_a);
CREATE INDEX IF NOT EXISTS relations_b ON relations(object_b);
"""

OBJECT_FILTERS = ('id', 'class', 'color', 'state', 'region', 'near', 'radius', 'max_age',
                  'min_confidence', 'min_grasp_score', 'free_only', 'include_grasps',
                  'max_results')
RELATION_FILTERS = ('object_id', 'other_id', 'object_a', 'object_b', 'type', 'family',
                    'state', 'source', 'min_confidence', 'confirmed')


# ----------------------------------------------------------------------------
# Records
# ----------------------------------------------------------------------------
@dataclass
class GraspRecord:
    """A grasp in the fixed frame."""

    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    score: float
    width: float = 0.0
    height: float = 0.0
    depth: float = 0.0
    stamp_ns: int = 0
    id: int = 0  # noqa: A003


@dataclass
class ObjectRecord:
    """An object of the database."""

    id: str  # noqa: A003
    class_name: str
    center: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    size: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: str = ''
    rgb: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    state: str = STATE_OK
    confidence: float = 0.0
    observations: int = 0
    first_seen_ns: int = 0
    last_seen_ns: int = 0
    grasps_stamp_ns: int = 0
    properties: Dict[str, str] = field(default_factory=dict)
    grasps: List[GraspRecord] = field(default_factory=list)

    @property
    def box(self) -> Box:
        """Box of the object."""
        return Box(self.center, self.quaternion, self.size)


@dataclass
class ObjectInput:
    """One observed object of an update (already associated)."""

    id: str  # noqa: A003
    """Existing object id, or empty for a new object."""
    class_name: str
    score: float
    center: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    size: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: str = ''
    rgb: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    moved: bool = False
    update_grasps: bool = False
    grasps: List[GraspRecord] = field(default_factory=list)


@dataclass
class Event:
    """A change notification."""

    kind: str
    object_id: str = ''
    relation_id: int = 0
    details: str = ''


@dataclass
class WriteResult:
    """Outcome of a write."""

    scene_version: int
    events: List[Event] = field(default_factory=list)
    object_ids: List[str] = field(default_factory=list)
    changed_object_ids: List[str] = field(default_factory=list)
    to_verify_ids: List[str] = field(default_factory=list)
    relation_id: int = 0
    message: str = ''


@dataclass
class StoreConfig:
    """Behavior of the store."""

    geometric: GeometricConfig = field(default_factory=GeometricConfig)
    inferred_min_confidence: float = 0.7
    """Inferred relations below this confidence need a second observation."""
    blocking_types: Tuple[str, ...] = ('closed_by', 'held_by')


def _parse_floats(text: str, n: int, key: str) -> List[float]:
    try:
        values = [float(v) for v in text.split(',')]
    except ValueError:
        values = []
    if len(values) != n:
        raise SceneError(f"Filter '{key}' needs {n} comma-separated numbers, got '{text}'")
    return values


def _parse_bool(text: str, key: str) -> bool:
    if text.lower() in ('true', '1', 'yes'):
        return True
    if text.lower() in ('false', '0', 'no'):
        return False
    raise SceneError(f"Filter '{key}' must be true or false, got '{text}'")


def _parse_number(text: str, key: str, cast=float):
    try:
        return cast(text)
    except ValueError:
        raise SceneError(f"Filter '{key}' must be a number, got '{text}'")


_ID_RE = re.compile(r'^(.*)_(\d+)$')


def id_sort_key(object_id: str) -> tuple:
    """Natural sort key: ``cup_2`` before ``cup_10``."""
    m = _ID_RE.match(object_id)
    return (m.group(1), int(m.group(2))) if m else (object_id, -1)


# ----------------------------------------------------------------------------
# Store
# ----------------------------------------------------------------------------
class SceneStore:
    """SQLite-backed scene database. Thread-safe (one lock around every call)."""

    def __init__(self, path: str, frame_id: str, config: Optional[StoreConfig] = None):
        """
        Open (or create) the database.

        :param path: SQLite file, or ``:memory:``.
        :param frame_id: Fixed frame of the stored geometry. An existing file
            saved with a different frame is rejected.
        :raises SceneError: On a frame or schema mismatch.
        """
        self.config = config or StoreConfig()
        self.frame_id = frame_id
        self._lock = threading.RLock()
        self._con = sqlite3.connect(path, isolation_level=None, check_same_thread=False)
        self._con.execute('PRAGMA foreign_keys = ON')
        if path != ':memory:':
            self._con.execute('PRAGMA journal_mode = WAL')
        self._con.executescript(_SCHEMA)
        meta = self._meta()
        if not meta:
            with self._tx():
                self._set_meta('schema_version', SCHEMA_VERSION)
                self._set_meta('frame_id', frame_id)
                self._set_meta('scene_version', '0')
        else:
            if meta.get('schema_version') != SCHEMA_VERSION:
                raise SceneError(f"{path}: schema version {meta.get('schema_version')}, "
                                 f'expected {SCHEMA_VERSION}')
            if meta.get('frame_id') != frame_id:
                raise SceneError(f"{path} stores objects in frame '{meta.get('frame_id')}', "
                                 f"but the fixed frame is '{frame_id}'")

    def close(self) -> None:
        """Close the connection."""
        with self._lock:
            self._con.close()

    # --- Low level ------------------------------------------------------------
    @contextmanager
    def _tx(self) -> Iterator[sqlite3.Connection]:
        """Run a block in one transaction (rolled back on any exception)."""
        with self._lock:
            self._con.execute('BEGIN IMMEDIATE')
            try:
                yield self._con
            except BaseException:
                self._con.execute('ROLLBACK')
                raise
            self._con.execute('COMMIT')

    def _meta(self) -> Dict[str, str]:
        return dict(self._con.execute('SELECT key, value FROM meta'))

    def _set_meta(self, key: str, value: str) -> None:
        self._con.execute('INSERT OR REPLACE INTO meta VALUES (?, ?)', (key, value))

    @property
    def scene_version(self) -> int:
        """Return the current version (incremented by every write)."""
        with self._lock:
            row = self._con.execute(
                "SELECT value FROM meta WHERE key = 'scene_version'").fetchone()
            return int(row[0]) if row else 0

    def _bump(self) -> int:
        version = self.scene_version + 1
        self._set_meta('scene_version', str(version))
        return version

    def _new_id(self, class_name: str) -> str:
        prefix = re.sub(r'\s+', '_', class_name.strip()) or 'object'
        key = f'counter.{prefix}'
        row = self._con.execute('SELECT value FROM meta WHERE key = ?', (key,)).fetchone()
        n = int(row[0]) if row else 0
        while True:
            n += 1
            object_id = f'{prefix}_{n}'
            if not self._con.execute('SELECT 1 FROM objects WHERE id = ?',
                                     (object_id,)).fetchone():
                self._set_meta(key, str(n))
                return object_id

    # --- Reading ----------------------------------------------------------------
    def objects(self, include_grasps: bool = True) -> List[ObjectRecord]:
        """All objects, sorted by id."""
        with self._lock:
            props: Dict[str, Dict[str, str]] = {}
            for oid, k, v in self._con.execute('SELECT object_id, key, value FROM properties'):
                props.setdefault(oid, {})[k] = v
            grasps: Dict[str, List[GraspRecord]] = {}
            if include_grasps:
                for row in self._con.execute(
                        'SELECT object_id, id, px, py, pz, qx, qy, qz, qw, score, width, '
                        'height, depth, stamp_ns FROM grasps ORDER BY score DESC'):
                    grasps.setdefault(row[0], []).append(GraspRecord(
                        position=tuple(row[2:5]), quaternion=tuple(row[5:9]), score=row[9],
                        width=row[10], height=row[11], depth=row[12], stamp_ns=row[13],
                        id=row[1]))
            out = []
            for row in self._con.execute(
                    'SELECT id, class_name, color, r, g, b, px, py, pz, qx, qy, qz, qw, '
                    'sx, sy, sz, state, confidence, observations, first_seen_ns, '
                    'last_seen_ns, grasps_stamp_ns FROM objects'):
                out.append(ObjectRecord(
                    id=row[0], class_name=row[1], color=row[2] or '', rgb=tuple(row[3:6]),
                    center=tuple(row[6:9]), quaternion=tuple(row[9:13]),
                    size=tuple(row[13:16]), state=row[16], confidence=row[17],
                    observations=row[18], first_seen_ns=row[19], last_seen_ns=row[20],
                    grasps_stamp_ns=row[21], properties=props.get(row[0], {}),
                    grasps=grasps.get(row[0], [])))
            return sorted(out, key=lambda o: id_sort_key(o.id))

    def get(self, object_id: str) -> Optional[ObjectRecord]:
        """One object by id."""
        return next((o for o in self.objects() if o.id == object_id), None)

    def relations(self) -> List[RelationRecord]:
        """All relations, sorted by id."""
        with self._lock:
            return [RelationRecord(
                id=r[0], object_a=r[1], object_b=r[2], type=r[3], family=r[4],
                fastener=r[5] or '', state=r[6], source=r[7], confidence=r[8],
                confirmed=bool(r[9]), observations=r[10], stamp_ns=r[11])
                for r in self._con.execute(
                    'SELECT id, object_a, object_b, type, family, fastener, state, source, '
                    'confidence, confirmed, observations, stamp_ns FROM relations ORDER BY id')]

    def blocking_model(self) -> BlockingModel:
        """Blocking logic over the current relations."""
        return BlockingModel(self.relations(), self.config.blocking_types)

    # --- Queries ----------------------------------------------------------------
    def query_objects(self, filters: Dict[str, str], now_ns: int
                      ) -> Tuple[List[ObjectRecord], List[float]]:
        """
        Filter objects (see scene_msgs/QueryObjects for the keys).

        :return: ``(objects, distances)``; distances only with ``near``.
        :raises SceneError: On unknown keys or malformed values.
        """
        for key in filters:
            if key not in OBJECT_FILTERS and not key.startswith('property.'):
                raise SceneError(f"Unknown filter '{key}'. Allowed: "
                                 f"{', '.join(OBJECT_FILTERS)}, property.<key>")
        f = filters
        include_grasps = _parse_bool(f.get('include_grasps', 'true'), 'include_grasps')
        region = _parse_floats(f['region'], 6, 'region') if 'region' in f else None
        near = _parse_floats(f['near'], 3, 'near') if 'near' in f else None
        radius = _parse_number(f.get('radius', '0'), 'radius')
        max_age = _parse_number(f.get('max_age', '0'), 'max_age')
        min_conf = _parse_number(f.get('min_confidence', '0'), 'min_confidence')
        min_grasp = (_parse_number(f['min_grasp_score'], 'min_grasp_score')
                     if 'min_grasp_score' in f else None)
        free_only = _parse_bool(f.get('free_only', 'false'), 'free_only')
        max_results = _parse_number(f.get('max_results', '0'), 'max_results', int)
        props = {k[len('property.'):]: v for k, v in f.items() if k.startswith('property.')}
        model = self.blocking_model() if free_only else None

        results = []
        for o in self.objects(include_grasps=True):
            if 'id' in f and o.id != f['id']:
                continue
            if 'class' in f and o.class_name != f['class']:
                continue
            if 'color' in f and o.color != f['color']:
                continue
            if 'state' in f and o.state != f['state']:
                continue
            if region is not None:
                lo, hi = region[0::2], region[1::2]
                if not all(lo[i] <= o.center[i] <= hi[i] for i in range(3)):
                    continue
            if max_age > 0 and now_ns - o.last_seen_ns > max_age * NS_PER_S:
                continue
            if o.confidence < min_conf:
                continue
            if min_grasp is not None and not any(g.score >= min_grasp for g in o.grasps):
                continue
            if any(k not in o.properties or (v and o.properties[k] != v)
                   for k, v in props.items()):
                continue
            if model is not None and not model.is_free(o.id):
                continue
            dist = math.dist(o.center, near) if near is not None else 0.0
            if near is not None and radius > 0 and dist > radius:
                continue
            if not include_grasps:
                o.grasps = []
            results.append((o, dist))
        if near is not None:
            results.sort(key=lambda r: r[1])
        if max_results > 0:
            results = results[:max_results]
        return [o for o, _ in results], ([d for _, d in results] if near is not None else [])

    def query_relations(self, filters: Dict[str, str]) -> List[RelationRecord]:
        """Filter relations (see scene_msgs/QueryRelations for the keys)."""
        for key in filters:
            if key not in RELATION_FILTERS:
                raise SceneError(f"Unknown filter '{key}'. Allowed: {', '.join(RELATION_FILTERS)}")
        f = filters
        min_conf = _parse_number(f.get('min_confidence', '0'), 'min_confidence')
        confirmed = _parse_bool(f['confirmed'], 'confirmed') if 'confirmed' in f else None
        out = []
        for r in self.relations():
            if 'object_id' in f:
                if f['object_id'] not in (r.object_a, r.object_b):
                    continue
                other = r.object_b if r.object_a == f['object_id'] else r.object_a
                if 'other_id' in f and other != f['other_id']:
                    continue
            if any(k in f and getattr(r, k) != f[k]
                   for k in ('object_a', 'object_b', 'type', 'family', 'state', 'source')):
                continue
            if r.confidence < min_conf or (confirmed is not None and r.confirmed != confirmed):
                continue
            out.append(r)
        return out

    def composite_query(self, query_type: str, params: Dict[str, str]) -> List[dict]:
        """Composite queries (see scene_msgs/Query)."""
        model = self.blocking_model()
        objects = {o.id: o for o in self.objects()}

        def need_object() -> str:
            oid = params.get('object_id', '')
            if oid not in objects:
                raise SceneError(f"Query '{query_type}' needs an existing object_id, "
                                 f"got '{oid}'")
            return oid

        if query_type == 'free_objects':
            return [{'id': o.id, 'class': o.class_name, 'color': o.color,
                     'center': list(o.center), 'state': o.state,
                     'best_grasp_score': o.grasps[0].score if o.grasps else None}
                    for o in objects.values() if model.is_free(o.id)]
        if query_type == 'blockers_of':
            return model.blockers_of(need_object())
        if query_type == 'supported_by':
            return [{'id': oid} for oid in model.supported_by(need_object())]
        if query_type == 'disassembly_order':
            targets = ([need_object()] if params.get('object_id')
                       else sorted(objects, key=id_sort_key))
            return model.disassembly_order(targets)
        raise SceneError(f"Unknown query_type '{query_type}'. Allowed: free_objects, "
                         'blockers_of, supported_by, disassembly_order')

    # --- Writing ----------------------------------------------------------------
    def update_scene(self, stamp_ns: int, objects: Sequence[ObjectInput],
                     relations: Sequence[RelationRecord] = ()) -> WriteResult:
        """
        Apply one acquisition atomically.

        Matched objects (``id`` set) are updated, new ones (``id`` empty) are
        inserted with a new id. Grasps are replaced where ``update_grasps``.
        Geometric relations of added/moved objects are recomputed. External
        relations are upserted (ends may be ``#<n>``).

        :raises SceneError: (nothing is written) on unknown ids or bad relations.
        """
        events: List[Event] = []
        with self._tx() as con:
            ids, changed = [], []
            for o in objects:
                if o.id:
                    row = con.execute('SELECT observations, confidence FROM objects '
                                      'WHERE id = ?', (o.id,)).fetchone()
                    if row is None:
                        raise SceneError(f"Unknown object id '{o.id}'")
                    n = row[0] + 1
                    con.execute(
                        'UPDATE objects SET class_name=?, color=?, r=?, g=?, b=?, px=?, py=?, '
                        'pz=?, qx=?, qy=?, qz=?, qw=?, sx=?, sy=?, sz=?, state=?, '
                        'confidence=?, observations=?, last_seen_ns=? WHERE id=?',
                        (o.class_name, o.color, *o.rgb, *o.center, *o.quaternion, *o.size,
                         STATE_OK, row[1] + (o.score - row[1]) / n, n, stamp_ns, o.id))
                    oid = o.id
                    if o.moved:
                        changed.append(oid)
                        events.append(Event(OBJECT_MOVED, oid, details='pose changed'))
                else:
                    oid = self._new_id(o.class_name)
                    con.execute(
                        'INSERT INTO objects VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                        (oid, o.class_name, o.color, *o.rgb, *o.center, *o.quaternion,
                         *o.size, STATE_OK, o.score, 1, stamp_ns, stamp_ns, 0))
                    changed.append(oid)
                    events.append(Event(OBJECT_ADDED, oid, details=o.class_name))
                if o.update_grasps:
                    con.execute('DELETE FROM grasps WHERE object_id = ?', (oid,))
                    con.executemany(
                        'INSERT INTO grasps (object_id, px, py, pz, qx, qy, qz, qw, score, '
                        'width, height, depth, stamp_ns) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)',
                        [(oid, *g.position, *g.quaternion, g.score, g.width, g.height,
                          g.depth, g.stamp_ns or stamp_ns) for g in o.grasps])
                    con.execute('UPDATE objects SET grasps_stamp_ns = ? WHERE id = ?',
                                (stamp_ns, oid))
                    if o.id and not o.moved:
                        events.append(Event(OBJECT_UPDATED, oid,
                                            details=f'{len(o.grasps)} grasps'))
                ids.append(oid)

            messages = []
            for r in relations:
                a, b = resolve_reference(r.object_a, ids), resolve_reference(r.object_b, ids)
                if a is None or b is None:
                    raise SceneError(f'Bad relation reference {r.object_a} -> {r.object_b}')
                rel = RelationRecord(**{**r.__dict__, 'object_a': a, 'object_b': b,
                                        'stamp_ns': r.stamp_ns or stamp_ns})
                rid, msg = self._upsert_relation(rel, events)
                if msg:
                    messages.append(msg)

            self._recompute_geometric(changed, events)
            version = self._bump()
        return WriteResult(version, events, ids, changed,
                           message='; '.join(messages))

    def _upsert_relation(self, r: RelationRecord, events: List[Event]) -> Tuple[int, str]:
        """Insert or update an external relation, applying the source policy."""
        con = self._con
        if r.source == GEOMETRIC or r.family == GEOMETRIC:
            raise SceneError('Geometric relations are computed by scene_db; '
                             'use family=mechanical with source a_priori or inferred')
        if r.source not in (A_PRIORI, INFERRED):
            raise SceneError(f"Relation source must be a_priori or inferred, got '{r.source}'")
        if r.state not in MECHANICAL_STATES:
            raise SceneError(f'Mechanical state must be one of {MECHANICAL_STATES}, '
                             f"got '{r.state}'")
        for oid in (r.object_a, r.object_b):
            if not con.execute('SELECT 1 FROM objects WHERE id = ?', (oid,)).fetchone():
                raise SceneError(f"Relation refers to unknown object '{oid}'")
        if r.object_a == r.object_b:
            raise SceneError('A relation needs two different objects')

        row = con.execute(
            'SELECT id, source, state, confidence, observations, stamp_ns, confirmed '
            'FROM relations WHERE object_a=? AND object_b=? AND type=?',
            (r.object_a, r.object_b, r.type)).fetchone()
        threshold = self.config.inferred_min_confidence
        desc = f'{r.object_a} {r.type} {r.object_b}'

        if row is None:
            confirmed = r.source == A_PRIORI or r.confidence >= threshold
            cur = con.execute(
                'INSERT INTO relations (object_a, object_b, type, family, fastener, state, '
                'source, confidence, confirmed, observations, stamp_ns) '
                'VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                (r.object_a, r.object_b, r.type, MECHANICAL, r.fastener, r.state, r.source,
                 r.confidence, int(confirmed), 1, r.stamp_ns))
            events.append(Event(RELATION_CHANGED, r.object_a, cur.lastrowid,
                                f'added {desc} ({r.source}, {r.state})'))
            return cur.lastrowid, ''

        rid, src, state, conf, obs, last_stamp, confirmed = row
        if r.source == INFERRED and src == A_PRIORI:
            # A priori knowledge prevails: only count the observation.
            con.execute('UPDATE relations SET observations=? WHERE id=?', (obs + 1, rid))
            if r.state != state:
                return rid, (f'conflict on {desc}: inferred {r.state}, a priori {state} '
                             '(a priori kept)')
            return rid, ''
        if r.source == INFERRED:
            # A low-confidence relation is confirmed by a second observation
            # from another acquisition.
            confirmed = bool(confirmed) or r.confidence >= threshold or \
                (r.stamp_ns != last_stamp)
            con.execute('UPDATE relations SET state=?, fastener=?, confidence=?, confirmed=?, '
                        'observations=?, stamp_ns=? WHERE id=?',
                        (r.state, r.fastener, max(conf, r.confidence), int(confirmed),
                         obs + 1, r.stamp_ns, rid))
        else:  # a priori overwrites everything
            con.execute('UPDATE relations SET family=?, fastener=?, state=?, source=?, '
                        'confidence=?, confirmed=1, observations=?, stamp_ns=? WHERE id=?',
                        (MECHANICAL, r.fastener, r.state, A_PRIORI, r.confidence, obs + 1,
                         r.stamp_ns, rid))
        if r.state != state or r.source != src:
            events.append(Event(RELATION_CHANGED, r.object_a, rid,
                                f'{desc}: {src}/{state} -> {r.source}/{r.state}'))
        return rid, ''

    def _recompute_geometric(self, changed: Sequence[str], events: List[Event]) -> None:
        """Replace the geometric relations of the changed objects (inside a transaction)."""
        if not changed:
            return
        con = self._con
        boxes = {row[0]: Box(tuple(row[1:4]), tuple(row[4:8]), tuple(row[8:11]))
                 for row in con.execute('SELECT id, px, py, pz, qx, qy, qz, qw, sx, sy, sz '
                                        'FROM objects')}
        marks = ','.join('?' * len(changed))
        # Pairwise relations of the changed objects, plus every on_top_of
        # (supports depend on the whole scene).
        old = {(a, t, b): rid for rid, a, t, b in con.execute(
            f'SELECT id, object_a, type, object_b FROM relations WHERE family = ? AND '
            f'(type = ? OR object_a IN ({marks}) OR object_b IN ({marks}))',
            (GEOMETRIC, ON_TOP_OF, *changed, *changed))}
        new = compute_geometric(boxes, changed, self.config.geometric) | \
            support_relations(boxes, self.config.geometric)
        for triple in set(old) - new:
            con.execute('DELETE FROM relations WHERE id = ?', (old[triple],))
            events.append(Event(RELATION_CHANGED, triple[0], old[triple],
                                f'removed {triple[0]} {triple[1]} {triple[2]}'))
        for a, t, b in sorted(new - set(old)):
            cur = con.execute(
                'INSERT INTO relations (object_a, object_b, type, family, fastener, state, '
                'source, confidence, confirmed, observations, stamp_ns) '
                "VALUES (?,?,?,?,'',?,?,1.0,1,1,0)", (a, b, t, GEOMETRIC, ACTIVE, GEOMETRIC))
            events.append(Event(RELATION_CHANGED, a, cur.lastrowid, f'added {a} {t} {b}'))

    def remove_object(self, object_id: str) -> WriteResult:
        """
        Remove an object, its grasps and relations in one transaction.

        Objects that were resting on it become ``to_verify``; geometric
        relations of the former neighbors are recomputed.
        """
        events: List[Event] = []
        with self._tx() as con:
            if not con.execute('SELECT 1 FROM objects WHERE id = ?', (object_id,)).fetchone():
                raise SceneError(f"Unknown object id '{object_id}'")
            supported = [r[0] for r in con.execute(
                'SELECT object_a FROM relations WHERE object_b = ? AND type = ? AND family = ?',
                (object_id, ON_TOP_OF, GEOMETRIC))]
            neighbors = [r[0] for r in con.execute(
                'SELECT object_b FROM relations WHERE object_a = ? AND family = ? '
                'UNION SELECT object_a FROM relations WHERE object_b = ? AND family = ?',
                (object_id, GEOMETRIC, object_id, GEOMETRIC))]
            con.execute('DELETE FROM objects WHERE id = ?', (object_id,))  # cascades
            events.append(Event(OBJECT_REMOVED, object_id))
            for oid in supported:
                con.execute('UPDATE objects SET state = ? WHERE id = ?', (STATE_TO_VERIFY, oid))
                events.append(Event(OBJECT_UPDATED, oid,
                                    details=f'to_verify: its support {object_id} was removed'))
            self._recompute_geometric(sorted(set(neighbors)), events)
            version = self._bump()
        return WriteResult(version, events, to_verify_ids=sorted(supported),
                           message=f"Removed '{object_id}'")

    def set_relation_state(self, relation_id: int, new_state: str) -> WriteResult:
        """Change the state of a mechanical relation."""
        with self._tx() as con:
            row = con.execute('SELECT family, state, object_a, type, object_b FROM relations '
                              'WHERE id = ?', (relation_id,)).fetchone()
            if row is None:
                raise SceneError(f'Unknown relation id {relation_id}')
            if row[0] != MECHANICAL:
                raise SceneError('Only mechanical relations have a state to set')
            if new_state not in MECHANICAL_STATES:
                raise SceneError(f'State must be one of {MECHANICAL_STATES}')
            con.execute('UPDATE relations SET state = ? WHERE id = ?', (new_state, relation_id))
            version = self._bump()
        desc = f'{row[2]} {row[3]} {row[4]}: {row[1]} -> {new_state}'
        return WriteResult(version, [Event(RELATION_CHANGED, row[2], relation_id, desc)],
                           message=desc)

    def add_relation(self, r: RelationRecord) -> WriteResult:
        """Add or update a mechanical relation between existing objects."""
        events: List[Event] = []
        with self._tx():
            rid, msg = self._upsert_relation(r, events)
            version = self._bump()
        return WriteResult(version, events, relation_id=rid, message=msg or f'Relation {rid}')

    def remove_relation(self, relation_id: int) -> WriteResult:
        """Delete a mechanical relation (geometric ones are recomputed, not deleted)."""
        with self._tx() as con:
            row = con.execute('SELECT family, object_a FROM relations WHERE id = ?',
                              (relation_id,)).fetchone()
            if row is None:
                raise SceneError(f'Unknown relation id {relation_id}')
            if row[0] == GEOMETRIC:
                raise SceneError('Geometric relations are derived from the boxes')
            con.execute('DELETE FROM relations WHERE id = ?', (relation_id,))
            version = self._bump()
        return WriteResult(version, [Event(RELATION_CHANGED, row[1], relation_id, 'removed')],
                           message=f'Removed relation {relation_id}')

    def set_grasps(self, object_id: str, grasps: Sequence[GraspRecord],
                   stamp_ns: int) -> WriteResult:
        """Replace the grasps of one object; its pose and statistics are kept."""
        with self._tx() as con:
            if not con.execute('SELECT 1 FROM objects WHERE id = ?', (object_id,)).fetchone():
                raise SceneError(f"Unknown object id '{object_id}'")
            con.execute('DELETE FROM grasps WHERE object_id = ?', (object_id,))
            con.executemany(
                'INSERT INTO grasps (object_id, px, py, pz, qx, qy, qz, qw, score, '
                'width, height, depth, stamp_ns) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)',
                [(object_id, *g.position, *g.quaternion, g.score, g.width, g.height,
                  g.depth, g.stamp_ns or stamp_ns) for g in grasps])
            con.execute('UPDATE objects SET grasps_stamp_ns = ? WHERE id = ?',
                        (stamp_ns, object_id))
            version = self._bump()
        return WriteResult(version, [Event(OBJECT_UPDATED, object_id,
                                           details=f'{len(grasps)} grasps')],
                           message=f"{len(grasps)} grasps stored for '{object_id}'")

    def set_properties(self, object_id: str, properties: Dict[str, str]) -> WriteResult:
        """Add or overwrite user properties."""
        if any(not k for k in properties):
            raise SceneError('Property keys must not be empty')
        with self._tx() as con:
            if not con.execute('SELECT 1 FROM objects WHERE id = ?', (object_id,)).fetchone():
                raise SceneError(f"Unknown object id '{object_id}'")
            con.executemany('INSERT OR REPLACE INTO properties VALUES (?, ?, ?)',
                            [(object_id, k, v) for k, v in properties.items()])
            version = self._bump()
        return WriteResult(version, [Event(OBJECT_UPDATED, object_id, details='properties')],
                           message=f"Properties of '{object_id}' updated")

    def remove_properties(self, object_id: str, keys: Sequence[str]) -> WriteResult:
        """Remove user properties (missing keys are ignored)."""
        with self._tx() as con:
            if not con.execute('SELECT 1 FROM objects WHERE id = ?', (object_id,)).fetchone():
                raise SceneError(f"Unknown object id '{object_id}'")
            con.executemany('DELETE FROM properties WHERE object_id = ? AND key = ?',
                            [(object_id, k) for k in keys])
            version = self._bump()
        return WriteResult(version, [Event(OBJECT_UPDATED, object_id, details='properties')],
                           message=f"Properties of '{object_id}' updated")

    def clear(self) -> WriteResult:
        """Remove everything; ids restart from 1. The version keeps increasing."""
        with self._tx() as con:
            con.execute('DELETE FROM objects')   # cascades to grasps, relations, properties
            con.execute("DELETE FROM meta WHERE key LIKE 'counter.%'")
            version = self._bump()
        return WriteResult(version, [Event(SCENE_CLEARED)], message='Scene cleared')
