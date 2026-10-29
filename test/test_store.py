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

"""Unit tests for the SQLite scene store and the relation logic (no ROS)."""

import json

import pytest

from scene_db.relations import (
    BlockingModel,
    Box,
    geometric_relations,
    GeometricConfig,
    RelationRecord,
    SceneError,
    support_relations,
)
from scene_db.store import GraspRecord, ObjectInput, SceneStore, STATE_TO_VERIFY

S = 1_000_000_000


def obj(cls, center, size=(0.1, 0.1, 0.1), oid='', **kw):
    """Build an ObjectInput."""
    return ObjectInput(id=oid, class_name=cls, score=kw.pop('score', 0.9), center=center,
                       size=size, **kw)


def mech(a, b, typ, state='tightened', source='a_priori', confidence=1.0, stamp=0):
    """Build a mechanical relation."""
    return RelationRecord(object_a=a, object_b=b, type=typ, family='mechanical', state=state,
                          source=source, confidence=confidence, stamp_ns=stamp)


@pytest.fixture
def store():
    s = SceneStore(':memory:', 'world')
    yield s
    s.close()


def stack_scene(store):
    """Table, a box on the table, a cup on the box, a bowl beside the box."""
    return store.update_scene(1 * S, [
        obj('table', (0.0, 0.0, 0.35), (1.0, 1.0, 0.7)),
        obj('box', (0.0, 0.0, 0.75), (0.2, 0.2, 0.1)),
        obj('cup', (0.0, 0.0, 0.85), (0.08, 0.08, 0.1)),
        obj('bowl', (0.0, 0.3, 0.75), (0.15, 0.15, 0.1)),
    ])


def test_geometric_relations_pairwise():
    cfg = GeometricConfig()
    table = Box((0, 0, 0.35), (0, 0, 0, 1), (1, 1, 0.7))
    cup = Box((0, 0.2, 0.75), (0, 0, 0, 1), (0.1, 0.1, 0.1))
    assert support_relations({'cup': cup, 'table': table}, cfg) == {
        ('cup', 'on_top_of', 'table')}
    top_only = Box((0, 0.2, 0.84), (0, 0, 0, 1), (0.1, 0.1, 0.005))   # visible top surface
    assert ('t', 'on_top_of', 'table') in support_relations({'t': top_only, 'table': table}, cfg)
    small = Box((0, 0, 0.75), (0, 0, 0, 1), (0.05, 0.05, 0.05))
    big = Box((0, 0, 0.75), (0, 0, 0, 1), (0.3, 0.3, 0.2))
    assert ('s', 'inside', 'b') in geometric_relations('s', small, 'b', big, cfg)
    flat = Box((0, 0, 0.03), (0, 0, 0, 1), (0.3, 0.3, 0.06))
    lying = Box((0, 0, 0.065), (0, 0, 0, 1), (0.05, 0.05, 0.01))   # on the flat box
    assert ('l', 'inside', 'f') not in geometric_relations('l', lying, 'f', flat, cfg)
    left = Box((0, 0.2, 0.75), (0, 0, 0, 1), (0.1, 0.1, 0.1))
    right = Box((0, 0.0, 0.75), (0, 0, 0, 1), (0.1, 0.1, 0.1))
    rel = set(geometric_relations('l', left, 'r', right, cfg))
    assert {('l', 'near', 'r'), ('l', 'left_of', 'r'), ('r', 'right_of', 'l')} <= rel


def test_update_assigns_ids_and_relations(store):
    res = stack_scene(store)
    assert res.object_ids == ['table_1', 'box_1', 'cup_1', 'bowl_1']
    assert res.scene_version == 1 and set(res.changed_object_ids) == set(res.object_ids)
    triples = {(r.object_a, r.type, r.object_b) for r in store.relations()}
    assert {('box_1', 'on_top_of', 'table_1'), ('cup_1', 'on_top_of', 'box_1'),
            ('bowl_1', 'on_top_of', 'table_1'), ('bowl_1', 'left_of', 'box_1')} <= triples
    assert ('cup_1', 'on_top_of', 'table_1') not in triples
    assert sum(e.kind == 'object_added' for e in res.events) == 4


def test_update_existing_and_grasps(store):
    stack_scene(store)
    g = [GraspRecord((0, 0, 0.9), (0, 0, 0, 1), 0.8), GraspRecord((0, 0, 0.9), (0, 0, 0, 1), 0.95)]
    res = store.update_scene(2 * S, [obj('cup', (0.0, 0.0, 0.85), oid='cup_1', score=0.7,
                                         update_grasps=True, grasps=g)])
    cup = store.get('cup_1')
    assert cup.observations == 2 and cup.confidence == pytest.approx(0.8)
    assert [x.score for x in cup.grasps] == pytest.approx([0.95, 0.8])
    assert cup.grasps_stamp_ns == 2 * S and cup.last_seen_ns == 2 * S
    assert res.changed_object_ids == [] and res.events[0].kind == 'object_updated'
    with pytest.raises(SceneError):
        store.update_scene(3 * S, [obj('cup', (0, 0, 0), oid='cup_99')])
    assert store.scene_version == 2   # failed update: nothing written


def test_moved_object_recomputes_relations(store):
    stack_scene(store)
    res = store.update_scene(2 * S, [obj('cup', (0.0, -0.3, 0.75), oid='cup_1', moved=True)])
    triples = {(r.object_a, r.type, r.object_b) for r in store.relations()}
    assert ('cup_1', 'on_top_of', 'table_1') in triples
    assert ('cup_1', 'on_top_of', 'box_1') not in triples
    assert any(e.kind == 'object_moved' for e in res.events)


def test_queries(store):
    stack_scene(store)
    store.set_properties('bowl_1', {'located': 'blue box'})
    objs, _ = store.query_objects({'class': 'cup'}, 1 * S)
    assert [o.id for o in objs] == ['cup_1']
    objs, d = store.query_objects({'near': '0,0.3,0.75', 'radius': '0.2'}, 1 * S)
    assert [o.id for o in objs] == ['bowl_1'] and d[0] == pytest.approx(0.0)
    objs, _ = store.query_objects({'region': '-1,1,-1,1,0.8,1'}, 1 * S)
    assert [o.id for o in objs] == ['cup_1']
    assert [o.id for o in store.query_objects({'property.located': ''}, 1 * S)[0]] == ['bowl_1']
    assert store.query_objects({'property.located': 'red box'}, 1 * S)[0] == []
    free = {o.id for o in store.query_objects({'free_only': 'true'}, 1 * S)[0]}
    assert free == {'cup_1', 'bowl_1'}
    assert store.query_objects({'max_age': '1'}, 5 * S)[0] == []
    with pytest.raises(SceneError):
        store.query_objects({'colour': 'blue'}, 0)
    with pytest.raises(SceneError):
        store.query_objects({'near': '1,2'}, 0)
    rels = store.query_relations({'object_id': 'cup_1', 'type': 'on_top_of'})
    assert [(r.object_a, r.object_b) for r in rels] == [('cup_1', 'box_1')]


def test_mechanical_relations_and_blocking(store):
    store.update_scene(1 * S, [obj('box', (0, 0, 0.1), (0.3, 0.3, 0.2)),
                               obj('lid', (1, 0, 0.1)), obj('part', (2, 0, 0.1))],
                       [mech('#2', '#0', 'closed_by'), mech('#1', '#0', 'screwed_to')])
    blockers = store.composite_query('blockers_of', {'object_id': 'part_1'})
    assert blockers[0]['kind'] == 'blocking_object' and blockers[0]['object_id'] == 'box_1'
    steps = store.composite_query('disassembly_order', {'object_id': 'part_1'})
    assert [(s['action'], s['object_id']) for s in steps] == [
        ('remove', 'box_1'), ('remove', 'part_1')]
    steps = store.composite_query('disassembly_order', {'object_id': 'lid_1'})
    assert [s['action'] for s in steps] == ['release', 'remove']
    rid = steps[0]['relation_id']
    store.set_relation_state(rid, 'loosened')
    assert store.composite_query('blockers_of', {'object_id': 'lid_1'}) == []
    with pytest.raises(SceneError):
        store.set_relation_state(rid, 'open')
    json.dumps(steps)   # results are JSON-serializable


def test_relation_source_policy(store):
    store.update_scene(1 * S, [obj('a', (0, 0, 0)), obj('b', (1, 0, 0))])
    store.add_relation(mech('a_1', 'b_1', 'screwed_to', source='inferred', confidence=0.4,
                            stamp=1))
    r = store.relations()[0]
    assert not r.confirmed
    store.add_relation(mech('a_1', 'b_1', 'screwed_to', source='inferred', confidence=0.4,
                            stamp=1))
    assert not store.relations()[0].confirmed      # same acquisition: not a second view
    store.add_relation(mech('a_1', 'b_1', 'screwed_to', source='inferred', confidence=0.4,
                            stamp=2))
    assert store.relations()[0].confirmed
    store.add_relation(mech('a_1', 'b_1', 'screwed_to', state='tightened'))  # a priori
    res = store.add_relation(mech('a_1', 'b_1', 'screwed_to', state='loosened',
                                  source='inferred', stamp=3))
    r = store.relations()[0]
    assert r.source == 'a_priori' and r.state == 'tightened' and 'conflict' in res.message
    with pytest.raises(SceneError):
        store.add_relation(RelationRecord('a_1', 'b_1', 'on_top_of'))   # geometric
    with pytest.raises(SceneError):
        store.remove_relation(999)


def test_remove_object_cascades(store):
    stack_scene(store)
    store.update_scene(2 * S, [obj('box', (0, 0, 0.75), (0.2, 0.2, 0.1), oid='box_1',
                                   update_grasps=True,
                                   grasps=[GraspRecord((0, 0, 0.8), (0, 0, 0, 1), 0.9)])])
    store.set_properties('box_1', {'k': 'v'})
    res = store.remove_object('box_1')
    assert res.to_verify_ids == ['cup_1']
    assert store.get('cup_1').state == STATE_TO_VERIFY
    assert all('box_1' not in (r.object_a, r.object_b) for r in store.relations())
    assert store._con.execute('SELECT COUNT(*) FROM grasps').fetchone()[0] == 0
    assert store._con.execute('SELECT COUNT(*) FROM properties').fetchone()[0] == 0
    # ids are never reused
    res = store.update_scene(3 * S, [obj('box', (0.5, 0, 0.75))])
    assert res.object_ids == ['box_2']
    store.update_scene(4 * S, [obj('cup', (0, 0, 0.85), oid='cup_1')])
    assert store.get('cup_1').state == 'ok'          # observed again


def test_disassembly_cycle(store):
    store.update_scene(1 * S, [obj('a', (0, 0, 0)), obj('b', (5, 0, 0))],
                       [mech('#0', '#1', 'held_by'), mech('#1', '#0', 'held_by')])
    with pytest.raises(SceneError, match='Cycle'):
        store.composite_query('disassembly_order', {'object_id': 'a_1'})


def test_persistence_and_frame_check(tmp_path):
    path = str(tmp_path / 'scene.sqlite')
    s = SceneStore(path, 'world')
    stack_scene(s)
    s.set_properties('cup_1', {'owner': 'leo'})
    s.close()
    s = SceneStore(path, 'world')
    assert s.scene_version == 2 and s.get('cup_1').properties == {'owner': 'leo'}
    assert len(s.relations()) > 0
    assert s.update_scene(2 * S, [obj('cup', (1, 1, 1))]).object_ids == ['cup_2']
    s.clear()
    assert s.objects() == [] and s.relations() == []
    assert s.update_scene(3 * S, [obj('cup', (1, 1, 1))]).object_ids == ['cup_1']
    s.close()
    with pytest.raises(SceneError, match='frame'):
        SceneStore(path, 'map')


def test_support_thin_lid_and_bridge():
    cfg = GeometricConfig()
    box = Box((0, 0, 0.80), (0, 0, 0, 1), (0.30, 0.20, 0.10))
    lid = Box((0, 0, 0.86), (0, 0, 0, 1), (0.30, 0.20, 0.02))
    cup = Box((0, 0, 0.92), (0, 0, 0, 1), (0.08, 0.08, 0.10))
    rel = support_relations({'box': box, 'lid': lid, 'cup': cup}, cfg)
    assert rel == {('cup', 'on_top_of', 'lid'), ('lid', 'on_top_of', 'box')}
    left = Box((0, -0.1, 0.05), (0, 0, 0, 1), (0.1, 0.1, 0.1))
    right = Box((0, 0.1, 0.05), (0, 0, 0, 1), (0.1, 0.1, 0.1))
    plank = Box((0, 0, 0.11), (0, 0, 0, 1), (0.1, 0.3, 0.02))
    rel = support_relations({'l': left, 'r': right, 'p': plank},
                            GeometricConfig(on_top_min_overlap=0.2))
    assert rel == {('p', 'on_top_of', 'l'), ('p', 'on_top_of', 'r')}


def test_blocking_model_supported_by():
    rels = [RelationRecord('b', 'a', 'on_top_of'), RelationRecord('c', 'b', 'on_top_of')]
    assert BlockingModel(rels).supported_by('a') == ['b', 'c']
