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
Command line client for scene_db and scene_updater.

Examples::

    ros2 run scene_db cli acquire --grasps
    ros2 run scene_db cli list
    ros2 run scene_db cli query class=cup color=blue
    ros2 run scene_db cli query near=1.2,0,0.8 radius=0.5 max_results=1
    ros2 run scene_db cli query property.located="blue box"
    ros2 run scene_db cli relations object_id=cup_1
    ros2 run scene_db cli q free_objects
    ros2 run scene_db cli q disassembly_order object_id=part_1
    ros2 run scene_db cli set-props cup_1 located="blue box" owner=anna
    ros2 run scene_db cli remove-props cup_1 owner
    ros2 run scene_db cli add-relation lid_1 screwed_to box_1 --fastener screw
    ros2 run scene_db cli set-state 7 loosened
    ros2 run scene_db cli remove cup_2
"""

import argparse
import json
import sys
from typing import List

import rclpy
from rclpy.node import Node
from scene_msgs.msg import KeyValue, Relation
from scene_msgs.srv import (
    Acquire,
    AddRelation,
    Query,
    QueryObjects,
    QueryRelations,
    RemoveObject,
    RemoveProperties,
    RemoveRelation,
    SetProperties,
    SetRelationState,
)
from std_srvs.srv import Trigger


def parse_pairs(items: List[str]) -> List[KeyValue]:
    """Parse ``key=value`` arguments."""
    out = []
    for item in items:
        key, sep, value = item.partition('=')
        if not sep or not key:
            raise SystemExit(f"Expected key=value, got '{item}'")
        out.append(KeyValue(key=key, value=value))
    return out


def format_object(o, distance=None) -> str:
    """One-line description of a SceneObject."""
    p, s = o.pose.position, o.size
    text = (f'{o.id:12s} {o.color:7s} {o.class_name:12s} '
            f'({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) size ({s.x:.2f}, {s.y:.2f}, {s.z:.2f}) '
            f'conf {o.confidence:.2f} obs {o.observations} {o.state}')
    if o.grasps:
        text += f' grasps {len(o.grasps)} (best {o.grasps[0].grasp.score:.2f})'
    if distance is not None:
        text += f' dist {distance:.3f}'
    if o.properties:
        text += ' ' + ', '.join(f'{kv.key}={kv.value}' for kv in o.properties)
    return text


def format_relation(r) -> str:
    """One-line description of a Relation."""
    text = f'[{r.id:3d}] {r.object_a} {r.type} {r.object_b} ({r.family}, {r.source}'
    if r.family == Relation.FAMILY_MECHANICAL:
        text += f', {r.state}'
        if r.fastener:
            text += f', {r.fastener}'
        text += f', conf {r.confidence:.2f}' + ('' if r.confirmed else ', unconfirmed')
    return text + ')'


def build_parser() -> argparse.ArgumentParser:
    """Command line arguments."""
    parser = argparse.ArgumentParser(prog='ros2 run scene_db cli', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--db', default='/scene_db', help='scene_db node name')
    parser.add_argument('--updater', default='/scene_updater', help='scene_updater node name')
    parser.add_argument('--timeout', type=float, default=30.0, help='service timeout [s]')
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('acquire', help='run one acquisition (scene_updater)')
    p.add_argument('--grasps', action='store_true', help='also detect grasps')
    sub.add_parser('list', help='list all objects')
    p = sub.add_parser('query', help='query objects with key=value filters')
    p.add_argument('filters', nargs='*', metavar='KEY=VALUE')
    p = sub.add_parser('relations', help='query relations with key=value filters')
    p.add_argument('filters', nargs='*', metavar='KEY=VALUE')
    p = sub.add_parser('q', help='composite query (free_objects, blockers_of, ...)')
    p.add_argument('query_type')
    p.add_argument('params', nargs='*', metavar='KEY=VALUE')
    p = sub.add_parser('remove', help='remove an object')
    p.add_argument('id')
    p = sub.add_parser('set-props', help='add or overwrite properties')
    p.add_argument('id')
    p.add_argument('properties', nargs='+', metavar='KEY=VALUE')
    p = sub.add_parser('remove-props', help='remove properties')
    p.add_argument('id')
    p.add_argument('keys', nargs='+')
    p = sub.add_parser('add-relation', help='add a mechanical relation: A TYPE B')
    p.add_argument('object_a')
    p.add_argument('type')
    p.add_argument('object_b')
    p.add_argument('--state', default='tightened')
    p.add_argument('--source', default='a_priori', choices=['a_priori', 'inferred'])
    p.add_argument('--fastener', default='')
    p.add_argument('--confidence', type=float, default=1.0)
    p = sub.add_parser('remove-relation', help='remove a mechanical relation')
    p.add_argument('relation_id', type=int)
    p = sub.add_parser('set-state', help='set the state of a mechanical relation')
    p.add_argument('relation_id', type=int)
    p.add_argument('state', choices=['tightened', 'loosened', 'removed'])
    sub.add_parser('clear', help='remove everything')
    return parser


def call(node: Node, srv_type, name: str, request, timeout: float):
    """Call a service and wait for the response."""
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
        raise SystemExit(f'Service {name} not available')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise SystemExit(f'{name}: no response within {timeout} s')
    return future.result()


def run(args) -> int:
    """Execute the command; return the exit code."""
    node = Node('scene_db_cli')
    db, t = args.db.rstrip('/'), args.timeout

    def status(res) -> int:
        version = f' (scene version {res.scene_version})' if hasattr(res, 'scene_version') \
            else ''
        print(('OK' if res.success else 'ERROR') + f': {res.message}{version}')
        return 0 if res.success else 1

    c = args.command
    if c == 'acquire':
        res = call(node, Acquire, f'{args.updater.rstrip("/")}/acquire',
                   Acquire.Request(detect_grasps=args.grasps), t)
        code = status(res)
        for label, ids in (('added', res.added_ids), ('moved', res.moved_ids),
                           ('unchanged', res.unchanged_ids)):
            if ids:
                print(f'  {label}: {", ".join(ids)}')
        return code
    if c in ('list', 'query'):
        filters = parse_pairs(args.filters) if c == 'query' else []
        res = call(node, QueryObjects, f'{db}/query_objects',
                   QueryObjects.Request(filters=filters), t)
        code = status(res)
        dists = list(res.distances) or [None] * len(res.objects)
        for o, d in zip(res.objects, dists):
            print('  ' + format_object(o, d))
        return code
    if c == 'relations':
        res = call(node, QueryRelations, f'{db}/query_relations',
                   QueryRelations.Request(filters=parse_pairs(args.filters)), t)
        code = status(res)
        for r in res.relations:
            print('  ' + format_relation(r))
        return code
    if c == 'q':
        res = call(node, Query, f'{db}/query',
                   Query.Request(query_type=args.query_type, params=parse_pairs(args.params)), t)
        code = status(res)
        for item in res.results_json:
            print('  ' + json.dumps(json.loads(item)))
        return code
    if c == 'remove':
        res = call(node, RemoveObject, f'{db}/remove_object',
                   RemoveObject.Request(object_id=args.id), t)
        code = status(res)
        if res.to_verify_ids:
            print(f'  to verify: {", ".join(res.to_verify_ids)}')
        return code
    if c == 'set-props':
        return status(call(node, SetProperties, f'{db}/set_properties', SetProperties.Request(
            object_id=args.id, properties=parse_pairs(args.properties)), t))
    if c == 'remove-props':
        return status(call(node, RemoveProperties, f'{db}/remove_properties',
                           RemoveProperties.Request(object_id=args.id, keys=args.keys), t))
    if c == 'add-relation':
        rel = Relation(object_a=args.object_a, object_b=args.object_b, type=args.type,
                       family=Relation.FAMILY_MECHANICAL, state=args.state,
                       source=args.source, fastener=args.fastener,
                       confidence=args.confidence)
        res = call(node, AddRelation, f'{db}/add_relation', AddRelation.Request(relation=rel), t)
        code = status(res)
        if res.success:
            print(f'  relation id: {res.relation_id}')
        return code
    if c == 'remove-relation':
        return status(call(node, RemoveRelation, f'{db}/remove_relation',
                           RemoveRelation.Request(relation_id=args.relation_id), t))
    if c == 'set-state':
        return status(call(node, SetRelationState, f'{db}/set_relation_state',
                           SetRelationState.Request(relation_id=args.relation_id,
                                                    new_state=args.state), t))
    if c == 'clear':
        return status(call(node, Trigger, f'{db}/clear', Trigger.Request(), t))
    return 2


def main(argv=None) -> None:
    """Entry point."""
    args = build_parser().parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    rclpy.init(args=argv)
    try:
        code = run(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
