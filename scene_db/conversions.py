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

"""Conversions between store records and ROS messages."""

from typing import Dict, Sequence

from builtin_interfaces.msg import Time
from grasp_detection_msgs.msg import Grasp
from scene_db.relations import RelationRecord
from scene_db.store import Event, GraspRecord, NS_PER_S, ObjectInput, ObjectRecord
from scene_msgs.msg import KeyValue, Relation, SceneEvent, SceneGrasp, SceneObject
from std_msgs.msg import ColorRGBA, Header


def ns_to_time(ns: int) -> Time:
    """Nanoseconds to builtin_interfaces/Time."""
    return Time(sec=int(ns // NS_PER_S), nanosec=int(ns % NS_PER_S))


def time_to_ns(t: Time) -> int:
    """builtin_interfaces/Time to nanoseconds."""
    return int(t.sec) * NS_PER_S + int(t.nanosec)


def key_values(items: Sequence[KeyValue]) -> Dict[str, str]:
    """KeyValue[] to dict (later keys win)."""
    return {kv.key: kv.value for kv in items}


def grasp_to_record(g: Grasp, stamp_ns: int = 0) -> GraspRecord:
    """Convert a grasp_detection_msgs/Grasp to a GraspRecord."""
    p, q = g.pose.position, g.pose.orientation
    return GraspRecord((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), float(g.score),
                       float(g.width), float(g.height), float(g.depth), stamp_ns)


def record_to_grasp(r: GraspRecord) -> Grasp:
    """Convert a GraspRecord to a grasp_detection_msgs/Grasp."""
    g = Grasp(score=float(r.score), width=float(r.width), height=float(r.height),
              depth=float(r.depth))
    p, q = g.pose.position, g.pose.orientation
    p.x, p.y, p.z = (float(v) for v in r.position)
    q.x, q.y, q.z, q.w = (float(v) for v in r.quaternion)
    return g


def object_to_msg(o: ObjectRecord) -> SceneObject:
    """Convert an ObjectRecord to a scene_msgs/SceneObject."""
    m = SceneObject(id=o.id, class_name=o.class_name, color=o.color, state=o.state,
                    confidence=float(o.confidence), observations=int(o.observations))
    m.rgb = ColorRGBA(r=float(o.rgb[0]), g=float(o.rgb[1]), b=float(o.rgb[2]), a=1.0)
    p, q = m.pose.position, m.pose.orientation
    p.x, p.y, p.z = (float(v) for v in o.center)
    q.x, q.y, q.z, q.w = (float(v) for v in o.quaternion)
    m.size.x, m.size.y, m.size.z = (float(v) for v in o.size)
    m.first_seen, m.last_seen = ns_to_time(o.first_seen_ns), ns_to_time(o.last_seen_ns)
    m.grasps_stamp = ns_to_time(o.grasps_stamp_ns)
    m.grasps = [SceneGrasp(id=int(g.id), grasp=record_to_grasp(g)) for g in o.grasps]
    m.properties = [KeyValue(key=k, value=v) for k, v in sorted(o.properties.items())]
    return m


def update_to_input(u) -> ObjectInput:
    """Convert a scene_msgs/ObjectUpdate to an ObjectInput."""
    det = u.detection
    hyp = det.results[0].hypothesis if det.results else None
    c = det.bbox.center
    return ObjectInput(
        id=det.id, class_name=hyp.class_id if hyp else '', score=float(hyp.score) if hyp else 0.0,
        center=(c.position.x, c.position.y, c.position.z),
        quaternion=(c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w),
        size=(det.bbox.size.x, det.bbox.size.y, det.bbox.size.z),
        color=u.color, rgb=(u.rgb.r, u.rgb.g, u.rgb.b), moved=u.moved,
        update_grasps=u.update_grasps, grasps=[grasp_to_record(g) for g in u.grasps])


def relation_to_msg(r: RelationRecord) -> Relation:
    """Convert a RelationRecord to a scene_msgs/Relation."""
    return Relation(id=int(r.id), object_a=r.object_a, object_b=r.object_b, type=r.type,
                    family=r.family, fastener=r.fastener, state=r.state, source=r.source,
                    confidence=float(r.confidence), confirmed=bool(r.confirmed),
                    observations=int(r.observations), stamp=ns_to_time(r.stamp_ns))


def msg_to_relation(m: Relation) -> RelationRecord:
    """Convert a scene_msgs/Relation to a RelationRecord (defaults for empty fields)."""
    return RelationRecord(object_a=m.object_a, object_b=m.object_b, type=m.type,
                          family=m.family or 'mechanical', state=m.state or 'tightened',
                          source=m.source or 'a_priori', fastener=m.fastener,
                          confidence=float(m.confidence) if m.confidence > 0 else 1.0,
                          stamp_ns=time_to_ns(m.stamp))


def event_to_msg(e: Event, frame_id: str, stamp: Time, version: int) -> SceneEvent:
    """Store event to scene_msgs/SceneEvent."""
    return SceneEvent(header=Header(stamp=stamp, frame_id=frame_id), event=e.kind,
                      object_id=e.object_id, relation_id=int(e.relation_id or 0),
                      details=e.details, scene_version=int(version))
