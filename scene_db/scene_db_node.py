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
ROS 2 front end of the SQLite scene database.

The database (``scene_db.store``) is the single source of truth; this node
only exposes it. It does not subscribe to perception: the ``scene_updater``
node (or any client) writes acquisitions with ``~/update_scene``.

Published topics:
    ``~/objects``  scene_msgs/SceneObjectArray     full content after every change (latched)
    ``~/events``   scene_msgs/SceneEvent           one message per change
    ``~/markers``  visualization_msgs/MarkerArray  boxes, labels, grasps (latched)

Services (all responses carry ``scene_version``):
    ``~/update_scene``        scene_msgs/UpdateScene
    ``~/query_objects``       scene_msgs/QueryObjects
    ``~/query_relations``     scene_msgs/QueryRelations
    ``~/query``               scene_msgs/Query          composite queries, JSON results
    ``~/remove_object``       scene_msgs/RemoveObject
    ``~/set_properties``      scene_msgs/SetProperties
    ``~/remove_properties``   scene_msgs/RemoveProperties
    ``~/add_relation``        scene_msgs/AddRelation
    ``~/remove_relation``     scene_msgs/RemoveRelation
    ``~/set_relation_state``  scene_msgs/SetRelationState
    ``~/clear``               std_srvs/Trigger

See ``docs/API.md`` for details.
"""

import json
import os

from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scene_db.conversions import (
    event_to_msg,
    grasp_to_record,
    key_values,
    msg_to_relation,
    object_to_msg,
    relation_to_msg,
    time_to_ns,
    update_to_input,
)
from scene_db.relations import GeometricConfig, SceneError
from scene_db.store import SceneStore, StoreConfig, WriteResult
from scene_msgs.msg import SceneEvent, SceneObjectArray
from scene_msgs.srv import (
    AddRelation,
    Query,
    QueryObjects,
    QueryRelations,
    RemoveObject,
    RemoveProperties,
    RemoveRelation,
    SetGrasps,
    SetProperties,
    SetRelationState,
    UpdateScene,
)
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class SceneDbNode(Node):
    """Expose the scene store through ROS services and topics."""

    def __init__(self) -> None:
        super().__init__('scene_db')
        p = self._declare_parameters()
        self._frame = p['fixed_frame']
        self._publish_markers = p['publish_markers']

        path = p['database_file']
        if path != ':memory:':
            path = os.path.expanduser(path)
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            if p['reset_on_start'] and os.path.exists(path):
                for suffix in ('', '-wal', '-shm'):
                    if os.path.exists(path + suffix):
                        os.remove(path + suffix)
        config = StoreConfig(
            geometric=GeometricConfig(
                on_top_tolerance=p['on_top_tolerance'],
                on_top_max_gap=p['on_top_max_gap'],
                on_top_min_overlap=p['on_top_min_overlap'],
                inside_tolerance=p['inside_tolerance'],
                near_distance=p['near_distance'],
                lateral_tolerance=p['lateral_tolerance'],
                left_axis=tuple(p['left_axis'])),
            inferred_min_confidence=p['inferred_min_confidence'],
            blocking_types=tuple(p['blocking_types']))
        self._store = SceneStore(path, self._frame, config)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pub_objects = self.create_publisher(SceneObjectArray, '~/objects', latched)
        self._pub_events = self.create_publisher(SceneEvent, '~/events', 100)
        self._pub_markers = (self.create_publisher(MarkerArray, '~/markers', latched)
                             if self._publish_markers else None)

        # The store serializes the calls itself: services can run in parallel.
        for srv_type, name, cb in (
                (UpdateScene, '~/update_scene', self._srv_update_scene),
                (QueryObjects, '~/query_objects', self._srv_query_objects),
                (QueryRelations, '~/query_relations', self._srv_query_relations),
                (Query, '~/query', self._srv_query),
                (RemoveObject, '~/remove_object', self._srv_remove_object),
                (SetProperties, '~/set_properties', self._srv_set_properties),
                (SetGrasps, '~/set_grasps', self._srv_set_grasps),
                (RemoveProperties, '~/remove_properties', self._srv_remove_properties),
                (AddRelation, '~/add_relation', self._srv_add_relation),
                (RemoveRelation, '~/remove_relation', self._srv_remove_relation),
                (SetRelationState, '~/set_relation_state', self._srv_set_relation_state),
                (Trigger, '~/clear', self._srv_clear)):
            self.create_service(srv_type, name, cb)

        self._publish_state()
        self.get_logger().info(
            f"Scene database '{path}' (frame '{self._frame}'): "
            f'{len(self._store.objects(False))} objects, version {self._store.scene_version}')

    def _declare_parameters(self) -> dict:
        defaults = {
            'fixed_frame': 'world',
            'database_file': '~/.ros/scene_db.sqlite',
            'reset_on_start': False,
            'on_top_tolerance': GeometricConfig.on_top_tolerance,
            'on_top_max_gap': GeometricConfig.on_top_max_gap,
            'on_top_min_overlap': GeometricConfig.on_top_min_overlap,
            'inside_tolerance': GeometricConfig.inside_tolerance,
            'near_distance': GeometricConfig.near_distance,
            'lateral_tolerance': GeometricConfig.lateral_tolerance,
            'left_axis': list(GeometricConfig.left_axis),
            'inferred_min_confidence': StoreConfig.inferred_min_confidence,
            'blocking_types': ['closed_by', 'held_by'],
            'publish_markers': True,
        }
        return {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

    # ------------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------------
    def _run_write(self, res, fn):
        """Run a store write, fill success/message/scene_version and publish."""
        try:
            out: WriteResult = fn()
        except SceneError as err:
            res.success, res.message = False, str(err)
            res.scene_version = self._store.scene_version
            self.get_logger().warn(str(err))
            return None
        res.success, res.message, res.scene_version = True, out.message, out.scene_version
        self._after_write(out)
        return out

    def _run_read(self, res, fn):
        try:
            fn()
            res.success = True
        except SceneError as err:
            res.success, res.message = False, str(err)
        res.scene_version = self._store.scene_version
        return res

    def _after_write(self, out: WriteResult) -> None:
        stamp = self.get_clock().now().to_msg()
        for e in out.events:
            self._pub_events.publish(event_to_msg(e, self._frame, stamp, out.scene_version))
            self.get_logger().info(f'[v{out.scene_version}] {e.kind} {e.object_id} {e.details}')
        self._publish_state()

    def _publish_state(self) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame)
        objects = self._store.objects()
        self._pub_objects.publish(SceneObjectArray(
            header=header, scene_version=self._store.scene_version,
            objects=[object_to_msg(o) for o in objects]))
        if self._pub_markers is not None:
            self._pub_markers.publish(self._make_markers(header, objects))

    @staticmethod
    def _make_markers(header: Header, objects) -> MarkerArray:
        out = MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)])
        for i, o in enumerate(objects):
            rgb = o.rgb if any(o.rgb) else (0.6, 0.6, 0.6)
            alpha = 0.5 if o.state == 'ok' else 0.2
            cube = Marker(header=header, ns='objects', id=i, type=Marker.CUBE,
                          action=Marker.ADD)
            cube.pose.position = Point(x=o.center[0], y=o.center[1], z=o.center[2])
            (cube.pose.orientation.x, cube.pose.orientation.y,
             cube.pose.orientation.z, cube.pose.orientation.w) = o.quaternion
            cube.scale.x, cube.scale.y, cube.scale.z = (max(v, 0.01) for v in o.size)
            cube.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=alpha)
            text = Marker(header=header, ns='labels', id=i, type=Marker.TEXT_VIEW_FACING,
                          action=Marker.ADD)
            text.pose.position = Point(x=o.center[0], y=o.center[1],
                                       z=o.center[2] + 0.5 * o.size[2] + 0.06)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.05
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = o.id + ('' if o.state == 'ok' else ' (?)')
            out.markers += [cube, text]
            for j, g in enumerate(o.grasps[:5]):
                arrow = Marker(header=header, ns=f'grasps/{o.id}', id=j, type=Marker.ARROW,
                               action=Marker.ADD)
                arrow.pose.position = Point(x=g.position[0], y=g.position[1],
                                            z=g.position[2])
                (arrow.pose.orientation.x, arrow.pose.orientation.y,
                 arrow.pose.orientation.z, arrow.pose.orientation.w) = g.quaternion
                # Arrow from the grasp center along the grasp x axis (approach).
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.008, 0.008
                arrow.color = ColorRGBA(r=1.0 - g.score, g=g.score, b=0.0, a=0.9)
                out.markers.append(arrow)
        return out

    # ------------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------------
    def _srv_update_scene(self, req, res):
        if req.frame_id and req.frame_id != self._frame:
            res.success = False
            res.message = (f"Objects are in frame '{req.frame_id}', the database uses "
                           f"'{self._frame}'")
            res.scene_version = self._store.scene_version
            return res
        stamp = time_to_ns(req.stamp) or self.get_clock().now().nanoseconds
        try:
            inputs = [update_to_input(u) for u in req.objects]
            relations = [msg_to_relation(r) for r in req.relations]
        except (IndexError, ValueError) as err:
            res.success, res.message = False, f'Malformed request: {err}'
            return res
        out = self._run_write(res, lambda: self._store.update_scene(stamp, inputs, relations))
        if out is not None:
            res.object_ids, res.changed_object_ids = out.object_ids, out.changed_object_ids
            res.message = res.message or (
                f'{len(inputs)} objects ({len(out.changed_object_ids)} added or moved)')
        return res

    def _srv_set_grasps(self, req, res):
        stamp = time_to_ns(req.stamp) or self.get_clock().now().nanoseconds
        grasps = [grasp_to_record(g) for g in req.grasps]
        self._run_write(res, lambda: self._store.set_grasps(req.object_id, grasps, stamp))
        return res

    def _srv_query_objects(self, req, res):
        def run():
            objs, dist = self._store.query_objects(
                key_values(req.filters), self.get_clock().now().nanoseconds)
            res.objects = [object_to_msg(o) for o in objs]
            res.distances = [float(d) for d in dist]
            res.message = f'{len(objs)} objects'
        return self._run_read(res, run)

    def _srv_query_relations(self, req, res):
        def run():
            rels = self._store.query_relations(key_values(req.filters))
            res.relations = [relation_to_msg(r) for r in rels]
            res.message = f'{len(rels)} relations'
        return self._run_read(res, run)

    def _srv_query(self, req, res):
        def run():
            results = self._store.composite_query(req.query_type, key_values(req.params))
            res.results_json = [json.dumps(r) for r in results]
            res.message = f'{len(results)} results'
        return self._run_read(res, run)

    def _srv_remove_object(self, req, res):
        out = self._run_write(res, lambda: self._store.remove_object(req.object_id))
        if out is not None:
            res.to_verify_ids = out.to_verify_ids
        return res

    def _srv_set_properties(self, req, res):
        self._run_write(res, lambda: self._store.set_properties(
            req.object_id, key_values(req.properties)))
        return res

    def _srv_remove_properties(self, req, res):
        self._run_write(res, lambda: self._store.remove_properties(req.object_id,
                                                                   list(req.keys)))
        return res

    def _srv_add_relation(self, req, res):
        out = self._run_write(res, lambda: self._store.add_relation(
            msg_to_relation(req.relation)))
        if out is not None:
            res.relation_id = int(out.relation_id)
        return res

    def _srv_remove_relation(self, req, res):
        self._run_write(res, lambda: self._store.remove_relation(int(req.relation_id)))
        return res

    def _srv_set_relation_state(self, req, res):
        self._run_write(res, lambda: self._store.set_relation_state(int(req.relation_id),
                                                                    req.new_state))
        return res

    def _srv_clear(self, req, res):
        out = self._store.clear()
        self._after_write(out)
        res.success, res.message = True, f'{out.message} (version {out.scene_version})'
        return res

    def destroy_node(self) -> None:
        """Close the database before destroying the node."""
        self._store.close()
        super().destroy_node()


def main(args=None) -> None:
    """Run the scene database node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SceneDbNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
