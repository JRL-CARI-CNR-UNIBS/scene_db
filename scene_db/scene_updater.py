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
Scene updater: one acquisition on request, written to scene_db atomically.

Services:
    ``~/acquire``         scene_msgs/Acquire
    ``~/capture_view``    scene_msgs/CaptureView
    ``~/acquire_grasps``  scene_msgs/AcquireGrasps

Flow of one acquisition:

1. take the first synchronized color + aligned depth + camera_info received
   after the request (stamp ``t``);
2. call the object detector (object_detection_msgs/DetectObjects, boxes in
   the fixed frame) and, if requested, the grasp detector
   (grasp_detection_msgs/DetectGrasps) **in parallel** on the same images;
3. transform the grasps to the fixed frame (TF at ``t``) and assign each one
   to the object box that contains it (expanded by ``grasp_margin``);
   objects of ``no_grasp_classes`` (supports such as the table) never get
   grasps; unassigned grasps are discarded;
4. associate the detections to the stored objects (same class, Hungarian on
   distance); unmatched detections are new objects;
5. send everything to scene_db in a single ``UpdateScene`` call.

Objects that are not detected are kept: removal happens only through
``scene_db/remove_object``.

Grasps of one object from close views (two-step use)::

    acquire (far views, objects only)  ->  query_objects id=<obj> (pose, size)
    -> the orchestrator moves the camera near the object, capture_view per view
    -> acquire_grasps <obj>

``~/capture_view`` stores the next synchronized frame with the camera pose
(tf in the fixed frame at the image stamp). ``~/acquire_grasps`` sends the
stored views to a MultiviewDetectGrasps server with a volume around the
object box (see ``association.grasp_volume``), keeps the grasps that fall in
the object box (expanded by ``object_grasp_margin``; grasps inside other
boxes belong to those objects and are dropped) and replaces the object grasps
with ``scene_db/set_grasps``. Pose and statistics of the object are unchanged.

Subscribed topics (remap them to your camera):
    ``color/image_raw``, ``depth/image_raw`` (aligned), ``color/camera_info``
"""

import threading
import time
from typing import Optional

from geometry_msgs.msg import Point, Pose
from grasp_detection_msgs.srv import DetectGrasps, MultiviewDetectGrasps
import message_filters
from object_detection_msgs.srv import DetectObjects
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scene_db.association import (
    associate,
    AssociationConfig,
    Candidate,
    grasp_volume,
    match_grasps,
    transform_pose,
)
from scene_db.relations import Box
from scene_msgs.msg import KeyValue, ObjectUpdate
from scene_msgs.srv import (
    Acquire,
    AcquireGrasps,
    CaptureView,
    QueryObjects,
    SetGrasps,
    UpdateScene,
)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class AcquireError(RuntimeError):
    """A step of the acquisition failed."""


class SceneUpdater(Node):
    """Run acquisitions and write them to scene_db."""

    def __init__(self) -> None:
        super().__init__('scene_updater')
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'fixed_frame': 'world',
            'detect_service': '/yolo_rgbd_detector/detect',
            'grasp_service': '/mock_grasp_server/detect_grasps',
            'scene_db': '/scene_db',
            'timeout': 10.0,
            'sync_slop': 0.05,
            'min_score': 0.0,
            'classes': [''],
            'grasp_margin': 0.015,
            'no_grasp_classes': [''],
            'multiview_grasp_service': '/vgn_grasp_server/multiview_detect_grasps',
            'grasp_volume_margin': 0.05,
            'grasp_volume_below': 0.05,
            'grasp_volume_height': 0.30,
            'object_grasp_margin': 0.03,
            'max_views': 20,
            'max_grasps_per_object': 10,
            'min_grasp_score': 0.0,
            'association_distance': AssociationConfig.distance,
            'association_size_factor': AssociationConfig.size_factor,
            'move_threshold': AssociationConfig.move_threshold,
            'move_size_factor': AssociationConfig.move_size_factor,
            'match_color': AssociationConfig.match_color,
        }.items()}
        self._p = p
        self._assoc = AssociationConfig(p['association_distance'],
                                        p['association_size_factor'], p['move_threshold'],
                                        p['move_size_factor'], p['match_color'])

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        clients = ReentrantCallbackGroup()
        self._detect = self.create_client(DetectObjects, p['detect_service'],
                                          callback_group=clients)
        self._grasps = self.create_client(DetectGrasps, p['grasp_service'],
                                          callback_group=clients)
        db = p['scene_db'].rstrip('/')
        self._query = self.create_client(QueryObjects, f'{db}/query_objects',
                                         callback_group=clients)
        self._update = self.create_client(UpdateScene, f'{db}/update_scene',
                                          callback_group=clients)
        self._mv_grasps = self.create_client(MultiviewDetectGrasps,
                                             p['multiview_grasp_service'],
                                             callback_group=clients)
        self._set_grasps = self.create_client(SetGrasps, f'{db}/set_grasps',
                                              callback_group=clients)
        self._views = []   # (color, depth, camera_info, camera Pose) for acquire_grasps

        # Images: keep only the frames that arrive while an acquisition waits.
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._frame = None
        self._waiting_since: Optional[Time] = None
        subs = [message_filters.Subscriber(self, Image, 'color/image_raw',
                                           qos_profile=qos_profile_sensor_data),
                message_filters.Subscriber(self, Image, 'depth/image_raw',
                                           qos_profile=qos_profile_sensor_data),
                message_filters.Subscriber(self, CameraInfo, 'color/camera_info',
                                           qos_profile=qos_profile_sensor_data)]
        self._sync = message_filters.ApproximateTimeSynchronizer(subs, 10, p['sync_slop'])
        self._sync.registerCallback(self._on_frame)

        # One acquisition at a time: they share the frame buffer and the views.
        acquisitions = MutuallyExclusiveCallbackGroup()
        self.create_service(Acquire, '~/acquire', self._srv_acquire,
                            callback_group=acquisitions)
        self.create_service(CaptureView, '~/capture_view', self._srv_capture_view,
                            callback_group=acquisitions)
        self.create_service(AcquireGrasps, '~/acquire_grasps', self._srv_acquire_grasps,
                            callback_group=acquisitions)
        self.get_logger().info(
            f'Ready: ~/acquire, ~/capture_view, ~/acquire_grasps (detector '
            f"{p['detect_service']}, grasps {p['grasp_service']}, multi-view grasps "
            f"{p['multiview_grasp_service']}, database {db}, frame '{p['fixed_frame']}')")

    # ------------------------------------------------------------------------
    # Images
    # ------------------------------------------------------------------------
    def _on_frame(self, color: Image, depth: Image, info: CameraInfo) -> None:
        with self._frame_lock:
            if self._waiting_since is None or self._frame is not None:
                return
            if Time.from_msg(color.header.stamp) < self._waiting_since:
                return
            self._frame = (color, depth, info)
            self._frame_event.set()

    def _wait_frame(self, timeout: float):
        with self._frame_lock:
            self._frame, self._waiting_since = None, self.get_clock().now()
            self._frame_event.clear()
        ok = self._frame_event.wait(timeout)
        with self._frame_lock:
            frame, self._frame, self._waiting_since = self._frame, None, None
        if not ok or frame is None:
            raise AcquireError(
                f'No synchronized color/depth/camera_info within {timeout:.1f} s '
                f"(topics: {self.resolve_topic_name('color/image_raw')}, "
                f"{self.resolve_topic_name('depth/image_raw')}, "
                f"{self.resolve_topic_name('color/camera_info')})")
        return frame

    # ------------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------------
    def _start(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise AcquireError(f'Service {client.srv_name} not available')
        return client.call_async(request)

    @staticmethod
    def _wait(future, name: str, deadline: float):
        while not future.done():
            if time.monotonic() > deadline:
                raise AcquireError(f'{name}: timeout')
            time.sleep(0.005)
        result = future.result()
        if result is None:
            raise AcquireError(f'{name}: call failed')
        if hasattr(result, 'success') and not result.success:
            raise AcquireError(f'{name}: {result.message}')
        return result

    def _call(self, client, request, timeout: float):
        return self._wait(self._start(client, request, timeout), client.srv_name,
                          time.monotonic() + timeout)

    # ------------------------------------------------------------------------
    # Acquisition
    # ------------------------------------------------------------------------
    def _srv_acquire(self, req, res):
        timeout = req.timeout if req.timeout > 0.0 else self._p['timeout']
        t0 = time.monotonic()
        try:
            self._acquire(req.detect_grasps, timeout, res)
        except (AcquireError, TransformException) as err:
            res.success, res.message = False, str(err)
            self.get_logger().warn(f'acquire: {err}')
            return res
        res.success = True
        res.message = (f'{res.num_detections} detections: {len(res.added_ids)} added, '
                       f'{len(res.moved_ids)} moved, {len(res.unchanged_ids)} unchanged; '
                       f'grasps {res.num_grasps_assigned}/{res.num_grasps} assigned '
                       f'({time.monotonic() - t0:.2f} s)')
        self.get_logger().info(res.message)
        return res

    def _acquire(self, detect_grasps: bool, timeout: float, res) -> None:
        p = self._p
        color, depth, info = self._wait_frame(timeout)
        stamp = color.header.stamp

        # 2. Detection and grasps in parallel on the same images.
        det_req = DetectObjects.Request(color_images=[color], depth_images=[depth],
                                        camera_infos=[info], target_frame=p['fixed_frame'],
                                        min_score=float(p['min_score']),
                                        classes=[c for c in p['classes'] if c])
        deadline = time.monotonic() + timeout
        det_future = self._start(self._detect, det_req, timeout)
        grasp_future = None
        if detect_grasps:
            grasp_future = self._start(self._grasps, DetectGrasps.Request(
                color=color, depth=depth, camera_info=info,
                min_score=float(p['min_grasp_score'])), timeout)
        # The stored objects, while the detectors run.
        stored = self._call(self._query, QueryObjects.Request(
            filters=[KeyValue(key='include_grasps', value='false')]), timeout).objects

        detections = self._wait(det_future, 'object detection', deadline)
        objects = [o for o in detections.objects if o.has_3d]
        res.num_detections = len(objects)
        grasps = []
        if grasp_future is not None:
            g = self._wait(grasp_future, 'grasp detection', deadline)
            grasps = self._grasps_to_fixed(g.header, g.grasps)
        res.num_grasps = len(grasps)

        # 3. Grasps to objects.
        boxes = [self._box(o.detection_3d) for o in objects]
        no_grasp = {c for c in p['no_grasp_classes'] if c}
        owners = match_grasps([pos for _, pos in grasps], boxes, p['grasp_margin'],
                              eligible=[o.class_name not in no_grasp for o in objects])
        per_object = [[] for _ in objects]
        for (msg, _), owner in zip(grasps, owners):
            if owner >= 0:
                per_object[owner].append(msg)
        for lst in per_object:
            lst.sort(key=lambda m: -m.score)
            del lst[p['max_grasps_per_object']:]
        res.num_grasps_assigned = sum(len(lst) for lst in per_object)

        # 4. Association.
        det_c = [Candidate(o.class_name, boxes[i].center, boxes[i].size, o.color)
                 for i, o in enumerate(objects)]
        obj_c = [Candidate(o.class_name, (o.pose.position.x, o.pose.position.y,
                                          o.pose.position.z),
                           (o.size.x, o.size.y, o.size.z), o.color) for o in stored]
        matches = associate(det_c, obj_c, self._assoc)

        # 5. One atomic update.
        updates = []
        for i, (o, (j, moved)) in enumerate(zip(objects, matches)):
            u = ObjectUpdate(detection=o.detection_3d, color=o.color, rgb=o.rgb)
            u.detection.id = stored[j].id if j >= 0 else ''
            u.moved = moved
            if detect_grasps:
                u.update_grasps, u.grasps = True, per_object[i]
            elif j < 0 or moved:
                u.update_grasps = True   # new or moved: no valid grasps
            updates.append(u)
        out = self._call(self._update, UpdateScene.Request(
            stamp=stamp, frame_id=p['fixed_frame'], objects=updates), timeout)

        res.scene_version = out.scene_version
        for oid, (j, moved) in zip(out.object_ids, matches):
            (res.added_ids if j < 0 else res.moved_ids if moved else res.unchanged_ids
             ).append(oid)

    # ------------------------------------------------------------------------
    # Grasps of one object from stored views
    # ------------------------------------------------------------------------
    def _srv_capture_view(self, req, res):
        timeout = req.timeout if req.timeout > 0.0 else self._p['timeout']
        try:
            if req.clear or req.clear_only:
                self._views.clear()
            if not req.clear_only:
                if len(self._views) >= self._p['max_views']:
                    raise AcquireError(f'{len(self._views)} views stored (max_views): '
                                       'call acquire_grasps or clear them')
                color, depth, info = self._wait_frame(timeout)
                frame = depth.header.frame_id or color.header.frame_id
                tf = self._tf_buffer.lookup_transform(
                    self._p['fixed_frame'], frame, Time.from_msg(depth.header.stamp),
                    timeout=Duration(seconds=min(timeout, 1.0)))
                pose = Pose()
                t = tf.transform.translation
                pose.position = Point(x=t.x, y=t.y, z=t.z)
                pose.orientation = tf.transform.rotation
                self._views.append((color, depth, info, pose))
                res.camera_pose = pose
        except (AcquireError, TransformException) as err:
            res.success, res.message = False, str(err)
            res.num_views = len(self._views)
            self.get_logger().warn(f'capture_view: {err}')
            return res
        res.success, res.num_views = True, len(self._views)
        res.message = ('views cleared' if req.clear_only
                       else f'{len(self._views)} views stored')
        self.get_logger().info(f'capture_view: {res.message}')
        return res

    def _srv_acquire_grasps(self, req, res):
        timeout = req.timeout if req.timeout > 0.0 else self._p['timeout']
        t0 = time.monotonic()
        try:
            self._acquire_grasps(req, timeout, res)
        except (AcquireError, TransformException) as err:
            res.success, res.message = False, str(err)
            self.get_logger().warn(f'acquire_grasps: {err}')
            return res
        res.success = True
        res.message = (f"'{req.object_id}': {res.num_grasps_assigned}/{res.num_grasps} "
                       f'grasps from {res.num_views} views ({time.monotonic() - t0:.2f} s)')
        self.get_logger().info(f'acquire_grasps: {res.message}')
        return res

    def _acquire_grasps(self, req, timeout: float, res) -> None:
        p = self._p
        views = list(self._views)
        if not views:
            raise AcquireError('no stored views: call capture_view first')
        no_grasp = {c for c in p['no_grasp_classes'] if c}
        objs = self._call(self._query, QueryObjects.Request(
            filters=[KeyValue(key='include_grasps', value='false')]), timeout).objects
        index = next((i for i, o in enumerate(objs) if o.id == req.object_id), None)
        if index is None:
            raise AcquireError(f"unknown object '{req.object_id}'")
        target = objs[index]
        if target.class_name in no_grasp:
            raise AcquireError(f"'{target.id}' is a {target.class_name} (no_grasp_classes)")

        # Grasp detection in a volume around the object box.
        boxes = [self._object_box(o) for o in objs]
        margin = req.margin if req.margin > 0.0 else p['grasp_volume_margin']
        lo, hi = grasp_volume(boxes[index], margin, p['grasp_volume_below'],
                              p['grasp_volume_height'])
        res.workspace_min = Point(x=lo[0], y=lo[1], z=lo[2])
        res.workspace_max = Point(x=hi[0], y=hi[1], z=hi[2])
        mreq = MultiviewDetectGrasps.Request(
            target_frame=p['fixed_frame'], use_workspace=True,
            workspace_min=res.workspace_min, workspace_max=res.workspace_max,
            min_score=float(p['min_grasp_score']))
        for color, depth, info, pose in views:
            mreq.color.append(color)
            mreq.depth.append(depth)
            mreq.camera_info.append(info)
            mreq.camera_poses.append(pose)
        out = self._call(self._mv_grasps, mreq, timeout)
        grasps = self._grasps_to_fixed(out.header, out.grasps)
        res.num_views, res.num_grasps = len(views), len(grasps)

        # Keep the grasps of the object; those inside other boxes are theirs.
        owners = match_grasps([pos for _, pos in grasps], boxes, p['object_grasp_margin'],
                              eligible=[o.class_name not in no_grasp for o in objs])
        mine = sorted((g for (g, _), o in zip(grasps, owners) if o == index),
                      key=lambda g: -g.score)[:p['max_grasps_per_object']]

        # The object must not have changed while the views were taken.
        now = self._call(self._query, QueryObjects.Request(filters=[
            KeyValue(key='id', value=target.id),
            KeyValue(key='include_grasps', value='false')]), timeout).objects
        if not now or self._object_box(now[0]) != boxes[index]:
            raise AcquireError(f"'{target.id}' changed during the grasp acquisition")
        stamp = max((d.header.stamp for _, d, _, _ in views),
                    key=lambda s: s.sec * 1_000_000_000 + s.nanosec)
        written = self._call(self._set_grasps, SetGrasps.Request(
            object_id=target.id, stamp=stamp, grasps=mine), timeout)
        res.scene_version = written.scene_version
        res.num_grasps_assigned = len(mine)
        if not req.keep_views:
            del self._views[:len(views)]

    @staticmethod
    def _object_box(o) -> Box:
        c, q, s = o.pose.position, o.pose.orientation, o.size
        return Box((c.x, c.y, c.z), (q.x, q.y, q.z, q.w), (s.x, s.y, s.z))

    @staticmethod
    def _box(det) -> Box:
        c, s = det.bbox.center, det.bbox.size
        return Box((c.position.x, c.position.y, c.position.z),
                   (c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w),
                   (s.x, s.y, s.z))

    def _grasps_to_fixed(self, header, grasps):
        """Transform grasps in place to the fixed frame; return (grasp, position) pairs."""
        frame = self._p['fixed_frame']
        if header.frame_id and header.frame_id != frame:
            tf = self._tf_buffer.lookup_transform(
                frame, header.frame_id, Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.5))
            t, q = tf.transform.translation, tf.transform.rotation
            trans, rot = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        else:
            trans, rot = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

        out = []
        for g in grasps:
            p, q = g.pose.position, g.pose.orientation
            pos, quat = transform_pose(trans, rot, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            g.pose.position.x, g.pose.position.y, g.pose.position.z = pos
            (g.pose.orientation.x, g.pose.orientation.y,
             g.pose.orientation.z, g.pose.orientation.w) = quat
            out.append((g, pos))
        return out


def main(args=None) -> None:
    """Run the scene updater."""
    rclpy.init(args=args)
    node = None
    try:
        node = SceneUpdater()
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
