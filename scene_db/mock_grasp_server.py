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
Geometric stand-in for AnyGrasp, with the same service interface.

Service:
    ``~/detect_grasps``  grasp_detection_msgs/DetectGrasps

Plans one top-down grasp per object resting on the dominant plane (see
``scene_db.grasp_planner``). Useful in simulation and for testing the
pipeline without a GPU; replace it with the real AnyGrasp server by
changing the ``grasp_service`` parameter of ``scene_updater``.
"""

from cv_bridge import CvBridge
from grasp_detection_msgs.msg import Grasp
from grasp_detection_msgs.srv import DetectGrasps
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from scene_db.grasp_planner import plan_grasps


class MockGraspServer(Node):
    """Serve DetectGrasps with the geometric planner."""

    def __init__(self) -> None:
        super().__init__('mock_grasp_server')
        self._min_height = self.declare_parameter('min_height', 0.01).value
        self._max_width = self.declare_parameter('max_width', 0.1).value
        self._min_points = self.declare_parameter('min_points', 100).value
        self._max_depth = self.declare_parameter('max_depth', 3.0).value
        self._bridge = CvBridge()
        self.create_service(DetectGrasps, '~/detect_grasps', self._srv_detect)
        self.get_logger().info('Ready: service ~/detect_grasps (geometric mock of AnyGrasp)')

    def _srv_detect(self, req, res):
        res.header = req.color.header
        k = req.camera_info.k
        if k[0] <= 0.0 or k[4] <= 0.0:
            res.success, res.message = False, 'camera_info has zero focal length'
            return res
        try:
            raw = self._bridge.imgmsg_to_cv2(req.depth, desired_encoding='passthrough')
        except Exception as err:  # cv_bridge raises its own error types
            res.success, res.message = False, f'Bad depth image: {err}'
            return res
        if req.depth.encoding in ('16UC1', 'mono16'):
            depth = raw.astype(np.float32) * 0.001
        elif req.depth.encoding == '32FC1':
            depth = raw.astype(np.float32)
        else:
            res.success, res.message = False, f'Unsupported depth encoding {req.depth.encoding}'
            return res
        depth[(depth <= 0.0) | (depth > self._max_depth) | ~np.isfinite(depth)] = np.nan

        h, w = depth.shape
        v, u = np.mgrid[0:h, 0:w]
        pts = np.dstack([(u - k[2]) * depth / k[0], (v - k[5]) * depth / k[4], depth])
        planned = plan_grasps(pts, self._min_height, self._max_width, self._min_points)

        lo, hi = req.workspace_min, req.workspace_max
        for g in planned:
            x, y, z = g.position
            if req.use_workspace and not (lo.x <= x <= hi.x and lo.y <= y <= hi.y
                                          and lo.z <= z <= hi.z):
                continue
            if req.min_score > 0.0 and g.score < req.min_score:
                continue
            msg = Grasp(score=g.score, width=g.width, height=g.height, depth=g.depth)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = g.position
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = g.quaternion
            res.grasps.append(msg)
        if req.max_grasps > 0:
            res.grasps = res.grasps[:req.max_grasps]
        res.success = True
        res.message = f'{len(res.grasps)} grasps'
        return res


def main(args=None) -> None:
    """Run the mock grasp server."""
    rclpy.init(args=args)
    node = None
    try:
        node = MockGraspServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
