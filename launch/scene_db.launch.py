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
Launch the scene database, the scene updater and (optionally) the mock grasp server.

    ros2 launch scene_db scene_db.launch.py camera:=camera mock_grasps:=true

The detector service (yolo_rgbd_detector) is launched separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('scene_db'), 'config', 'params.yaml')
    p = LaunchConfiguration('params_file')
    cam = LaunchConfiguration('camera')
    sim = {'use_sim_time': PythonExpression(["'", LaunchConfiguration('use_sim_time'),
                                             "' == 'true'"])}
    overrides = {'fixed_frame': LaunchConfiguration('fixed_frame')}
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('camera', default_value='camera',
                              description='Camera namespace (RealSense layout)'),
        DeclareLaunchArgument('fixed_frame', default_value='world'),
        DeclareLaunchArgument('database_file', default_value='~/.ros/scene_db.sqlite'),
        DeclareLaunchArgument('reset', default_value='false',
                              description='Start with an empty database'),
        DeclareLaunchArgument('mock_grasps', default_value='true',
                              description='Start the geometric mock of AnyGrasp'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='scene_db', executable='scene_db_node', name='scene_db', output='screen',
             parameters=[p, sim, overrides, {
                 'database_file': LaunchConfiguration('database_file'),
                 'reset_on_start': PythonExpression(["'", LaunchConfiguration('reset'),
                                                     "' == 'true'"])}]),
        Node(package='scene_db', executable='scene_updater', name='scene_updater',
             output='screen', parameters=[p, sim, overrides],
             remappings=[('color/image_raw', [cam, '/color/image_raw']),
                         ('depth/image_raw', [cam, '/aligned_depth_to_color/image_raw']),
                         ('color/camera_info', [cam, '/color/camera_info'])]),
        Node(package='scene_db', executable='mock_grasp_server', name='mock_grasp_server',
             output='screen', parameters=[p, sim],
             condition=IfCondition(LaunchConfiguration('mock_grasps'))),
    ])
