# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro
import yaml

def generate_launch_description():
    
    robot_description_path = os.path.join(
        get_package_share_directory('cross_description'),
        'urdf',
        'cross_robot.urdf.xacro')
        
    robot_description_config = xacro.process_file(robot_description_path)
        
    robot_description = {'robot_description': robot_description_config.toxml()}
        
    cross_mecanum_controller_path = os.path.join(
        get_package_share_directory('cross_control'),
        'config',
        'cross_controllers.yaml'
        )

    with open(cross_mecanum_controller_path, 'r') as f:
        base_controller = yaml.safe_load(f)["controller_manager"]["ros__parameters"]
    with open(cross_mecanum_controller_path, 'r') as f:
        cross_mecanum_controller = yaml.safe_load(f)["mecanum_omni_controller"]["ros__parameters"]
        
    return LaunchDescription([
        Node(package='controller_manager',
        name='controller_manager',
        executable='ros2_control_node',
        parameters=[base_controller, cross_mecanum_controller, robot_description],
        output='screen')
    ])
