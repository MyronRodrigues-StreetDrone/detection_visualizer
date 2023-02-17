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

from launch import LaunchDescription, LaunchContext
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share_dir = FindPackageShare("detection_visualizer")
    context = LaunchContext()

    obj_det_node = Node(
        package="detection_visualizer",
        executable="visualizer_3d",
        name="visualizer_3d",
        namespace="",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d"
            + PathJoinSubstitution([share_dir, "rviz", "vis_3d.rviz"]).perform(context)
        ],
    )

    return LaunchDescription(
        [
            rviz_node,
            obj_det_node,
        ]
    )
