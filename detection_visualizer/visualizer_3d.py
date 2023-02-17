# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from vision_msgs.msg import Detection3DArray, Detection3D
from visualization_msgs.msg import MarkerArray, Marker


class Visualizer3D(Node):
    def __init__(self):
        super().__init__("visualizer_3d")

        output_image_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.vis_pub = self.create_publisher(
            MarkerArray, "/detections_vis", output_image_qos
        )

        self.detections_sub = self.create_subscription(
            Detection3DArray, "/detections", self.on_detections, 10
        )

        self.colors_rgb = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]

    # get best score and related class: (class_id, max_score)
    def best_hypothesis(self, results):
        class_id = int(results[0].hypothesis.class_id)
        max_score = results[0].hypothesis.score
        for res in results:
            if res.hypothesis.score > max_score:
                max_score = res.hypothesis.score
                class_id = int(res.hypothesis.class_id)
        return class_id, max_score

    def on_detections(self, detections_arr: Detection3DArray):
        marker_arr = MarkerArray()

        remove_old = Marker()
        remove_old.header = detections_arr.header
        remove_old.action = Marker.DELETEALL
        marker_arr.markers.append(remove_old)

        for i, det in enumerate(detections_arr.detections):
            box = Marker()
            class_id, max_score = self.best_hypothesis(det.results)
            box.id = i * 100000 + class_id + 1  # id 0 is remove marker
            box.header = det.header
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose = det.bbox.center
            box.scale = det.bbox.size
            box.color.r = self.colors_rgb[class_id][0]
            box.color.g = self.colors_rgb[class_id][1]
            box.color.b = self.colors_rgb[class_id][2]
            box.color.a = max(0.5 * max_score, 0.1)

            marker_arr.markers.append(box)

        self.vis_pub.publish(marker_arr)


def main():
    rclpy.init()
    rclpy.spin(Visualizer3D())
    rclpy.shutdown()
