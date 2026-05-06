#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SimpleMapMerge(Node):
    def __init__(self) -> None:
        super().__init__("simple_map_merge")

        self.declare_parameter("map_topics", ["/robmini_01/map", "/robmini_02/map"])
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("merged_map_topic", "/merged_map")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("free_threshold", 20)

        self.map_topics = [
            str(topic)
            for topic in self.get_parameter("map_topics").get_parameter_value().string_array_value
        ]
        if not self.map_topics:
            self.map_topics = ["/robmini_01/map", "/robmini_02/map"]

        self.target_frame = self.get_parameter("target_frame").value
        self.merged_map_topic = self.get_parameter("merged_map_topic").value
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.maps: Dict[str, OccupancyGrid] = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(OccupancyGrid, self.merged_map_topic, map_qos)
        for topic in self.map_topics:
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, topic=topic: self.map_callback(topic, msg),
                map_qos,
            )

        self.create_timer(self.publish_period, self.publish_merged_map)
        self.get_logger().info(
            f"merging maps {self.map_topics} into {self.merged_map_topic} in frame {self.target_frame}"
        )

    def map_callback(self, topic: str, msg: OccupancyGrid) -> None:
        self.maps[topic] = msg

    def lookup_map_transform(self, source_frame: str) -> Optional[Tuple[float, float, float]]:
        if source_frame == self.target_frame:
            return (0.0, 0.0, 0.0)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for transform {self.target_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return (translation.x, translation.y, yaw)

    def map_point_to_target(
        self,
        local_x: float,
        local_y: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        tf_x: float,
        tf_y: float,
        tf_yaw: float,
    ) -> Tuple[float, float]:
        map_x = origin_x + local_x * math.cos(origin_yaw) - local_y * math.sin(origin_yaw)
        map_y = origin_y + local_x * math.sin(origin_yaw) + local_y * math.cos(origin_yaw)
        target_x = tf_x + map_x * math.cos(tf_yaw) - map_y * math.sin(tf_yaw)
        target_y = tf_y + map_x * math.sin(tf_yaw) + map_y * math.cos(tf_yaw)
        return target_x, target_y

    def transformed_corners(self, msg: OccupancyGrid, transform: Tuple[float, float, float]) -> List[Tuple[float, float]]:
        width_m = msg.info.width * msg.info.resolution
        height_m = msg.info.height * msg.info.resolution
        origin = msg.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        tf_x, tf_y, tf_yaw = transform

        return [
            self.map_point_to_target(x, y, origin.position.x, origin.position.y, origin_yaw, tf_x, tf_y, tf_yaw)
            for x, y in ((0.0, 0.0), (width_m, 0.0), (0.0, height_m), (width_m, height_m))
        ]

    def publish_merged_map(self) -> None:
        if not self.maps:
            self.get_logger().warn("no input maps received yet", throttle_duration_sec=5.0)
            return

        usable_maps = []
        all_corners = []
        for topic in self.map_topics:
            msg = self.maps.get(topic)
            if msg is None:
                continue

            source_frame = msg.header.frame_id
            transform = self.lookup_map_transform(source_frame)
            if transform is None:
                continue

            usable_maps.append((msg, transform))
            all_corners.extend(self.transformed_corners(msg, transform))

        if not usable_maps or not all_corners:
            return

        min_x = min(x for x, _ in all_corners)
        min_y = min(y for _, y in all_corners)
        max_x = max(x for x, _ in all_corners)
        max_y = max(y for _, y in all_corners)

        width = max(1, int(math.ceil((max_x - min_x) / self.resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / self.resolution)))
        merged_data = [-1] * (width * height)

        for msg, transform in usable_maps:
            self.overlay_map(msg, transform, min_x, min_y, width, height, merged_data)

        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.target_frame
        merged.info.map_load_time = merged.header.stamp
        merged.info.resolution = self.resolution
        merged.info.width = width
        merged.info.height = height
        merged.info.origin.position.x = min_x
        merged.info.origin.position.y = min_y
        merged.info.origin.position.z = 0.0
        merged.info.origin.orientation.w = 1.0
        merged.data = merged_data

        self.publisher.publish(merged)

    def overlay_map(
        self,
        msg: OccupancyGrid,
        transform: Tuple[float, float, float],
        min_x: float,
        min_y: float,
        width: int,
        height: int,
        merged_data: List[int],
    ) -> None:
        origin = msg.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        tf_x, tf_y, tf_yaw = transform
        source_resolution = msg.info.resolution

        for y in range(msg.info.height):
            row_offset = y * msg.info.width
            local_y = (y + 0.5) * source_resolution
            for x in range(msg.info.width):
                value = msg.data[row_offset + x]
                if value < 0:
                    continue

                local_x = (x + 0.5) * source_resolution
                target_x, target_y = self.map_point_to_target(
                    local_x,
                    local_y,
                    origin.position.x,
                    origin.position.y,
                    origin_yaw,
                    tf_x,
                    tf_y,
                    tf_yaw,
                )

                mx = int(math.floor((target_x - min_x) / self.resolution))
                my = int(math.floor((target_y - min_y) / self.resolution))
                if mx < 0 or my < 0 or mx >= width or my >= height:
                    continue

                merged_index = my * width + mx
                current = merged_data[merged_index]
                normalized = self.normalize_cell(value)

                if current < 0:
                    merged_data[merged_index] = normalized
                elif normalized >= self.occupied_threshold or current >= self.occupied_threshold:
                    merged_data[merged_index] = max(current, normalized)
                elif normalized <= self.free_threshold and current <= self.free_threshold:
                    merged_data[merged_index] = min(current, normalized)
                else:
                    merged_data[merged_index] = max(current, normalized)

    def normalize_cell(self, value: int) -> int:
        if value >= self.occupied_threshold:
            return 100
        if value <= self.free_threshold:
            return 0
        return max(0, min(100, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleMapMerge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
