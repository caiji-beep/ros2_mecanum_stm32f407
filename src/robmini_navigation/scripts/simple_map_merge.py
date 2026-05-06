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
        self.declare_parameter("clear_regions", [""])
        self.declare_parameter("clear_robot_frames", [""])
        self.declare_parameter("clear_robot_initial_poses", [""])
        self.declare_parameter("clear_robot_radius", 0.35)
        self.declare_parameter("clear_robot_path_distance", 0.15)
        self.declare_parameter("max_clear_robot_poses", 2000)

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
        self.clear_regions = self.parse_clear_regions(
            self.get_parameter("clear_regions").get_parameter_value().string_array_value
        )
        self.clear_robot_frames = [
            str(frame).strip()
            for frame in self.get_parameter("clear_robot_frames").get_parameter_value().string_array_value
            if str(frame).strip()
        ]
        self.clear_robot_initial_poses = self.parse_clear_robot_initial_poses(
            self.get_parameter("clear_robot_initial_poses").get_parameter_value().string_array_value
        )
        self.clear_robot_radius = float(self.get_parameter("clear_robot_radius").value)
        self.clear_robot_path_distance = float(self.get_parameter("clear_robot_path_distance").value)
        self.max_clear_robot_poses = int(self.get_parameter("max_clear_robot_poses").value)
        self.clear_robot_poses: Dict[str, List[Tuple[float, float]]] = {}
        for index, frame in enumerate(self.clear_robot_frames):
            poses = []
            if index < len(self.clear_robot_initial_poses):
                poses.append(self.clear_robot_initial_poses[index])
            self.clear_robot_poses[frame] = poses

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
        self.update_clear_robot_poses()

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

        self.clear_configured_regions(min_x, min_y, width, height, merged_data)
        self.clear_robot_trajectory_regions(min_x, min_y, width, height, merged_data)

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

    def parse_clear_regions(self, values) -> List[Tuple[float, float, float]]:
        regions = []
        for value in values:
            text = str(value).strip()
            if not text:
                continue

            parts = [part.strip() for part in text.split(",")]
            if len(parts) != 3:
                self.get_logger().warn(f"ignoring invalid clear region '{text}', expected 'x,y,radius'")
                continue

            try:
                x, y, radius = (float(part) for part in parts)
            except ValueError:
                self.get_logger().warn(f"ignoring invalid clear region '{text}', expected numbers")
                continue

            if radius <= 0.0:
                continue
            regions.append((x, y, radius))
        return regions

    def parse_clear_robot_initial_poses(self, values) -> List[Tuple[float, float]]:
        poses = []
        for value in values:
            text = str(value).strip()
            if not text:
                continue

            parts = [part.strip() for part in text.split(",")]
            if len(parts) != 2:
                self.get_logger().warn(f"ignoring invalid clear robot initial pose '{text}', expected 'x,y'")
                continue

            try:
                x, y = (float(part) for part in parts)
            except ValueError:
                self.get_logger().warn(f"ignoring invalid clear robot initial pose '{text}', expected numbers")
                continue

            poses.append((x, y))
        return poses

    def clear_configured_regions(
        self,
        min_x: float,
        min_y: float,
        width: int,
        height: int,
        merged_data: List[int],
    ) -> None:
        for clear_x, clear_y, radius in self.clear_regions:
            radius_cells = max(1, int(math.ceil(radius / self.resolution)))
            center_x = int(math.floor((clear_x - min_x) / self.resolution))
            center_y = int(math.floor((clear_y - min_y) / self.resolution))

            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if math.hypot(dx * self.resolution, dy * self.resolution) > radius:
                        continue

                    mx = center_x + dx
                    my = center_y + dy
                    if mx < 0 or my < 0 or mx >= width or my >= height:
                        continue

                    merged_data[my * width + mx] = 0

    def update_clear_robot_poses(self) -> None:
        for frame in self.clear_robot_frames:
            transform = self.lookup_map_transform(frame)
            if transform is None:
                continue

            x, y, _ = transform
            poses = self.clear_robot_poses.setdefault(frame, [])
            if poses:
                last_x, last_y = poses[-1]
                if math.hypot(x - last_x, y - last_y) < self.clear_robot_path_distance:
                    continue

            poses.append((x, y))
            if len(poses) > self.max_clear_robot_poses:
                del poses[:len(poses) - self.max_clear_robot_poses]

    def clear_robot_trajectory_regions(
        self,
        min_x: float,
        min_y: float,
        width: int,
        height: int,
        merged_data: List[int],
    ) -> None:
        if self.clear_robot_radius <= 0.0:
            return

        for poses in self.clear_robot_poses.values():
            for start, end in zip(poses, poses[1:]):
                self.clear_line(min_x, min_y, width, height, merged_data, start, end, self.clear_robot_radius)
            for x, y in poses:
                self.clear_circle(min_x, min_y, width, height, merged_data, x, y, self.clear_robot_radius)

    def clear_line(
        self,
        min_x: float,
        min_y: float,
        width: int,
        height: int,
        merged_data: List[int],
        start: Tuple[float, float],
        end: Tuple[float, float],
        radius: float,
    ) -> None:
        start_x, start_y = start
        end_x, end_y = end
        distance = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(1, int(math.ceil(distance / max(self.resolution, radius * 0.5))))

        for step in range(steps + 1):
            ratio = step / steps
            x = start_x + (end_x - start_x) * ratio
            y = start_y + (end_y - start_y) * ratio
            self.clear_circle(min_x, min_y, width, height, merged_data, x, y, radius)

    def clear_circle(
        self,
        min_x: float,
        min_y: float,
        width: int,
        height: int,
        merged_data: List[int],
        center_world_x: float,
        center_world_y: float,
        radius: float,
    ) -> None:
        radius_cells = max(1, int(math.ceil(radius / self.resolution)))
        center_x = int(math.floor((center_world_x - min_x) / self.resolution))
        center_y = int(math.floor((center_world_y - min_y) / self.resolution))

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx * self.resolution, dy * self.resolution) > radius:
                    continue

                mx = center_x + dx
                my = center_y + dy
                if mx < 0 or my < 0 or mx >= width or my >= height:
                    continue

                merged_data[my * width + mx] = 0


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
