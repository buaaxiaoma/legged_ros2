#!/usr/bin/env python3
"""Publish Unitree HeightMap observations from the generated MuJoCo terrain XML."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import HeightMap, SportModeState


@dataclass(frozen=True)
class TerrainBox:
    name: str
    group: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    top_z: float


def parse_float_list(value: str | None, expected: int, default: float = 0.0) -> list[float]:
    if value is None:
        return [default] * expected
    parts = value.split()
    if len(parts) < expected:
        parts.extend([str(default)] * (expected - len(parts)))
    return [float(part) for part in parts[:expected]]


def parse_string_array(value: object) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    text = str(value).strip()
    if not text:
        return []
    return [item.strip() for item in text.split(",") if item.strip()]


def parse_int_array(value: object) -> set[int]:
    if value is None:
        return set()
    if isinstance(value, (list, tuple)):
        return {int(item) for item in value}
    text = str(value).strip()
    if not text:
        return set()
    return {int(item.strip()) for item in text.split(",") if item.strip()}


def yaw_from_wxyz(quat: Iterable[float]) -> float:
    values = list(quat)
    if len(values) < 4:
        return 0.0
    w, x, y, z = (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TerrainHeightMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("terrain_heightmap_publisher")

        self.scene_xml_path = self.declare_parameter("scene_xml_path", "").value
        self.sport_mode_state_topic = self.declare_parameter(
            "sport_mode_state_topic", "/sportmodestate"
        ).value
        self.heightmap_topic = self.declare_parameter("heightmap_topic", "/heightmap").value
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.width = int(self.declare_parameter("width", 17).value)
        self.height = int(self.declare_parameter("height", 11).value)
        self.size_x = float(self.declare_parameter("size_x", 1.6).value)
        self.size_y = float(self.declare_parameter("size_y", 1.0).value)
        self.resolution = float(self.declare_parameter("resolution", 0.1).value)
        self.height_offset = float(self.declare_parameter("height_offset", 0.5).value)
        self.floor_height = float(self.declare_parameter("floor_height", 0.0).value)
        self.base_height_fallback = float(
            self.declare_parameter("base_height_fallback", 0.5).value
        )
        self.clip_min = float(self.declare_parameter("clip_min", -1.0).value)
        self.clip_max = float(self.declare_parameter("clip_max", 1.0).value)
        self.debug_log = bool(self.declare_parameter("debug_log", True).value)

        include_groups_value = self.declare_parameter("include_groups", [0, 1]).value
        skip_prefixes_value = self.declare_parameter(
            "skip_name_prefixes", ["start_", "side_lane_"]
        ).value
        skip_materials_value = self.declare_parameter("skip_materials", ["marker_red"]).value

        self.include_groups = parse_int_array(include_groups_value)
        self.skip_name_prefixes = parse_string_array(skip_prefixes_value)
        self.skip_materials = set(parse_string_array(skip_materials_value))

        self.base_x = 0.0
        self.base_y = 0.0
        self.base_z = self.base_height_fallback
        self.base_yaw = 0.0
        self.have_base_pose = False
        self.last_debug_time = 0.0
        self.last_missing_pose_warning_time = 0.0

        self.grid_points = self._build_grid()
        self.terrain_boxes = self._load_terrain_boxes()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sport_sub = self.create_subscription(
            SportModeState,
            self.sport_mode_state_topic,
            self._sport_mode_state_callback,
            qos,
        )
        self.heightmap_pub = self.create_publisher(HeightMap, self.heightmap_topic, 10)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._publish_heightmap)

        self.get_logger().info(
            "terrain_heightmap_publisher ready: scene=%s boxes=%d topic=%s grid=%dx%d size=(%.2f, %.2f)"
            % (
                self.scene_xml_path,
                len(self.terrain_boxes),
                self.heightmap_topic,
                self.width,
                self.height,
                self.size_x,
                self.size_y,
            )
        )

    def _build_grid(self) -> list[tuple[float, float]]:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("heightmap width and height must be positive")

        def coord(index: int, count: int, size: float) -> float:
            if count == 1:
                return 0.0
            return -0.5 * size + size * float(index) / float(count - 1)

        points: list[tuple[float, float]] = []
        for iy in range(self.height):
            y = coord(iy, self.height, self.size_y)
            for ix in range(self.width):
                x = coord(ix, self.width, self.size_x)
                points.append((x, y))
        return points

    def _load_terrain_boxes(self) -> list[TerrainBox]:
        scene_path_text = str(self.scene_xml_path).strip()
        if not scene_path_text:
            self.get_logger().warn("scene_xml_path is empty; publishing floor-only heightmaps")
            return []
        scene_path = Path(scene_path_text).expanduser()
        if not scene_path.exists():
            self.get_logger().warn(
                "scene_xml_path does not exist: %s; publishing floor-only heightmaps"
                % scene_path
            )
            return []

        try:
            root = ET.parse(scene_path).getroot()
        except (ET.ParseError, OSError) as exc:
            self.get_logger().error(
                "Failed to parse scene XML %s: %s; publishing floor-only heightmaps"
                % (scene_path, exc)
            )
            return []

        boxes: list[TerrainBox] = []
        for geom in root.iter("geom"):
            if geom.get("type", "").strip() != "box":
                continue

            name = geom.get("name", "")
            material = geom.get("material", "")
            group = int(geom.get("group", "0") or 0)
            if self.include_groups and group not in self.include_groups:
                continue
            if material in self.skip_materials:
                continue
            if any(name.startswith(prefix) for prefix in self.skip_name_prefixes):
                continue

            pos = parse_float_list(geom.get("pos"), 3, 0.0)
            size = parse_float_list(geom.get("size"), 3, 0.0)
            if size[0] <= 0.0 or size[1] <= 0.0:
                continue

            boxes.append(
                TerrainBox(
                    name=name,
                    group=group,
                    min_x=pos[0] - size[0],
                    max_x=pos[0] + size[0],
                    min_y=pos[1] - size[1],
                    max_y=pos[1] + size[1],
                    top_z=pos[2] + size[2],
                )
            )

        if boxes:
            min_x = min(box.min_x for box in boxes)
            max_x = max(box.max_x for box in boxes)
            self.get_logger().info(
                "Loaded %d terrain boxes from %s, x-range=[%.2f, %.2f]"
                % (len(boxes), scene_path, min_x, max_x)
            )
        else:
            self.get_logger().warn(
                "No terrain boxes loaded from %s; publishing floor-only heightmaps" % scene_path
            )
        return boxes

    def _sport_mode_state_callback(self, msg: SportModeState) -> None:
        try:
            self.base_x = float(msg.position[0])
            self.base_y = float(msg.position[1])
            self.base_z = float(msg.position[2])
            if not math.isfinite(self.base_z) or self.base_z <= 0.0:
                self.base_z = self.base_height_fallback
            self.base_yaw = yaw_from_wxyz(msg.imu_state.quaternion)
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            self.get_logger().warn("Ignoring malformed SportModeState: %s" % exc)
            return

        if not all(
            math.isfinite(value)
            for value in (self.base_x, self.base_y, self.base_z, self.base_yaw)
        ):
            self.get_logger().warn("Ignoring non-finite SportModeState pose")
            return

        self.have_base_pose = True

    def _terrain_height(self, x: float, y: float) -> float:
        height = self.floor_height
        for box in self.terrain_boxes:
            if box.min_x <= x <= box.max_x and box.min_y <= y <= box.max_y:
                height = max(height, box.top_z)
        return height

    def _publish_heightmap(self) -> None:
        if not self.have_base_pose:
            now = time.monotonic()
            if now - self.last_missing_pose_warning_time > 2.0:
                self.last_missing_pose_warning_time = now
                self.get_logger().warn(
                    "No SportModeState received yet; publishing heightmap around fallback origin."
                )

        cos_yaw = math.cos(self.base_yaw)
        sin_yaw = math.sin(self.base_yaw)
        data: list[float] = []
        min_value = float("inf")
        max_value = float("-inf")

        for local_x, local_y in self.grid_points:
            world_x = self.base_x + cos_yaw * local_x - sin_yaw * local_y
            world_y = self.base_y + sin_yaw * local_x + cos_yaw * local_y
            terrain_z = self._terrain_height(world_x, world_y)
            value = self.base_z - terrain_z - self.height_offset
            value = min(self.clip_max, max(self.clip_min, value))
            data.append(float(value))
            min_value = min(min_value, value)
            max_value = max(max_value, value)

        msg = HeightMap()
        msg.stamp = float(self.get_clock().now().nanoseconds) * 1.0e-9
        msg.frame_id = self.frame_id
        msg.resolution = float(self.resolution)
        msg.width = int(self.width)
        msg.height = int(self.height)
        msg.origin = [-0.5 * self.size_x, -0.5 * self.size_y]
        msg.data = data
        self.heightmap_pub.publish(msg)

        now = time.monotonic()
        if self.debug_log and now - self.last_debug_time > 2.0:
            self.last_debug_time = now
            self.get_logger().info(
                "heightmap pose=(%.2f, %.2f, %.2f yaw %.2f) range=[%.3f, %.3f]"
                % (self.base_x, self.base_y, self.base_z, self.base_yaw, min_value, max_value)
            )


def main() -> None:
    rclpy.init()
    node = TerrainHeightMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
