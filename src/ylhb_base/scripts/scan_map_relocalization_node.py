#!/usr/bin/env python3

import math
import sys
from collections import deque
from typing import NamedTuple

import numpy as np


class MatchScore(NamedTuple):
    score: float
    mean_distance: float
    inlier_ratio: float
    used_points: int


class MatchResult(NamedTuple):
    pose: tuple
    score: MatchScore


def is_supported_initialpose_frame(frame_id):
    return frame_id.strip("/") == "map"


def approximate_distance_field(occupied, resolution, max_cells=None):
    height, width = occupied.shape
    if max_cells is None:
        max_cells = height + width
    distances = np.full((height, width), np.inf, dtype=float)
    queue = deque()

    for y, x in np.argwhere(occupied):
        distances[y, x] = 0.0
        queue.append((int(y), int(x)))

    if not queue:
        return distances

    neighbors = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    while queue:
        y, x = queue.popleft()
        base = distances[y, x]
        if base / resolution > max_cells:
            continue
        for dy, dx, step in neighbors:
            ny = y + dy
            nx = x + dx
            if ny < 0 or ny >= height or nx < 0 or nx >= width:
                continue
            candidate = base + step * resolution
            if candidate < distances[ny, nx]:
                distances[ny, nx] = candidate
                queue.append((ny, nx))

    return distances


def _world_to_grid(x, y, origin, resolution):
    gx = int(math.floor((x - origin[0]) / resolution))
    gy = int(math.floor((y - origin[1]) / resolution))
    return gx, gy


def score_scan_points(
    scan_points,
    pose,
    distance_field,
    origin,
    resolution,
    inlier_distance=0.15,
    max_mean_distance=0.35,
):
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    distances = []
    height, width = distance_field.shape

    for px, py in scan_points:
        wx = x + cos_yaw * px - sin_yaw * py
        wy = y + sin_yaw * px + cos_yaw * py
        gx, gy = _world_to_grid(wx, wy, origin, resolution)
        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            continue
        value = float(distance_field[gy, gx])
        if math.isfinite(value):
            distances.append(value)

    if not distances:
        return MatchScore(score=0.0, mean_distance=float("inf"), inlier_ratio=0.0, used_points=0)

    mean_distance = float(np.mean(distances))
    inlier_ratio = float(np.mean(np.asarray(distances) <= inlier_distance))
    score = inlier_ratio * math.exp(-mean_distance / max_mean_distance)
    return MatchScore(
        score=score,
        mean_distance=mean_distance,
        inlier_ratio=inlier_ratio,
        used_points=len(distances),
    )


def _search_offsets(xy_radius, xy_step, yaw_radius, yaw_step):
    xy_values = np.arange(-xy_radius, xy_radius + xy_step * 0.5, xy_step)
    yaw_values = np.arange(-yaw_radius, yaw_radius + yaw_step * 0.5, yaw_step)
    for dx in xy_values:
        for dy in xy_values:
            for dyaw in yaw_values:
                yield float(dx), float(dy), float(dyaw)


def _best_pose_in_window(
    scan_points,
    seed_pose,
    distance_field,
    origin,
    resolution,
    xy_radius,
    xy_step,
    yaw_radius,
    yaw_step,
):
    best = None
    sx, sy, syaw = seed_pose
    for dx, dy, dyaw in _search_offsets(xy_radius, xy_step, yaw_radius, yaw_step):
        pose = (sx + dx, sy + dy, syaw + dyaw)
        score = score_scan_points(scan_points, pose, distance_field, origin, resolution)
        if best is None or score.score > best.score.score:
            best = MatchResult(pose=pose, score=score)
    return best


def refine_pose_near_seed(
    scan_points,
    seed_pose,
    distance_field,
    origin,
    resolution,
    min_score=0.45,
    min_inlier_ratio=0.45,
    max_mean_distance=0.30,
):
    coarse = _best_pose_in_window(
        scan_points,
        seed_pose,
        distance_field,
        origin,
        resolution,
        xy_radius=0.20,
        xy_step=0.05,
        yaw_radius=math.radians(10.0),
        yaw_step=math.radians(2.5),
    )
    fine = _best_pose_in_window(
        scan_points,
        coarse.pose,
        distance_field,
        origin,
        resolution,
        xy_radius=0.05,
        xy_step=0.02,
        yaw_radius=math.radians(3.0),
        yaw_step=math.radians(1.0),
    )

    if fine.score.score < min_score:
        return None
    if fine.score.inlier_ratio < min_inlier_ratio:
        return None
    if fine.score.mean_distance > max_mean_distance:
        return None
    return fine


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_quaternion_from_yaw(orientation, yaw):
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


def occupancy_grid_to_distance_field(map_msg, occupied_threshold=50):
    data = np.asarray(map_msg.data, dtype=np.int16).reshape(
        (map_msg.info.height, map_msg.info.width)
    )
    occupied = data >= occupied_threshold
    field = approximate_distance_field(occupied, map_msg.info.resolution)
    origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
    return field, origin, map_msg.info.resolution


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformException, TransformListener

    class ScanMapRelocalizationNode(Node):
        def __init__(self):
            super().__init__("scan_map_relocalization_node")
            self.declare_parameter("base_frame", "base_footprint")
            self.declare_parameter("min_score", 0.45)
            self.declare_parameter("min_inlier_ratio", 0.45)
            self.declare_parameter("max_mean_distance", 0.30)
            self.declare_parameter("max_scan_range", 8.0)
            self.declare_parameter("sample_stride", 2)
            self.declare_parameter("self_publish_ignore_seconds", 1.0)

            self._map_msg = None
            self._distance_field = None
            self._map_origin = None
            self._map_resolution = None
            self._latest_scan = None
            self._last_published_pose = None
            self._last_published_time = None

            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._initialpose_pub = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )
            self._scan_match_pub = self.create_publisher(
                PoseStamped, "/scan_match_pose", 10
            )

            self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
            self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/initialpose",
                self._on_initialpose,
                10,
            )
            self.get_logger().info(
                "waiting for /map, /scan, and a map-frame /initialpose"
            )

        def _on_map(self, msg):
            self._map_msg = msg
            (
                self._distance_field,
                self._map_origin,
                self._map_resolution,
            ) = occupancy_grid_to_distance_field(msg)
            self.get_logger().info(
                "map loaded for scan matching: "
                f"{msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution:.3f}"
            )

        def _on_scan(self, msg):
            self._latest_scan = msg

        def _is_own_correction(self, msg):
            if self._last_published_pose is None or self._last_published_time is None:
                return False
            age = (self.get_clock().now() - self._last_published_time).nanoseconds / 1e9
            if age > float(self.get_parameter("self_publish_ignore_seconds").value):
                return False
            pose = msg.pose.pose
            last = self._last_published_pose
            dx = pose.position.x - last[0]
            dy = pose.position.y - last[1]
            dyaw = yaw_from_quaternion(pose.orientation) - last[2]
            normalized_dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            return (
                math.hypot(dx, dy) < 0.02
                and abs(normalized_dyaw) < math.radians(2.0)
            )

        def _on_initialpose(self, msg):
            frame_id = msg.header.frame_id
            if self._is_own_correction(msg):
                self.get_logger().debug("ignoring self-published corrected /initialpose")
                return
            if not is_supported_initialpose_frame(frame_id):
                self.get_logger().error(
                    f"rejecting /initialpose in frame '{frame_id}'; "
                    "publish the coarse pose in map frame"
                )
                return
            if self._distance_field is None:
                self.get_logger().warn("cannot refine pose yet: /map has not been received")
                return
            if self._latest_scan is None:
                self.get_logger().warn("cannot refine pose yet: /scan has not been received")
                return

            scan_points = self._scan_to_base_points(self._latest_scan)
            if not scan_points:
                self.get_logger().warn("cannot refine pose: no valid laser points after filtering")
                return

            pose = msg.pose.pose
            seed = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
            result = refine_pose_near_seed(
                scan_points,
                seed,
                self._distance_field,
                self._map_origin,
                self._map_resolution,
                min_score=float(self.get_parameter("min_score").value),
                min_inlier_ratio=float(self.get_parameter("min_inlier_ratio").value),
                max_mean_distance=float(self.get_parameter("max_mean_distance").value),
            )

            if result is None:
                seed_score = score_scan_points(
                    scan_points,
                    seed,
                    self._distance_field,
                    self._map_origin,
                    self._map_resolution,
                )
                self.get_logger().warn(
                    "scan-map relocalization failed: "
                    f"score={seed_score.score:.3f}, mean_distance={seed_score.mean_distance:.3f}, "
                    f"inliers={seed_score.inlier_ratio:.2f}; not publishing a correction"
                )
                return

            self._publish_result(msg, result)

        def _scan_to_base_points(self, scan):
            base_frame = str(self.get_parameter("base_frame").value)
            try:
                transform = self._tf_buffer.lookup_transform(
                    base_frame,
                    scan.header.frame_id,
                    scan.header.stamp,
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                self.get_logger().warn(f"cannot transform scan to {base_frame}: {exc}")
                return []

            t = transform.transform.translation
            yaw = yaw_from_quaternion(transform.transform.rotation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            max_range = min(float(self.get_parameter("max_scan_range").value), scan.range_max)
            stride = max(1, int(self.get_parameter("sample_stride").value))
            points = []
            angle = scan.angle_min
            for index, distance in enumerate(scan.ranges):
                valid_range = scan.range_min <= distance <= max_range
                if index % stride == 0 and valid_range and math.isfinite(distance):
                    sx = math.cos(angle) * distance
                    sy = math.sin(angle) * distance
                    bx = t.x + cos_yaw * sx - sin_yaw * sy
                    by = t.y + sin_yaw * sx + cos_yaw * sy
                    points.append((bx, by))
                angle += scan.angle_increment
            return points

        def _publish_result(self, seed_msg, result):
            corrected = PoseWithCovarianceStamped()
            corrected.header.stamp = self.get_clock().now().to_msg()
            corrected.header.frame_id = "map"
            corrected.pose = seed_msg.pose
            corrected.pose.pose.position.x = result.pose[0]
            corrected.pose.pose.position.y = result.pose[1]
            corrected.pose.pose.position.z = 0.0
            set_quaternion_from_yaw(corrected.pose.pose.orientation, result.pose[2])

            pose_msg = PoseStamped()
            pose_msg.header = corrected.header
            pose_msg.pose = corrected.pose.pose

            self._last_published_pose = result.pose
            self._last_published_time = self.get_clock().now()
            self._scan_match_pub.publish(pose_msg)
            self._initialpose_pub.publish(corrected)
            self.get_logger().info(
                "scan-map relocalization succeeded: "
                f"x={result.pose[0]:.3f}, y={result.pose[1]:.3f}, "
                f"yaw={math.degrees(result.pose[2]):.1f}deg, "
                f"score={result.score.score:.3f}, "
                f"mean_distance={result.score.mean_distance:.3f}, "
                f"inliers={result.score.inlier_ratio:.2f}"
            )

    rclpy.init(args=args)
    node = ScanMapRelocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
