#!/usr/bin/env python3
"""
Vessel detector using Livox lidar point cloud.

Clusters the lidar PointCloud2, filters out buoy-sized objects, and
publishes the position and range of detected vessels.

Inputs:
    /sensors/livox_lidar/scan   sensor_msgs/PointCloud2

Outputs:
    /vessel_detection/closest   geometry_msgs/PointStamped  (map frame, closest vessel)
    /vessel_detection/markers   visualization_msgs/MarkerArray  (RViz)

Parameters:
    map_frame               TF frame for output (default: map)
    base_frame              TF frame of the boat (default: base_link)
    forward_arc_deg         Half-angle of forward detection arc in degrees (default: 100)
    max_detect_range        m — ignore points beyond this range (default: 40.0)
    max_point_height        m — ignore points above this height (default: 2.5)
    dbscan_eps              m — DBSCAN neighbourhood radius (default: 0.6)
    dbscan_min_samples      minimum points to form a cluster (default: 5)
    min_vessel_extent       m — clusters smaller than this are buoys (default: 0.8)
    max_buoy_extent         m — upper bound for buoy classification (default: 0.8)
    log_interval_sec        seconds between distance log messages (default: 2.0)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform support


class VesselDetector(Node):
    def __init__(self):
        super().__init__('vessel_detector')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('forward_arc_deg', 100.0)
        self.declare_parameter('max_detect_range', 40.0)
        self.declare_parameter('max_point_height', 2.5)
        self.declare_parameter('dbscan_eps', 0.6)
        self.declare_parameter('dbscan_min_samples', 5)
        self.declare_parameter('min_vessel_extent', 0.8)
        self.declare_parameter('max_vessel_extent', 5.0)
        self.declare_parameter('log_interval_sec', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.forward_arc = math.radians(self.get_parameter('forward_arc_deg').value)
        self.max_range = self.get_parameter('max_detect_range').value
        self.max_height = self.get_parameter('max_point_height').value
        self.eps = self.get_parameter('dbscan_eps').value
        self.min_samples = int(self.get_parameter('dbscan_min_samples').value)
        self.min_vessel_extent = self.get_parameter('min_vessel_extent').value
        self.max_vessel_extent = self.get_parameter('max_vessel_extent').value
        self.log_interval = self.get_parameter('log_interval_sec').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2, '/sensors/livox_lidar/scan', self._lidar_cb, 10)

        self.closest_pub = self.create_publisher(
            PointStamped, '/vessel_detection/closest', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/vessel_detection/markers', 10)

        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f'vessel_detector started — arc=±{self.get_parameter("forward_arc_deg").value}° '
            f'range={self.max_range}m  extent=[{self.min_vessel_extent}m, {self.max_vessel_extent}m]')


    def _lidar_cb(self, msg: PointCloud2):
        # Get boat pose in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF unavailable: {e}', throttle_duration_sec=3.0)
            return

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        q = tf.transform.rotation
        boat_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Get lidar → map transform
        try:
            lidar_tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Lidar TF unavailable: {e}', throttle_duration_sec=3.0)
            return

        # Read XYZ points
        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float32)

        if points.shape[0] == 0:
            return

        # Drop inf/nan rows (skip_nans only filters NaN, not ±inf)
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return

        # Transform points to map frame
        tx = lidar_tf.transform.translation
        r = lidar_tf.transform.rotation
        if any(math.isnan(v) for v in (r.w, r.x, r.y, r.z, tx.x, tx.y, tx.z)):
            return
        qw, qx, qy, qz = r.w, r.x, r.y, r.z
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),        1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),        2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
        ], dtype=np.float64)
        t = np.array([tx.x, tx.y, tx.z], dtype=np.float64)
        map_pts = (R @ points.T).T + t  # (N, 3)

        # Filter by height
        map_pts = map_pts[map_pts[:, 2] < self.max_height]
        if map_pts.shape[0] == 0:
            return

        # Filter by range from boat
        dx = map_pts[:, 0] - bx
        dy = map_pts[:, 1] - by
        ranges = np.hypot(dx, dy)
        map_pts = map_pts[ranges < self.max_range]
        dx = dx[ranges < self.max_range]
        dy = dy[ranges < self.max_range]

        if map_pts.shape[0] == 0:
            return

        # Filter by forward arc (relative to boat heading)
        bearings = np.arctan2(dy, dx) - boat_yaw
        # Normalise to [-π, π]
        bearings = (bearings + math.pi) % (2 * math.pi) - math.pi
        map_pts = map_pts[np.abs(bearings) < self.forward_arc]

        if map_pts.shape[0] < self.min_samples:
            self._publish_clear()
            return

        # DBSCAN clustering on XY only
        xy = map_pts[:, :2]
        labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit_predict(xy)

        vessels = []
        for label in set(labels):
            if label == -1:
                continue  # noise
            cluster = xy[labels == label]
            extent_x = float(cluster[:, 0].max() - cluster[:, 0].min())
            extent_y = float(cluster[:, 1].max() - cluster[:, 1].min())
            extent = max(extent_x, extent_y)

            if extent < self.min_vessel_extent:
                continue  # too small — buoy or noise
            if extent > self.max_vessel_extent:
                continue  # too large — wall, dock, or static structure

            cx = float(cluster[:, 0].mean())
            cy = float(cluster[:, 1].mean())
            dist = math.hypot(cx - bx, cy - by)
            bearing_deg = math.degrees(math.atan2(cy - by, cx - bx) - boat_yaw)
            bearing_deg = (bearing_deg + 180) % 360 - 180

            vessels.append({
                'cx': cx, 'cy': cy,
                'dist': dist,
                'bearing_deg': bearing_deg,
                'extent': extent,
                'n_pts': int(cluster.shape[0]),
            })

        self._publish_markers(vessels, msg.header.stamp)

        if not vessels:
            self._publish_clear()
            return

        # Pick closest vessel
        closest = min(vessels, key=lambda v: v['dist'])

        pt = PointStamped()
        pt.header.frame_id = self.map_frame
        pt.header.stamp = msg.header.stamp
        pt.point.x = closest['cx']
        pt.point.y = closest['cy']
        pt.point.z = 0.0
        self.closest_pub.publish(pt)

        # Throttled log
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds * 1e-9
        if elapsed >= self.log_interval:
            self._last_log_time = now
            for v in sorted(vessels, key=lambda v: v['dist']):
                self.get_logger().info(
                    f'[VESSEL]  dist={v["dist"]:.1f}m  '
                    f'bearing={v["bearing_deg"]:+.1f}°  '
                    f'extent={v["extent"]:.2f}m  '
                    f'pts={v["n_pts"]}')


    def _publish_clear(self):
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)
        self.marker_pub.publish(arr)


    def _publish_markers(self, vessels, stamp):
        arr = MarkerArray()
        for i, v in enumerate(vessels):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'vessels'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = v['cx']
            m.pose.position.y = v['cy']
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = max(v['extent'], 0.5)
            m.scale.y = max(v['extent'], 0.5)
            m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 1
            arr.markers.append(m)

            # Distance label
            txt = Marker()
            txt.header.frame_id = self.map_frame
            txt.header.stamp = stamp
            txt.ns = 'vessel_labels'
            txt.id = i + 1000
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = v['cx']
            txt.pose.position.y = v['cy']
            txt.pose.position.z = 1.8
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.8
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{v["dist"]:.1f}m  {v["bearing_deg"]:+.0f}°'
            txt.lifetime.sec = 1
            arr.markers.append(txt)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VesselDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()