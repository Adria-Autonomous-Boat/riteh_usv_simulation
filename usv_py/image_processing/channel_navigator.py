#!/usr/bin/env python3
"""
Channel navigator for GPS-waypoint buoy-gate task.

Behaviour (priority order):
  1. If a valid gate (red LEFT + green RIGHT of GPS-6 bearing, midpoint within
     `gate_commit_dist`) is visible -> thread the gate midpoint, then continue
     to GPS 6.
  2. Else if a single buoy is visible within `buoy_react_dist` -> offset the
     aim point so the buoy is passed on the correct side (red stays left, green
     stays right), then continue to GPS 6.
  3. Else -> drive straight to GPS 6.

"Left/right" is always relative to the boat's current bearing toward GPS 6,
NOT the boat's heading. This ensures that even after a COLREG avoidance
manoeuvre the boat re-acquires the buoy on the correct side.

The GPS-6 bearing is the baseline direction. Buoys are only reacted to when
they are between the boat and GPS 6 (positive forward component).

Inputs:
    /sensors/cameras/zed_camera/{image_raw, depth_image, camera_info}
    /goal_pose   geometry_msgs/PoseStamped  (optional runtime override)

Parameters:
    final_goal_lat / final_goal_lon   GPS destination in decimal degrees
    camera_ns                         topic namespace for the ZED camera
    gate_commit_dist              m — only thread a gate within this range
    buoy_react_dist               m — only offset for a single buoy within this range
    pass_distance                 m — lateral offset past a single buoy
    aim_beyond                    m — place Nav2 goal this far past the buoy/gate
    max_gate_width                m — reject red/green pairs farther apart than this
    max_gate_skew                 m — reject pairs whose forward distances differ this much
    goal_update_thresh            m — suppress Nav2 resend if goal moved less than this
    max_goal_dist                 m — clamp distant goals inside rolling costmap
    min_contour_area              px² — reject tiny blobs
    max_contour_area              px² — reject huge blobs (tree canopy)
    depth_window                  px — median-depth sample window half-size
    max_buoy_depth                m — ignore camera detections beyond this range
    roi_top_frac                  0-1 — ignore the top fraction of the image (sky)
    goal_reached_dist             m — "arrived at GPS 6" tolerance
    send_to_nav2                  bool — send NavigateToPose actions
    publish_debug                 bool — publish annotated camera image

Outputs:
    /channel_nav/next_goal    geometry_msgs/PoseStamped  (map frame)
    /channel_nav/markers      visualization_msgs/MarkerArray
    /channel_nav/debug_image  sensor_msgs/Image
    Nav2 NavigateToPose action
"""

import math

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from riteh_usv_sim.srv import SetGpsGoal

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class ChannelNavigator(Node):
    def __init__(self):
        super().__init__('channel_navigator')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('camera_ns', '/sensors/cameras/zed_camera')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Detection filters
        self.declare_parameter('min_contour_area', 150.0)
        self.declare_parameter('max_contour_area', 60000.0)
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('max_buoy_depth', 35.0)
        self.declare_parameter('roi_top_frac', 0.35)

        # Gate logic
        self.declare_parameter('gate_commit_dist', 7.0)   # m, commit to gate within this range
        self.declare_parameter('max_gate_width', 16.0)    # m, reject wide pairs
        self.declare_parameter('max_gate_skew', 6.0)      # m, reject diagonal pairs
        self.declare_parameter('aim_beyond', 5.0)         # m, goal placed past gate/buoy

        # Single-buoy logic
        self.declare_parameter('buoy_react_dist', 20.0)   # m, react to single buoy within this
        self.declare_parameter('pass_distance', 5.0)      # m, lateral offset from buoy

        # Navigation — GPS destination in decimal degrees
        self.declare_parameter('final_goal_lat', float('nan'))
        self.declare_parameter('final_goal_lon', float('nan'))
        self.declare_parameter('goal_reached_dist', 2.0)
        self.declare_parameter('goal_update_thresh', 1.0)
        self.declare_parameter('max_goal_dist', 30.0)
        self.declare_parameter('send_to_nav2', True)
        self.declare_parameter('publish_debug', True)

        cam_ns = self.get_parameter('camera_ns').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_area = self.get_parameter('min_contour_area').value
        self.max_area = self.get_parameter('max_contour_area').value
        self.depth_win = int(self.get_parameter('depth_window').value)
        self.max_buoy_depth = self.get_parameter('max_buoy_depth').value
        self.roi_top_frac = self.get_parameter('roi_top_frac').value
        self.gate_commit_dist = self.get_parameter('gate_commit_dist').value
        self.max_gate_width = self.get_parameter('max_gate_width').value
        self.max_gate_skew = self.get_parameter('max_gate_skew').value
        self.aim_beyond = self.get_parameter('aim_beyond').value
        self.buoy_react_dist = self.get_parameter('buoy_react_dist').value
        self.pass_distance = self.get_parameter('pass_distance').value
        self.goal_reached_dist = self.get_parameter('goal_reached_dist').value
        self.goal_update_thresh = self.get_parameter('goal_update_thresh').value
        self.max_goal_dist = self.get_parameter('max_goal_dist').value
        self.send_to_nav2 = self.get_parameter('send_to_nav2').value
        self.publish_debug = self.get_parameter('publish_debug').value

        self._goal_lat = self.get_parameter('final_goal_lat').value
        self._goal_lon = self.get_parameter('final_goal_lon').value
        self.final_goal = None       # set once GPS anchor is established
        self._gps_anchor = None      # (lat, lon, map_x, map_y) established at first fix

        # ---- HSV colour thresholds (OpenCV H in [0,180]) ----------------
        self.red_ranges = [
            (np.array([0,   120, 70]), np.array([10,  255, 255])),
            (np.array([170, 120, 70]), np.array([180, 255, 255])),
        ]
        self.green_range = (np.array([45, 110, 60]), np.array([85, 255, 255]))

        # ---- State ------------------------------------------------------
        self.bridge = CvBridge()
        self.K = None
        self._last_sent_goal = None
        self._active_goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- I/O --------------------------------------------------------
        self.create_subscription(
            CameraInfo, f'{cam_ns}/camera_info', self._camera_info_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_cb, 10)

        color_sub = Subscriber(self, Image, f'{cam_ns}/image_raw')
        depth_sub = Subscriber(self, Image, f'{cam_ns}/depth_image')
        self._sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._image_cb)

        self.goal_pub = self.create_publisher(PoseStamped, '/channel_nav/next_goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/channel_nav/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/channel_nav/debug_image', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---- BT integration ---------------------------------------------
        bt_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self.anchor_ready_pub = self.create_publisher(Bool, '/channel_nav/anchor_ready', bt_qos)
        self.goal_reached_pub = self.create_publisher(Bool, '/channel_nav/goal_reached', bt_qos)
        self.create_service(SetGpsGoal, '/channel_nav/set_gps_goal', self._set_gps_goal_cb)
        self._goal_reached_published = False
        self._paused = False
        self.create_subscription(Bool, '/channel_nav/paused', self._paused_cb, bt_qos)

        # Subscribe to GPS to establish a lat/lon ↔ map-frame anchor
        self.create_subscription(NavSatFix, '/navsat', self._navsat_cb, 10)

        self.get_logger().info(
            f'channel_navigator started  camera_ns={cam_ns}  '
            f'goal=({self._goal_lat:.6f}, {self._goal_lon:.6f})  send_to_nav2={self.send_to_nav2}')
        self.get_logger().info('Waiting for GPS fix to establish map anchor…')


    def _camera_info_cb(self, msg: CameraInfo):
        k = msg.k
        self.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy


    def _goal_pose_cb(self, msg: PoseStamped):
        """Runtime override via RViz 2D Goal Pose or ros2 topic pub."""
        self.final_goal = (msg.pose.position.x, msg.pose.position.y)
        self._last_sent_goal = None
        self.get_logger().info(
            f'GPS-6 destination updated (map frame): '
            f'({self.final_goal[0]:.2f}, {self.final_goal[1]:.2f})')


    def _paused_cb(self, msg: Bool):
        self._paused = msg.data
        self.get_logger().info(
            f'channel_navigator: {"PAUSED" if self._paused else "RESUMED"} by COLREG')
        if self._paused:
            if self._active_goal_handle is not None:
                self._active_goal_handle.cancel_goal_async()
                self._active_goal_handle = None
            self._last_sent_goal = None


    def _set_gps_goal_cb(self, req: SetGpsGoal.Request, resp: SetGpsGoal.Response):
        if self._gps_anchor is None:
            resp.success = False
            resp.message = 'GPS anchor not yet established'
            return resp
        mx, my = self._gps_to_map(req.lat, req.lon)
        self.final_goal = (mx, my)
        self._last_sent_goal = None
        self._goal_reached_published = False
        resp.success = True
        resp.message = f'Goal set: ({mx:.2f}, {my:.2f})'
        self.get_logger().info(
            f'BT: goal set via service → ({req.lat:.6f}, {req.lon:.6f}) '
            f'= map ({mx:.2f}, {my:.2f})')
        return resp


    def _navsat_cb(self, msg: NavSatFix):
        """On first valid GPS fix, record the anchor and convert the goal."""
        if self._gps_anchor is not None:
            return  # anchor already established
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        boat = self._boat_pose()
        if boat is None:
            return  # TF not ready yet

        self._gps_anchor = (msg.latitude, msg.longitude, boat[0], boat[1])
        self.get_logger().info(
            f'GPS anchor set: ({msg.latitude:.6f}, {msg.longitude:.6f}) '
            f'= map ({boat[0]:.2f}, {boat[1]:.2f})')

        ar = Bool()
        ar.data = True
        self.anchor_ready_pub.publish(ar)
        self.get_logger().info('BT: /channel_nav/anchor_ready published')

        if not (math.isnan(self._goal_lat) or math.isnan(self._goal_lon)):
            mx, my = self._gps_to_map(self._goal_lat, self._goal_lon)
            self.final_goal = (mx, my)
            self._last_sent_goal = None
            self.get_logger().info(
                f'GPS-6 ({self._goal_lat:.6f}, {self._goal_lon:.6f}) → '
                f'map ({mx:.2f}, {my:.2f})')


    def _gps_to_map(self, lat: float, lon: float):
        """Convert a GPS lat/lon to map-frame XY using the established anchor.

        Uses a flat-earth approximation valid for distances up to ~1 km.
        """
        anc_lat, anc_lon, anc_mx, anc_my = self._gps_anchor
        # metres per degree at this latitude
        m_per_lat = 111320.0
        m_per_lon = 111320.0 * math.cos(math.radians(anc_lat))
        # ENU: X = East = longitude, Y = North = latitude
        dy = (lat - anc_lat) * m_per_lat   # north offset in metres
        dx = (lon - anc_lon) * m_per_lon   # east offset in metres
        return (anc_mx + dx, anc_my + dy)


    def _image_cb(self, color_msg: Image, depth_msg: Image):
        if self.K is None:
            return
        if self.final_goal is None:
            self.get_logger().warn('No final_goal set — waiting', throttle_duration_sec=5.0)
            return

        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        red_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in self.red_ranges:
            red_mask |= cv2.inRange(hsv, lo, hi)
        green_mask = cv2.inRange(hsv, *self.green_range)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, color_msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'TF unavailable: {e}', throttle_duration_sec=2.0)
            return

        debug = color.copy() if self.publish_debug else None

        reds = self._detect_buoys(red_mask, depth, tf, debug, (0, 0, 255), 'R')
        greens = self._detect_buoys(green_mask, depth, tf, debug, (0, 255, 0), 'G')

        boat = self._boat_pose()
        if boat is None:
            return
        bx, by, byaw = boat

        if self._reached(self.final_goal, bx, by):
            self.get_logger().info('Reached GPS-6 destination.', throttle_duration_sec=10.0)
            if not self._goal_reached_published:
                gr = Bool()
                gr.data = True
                self.goal_reached_pub.publish(gr)
                self._goal_reached_published = True
                self.get_logger().info('BT: /channel_nav/goal_reached published')
            if debug is not None:
                self._publish_debug(debug, color_msg.header, reds, greens, 'ARRIVED')
            return

        aim, mode = self._decide(reds, greens, bx, by, byaw)

        self.get_logger().info(
            f'[{mode}]  R:{len(reds)} G:{len(greens)}  '
            f'boat=({bx:.1f},{by:.1f})  '
            f'aim={f"({aim[0]:.1f},{aim[1]:.1f})" if aim else "None"}',
            throttle_duration_sec=1.0)

        if aim is not None and not self._paused:
            dx, dy = aim[0] - bx, aim[1] - by
            yaw = math.atan2(dy, dx)
            self._go_to(aim, yaw)

        self._publish_markers(reds, greens)
        if debug is not None:
            self._publish_debug(debug, color_msg.header, reds, greens, mode)


    def _decide(self, reds, greens, bx, by, byaw):
        """Return (aim_xy, mode_string) for the current frame.

        Priority:
          1. Gate   — paired red(left) + green(right) within gate_commit_dist
          2. Single buoy — any buoy within buoy_react_dist, passed on correct side
          3. GPS 6  — straight-line carrot
        """
        gx, gy = self.final_goal
        dx, dy = gx - bx, gy - by
        n = math.hypot(dx, dy)
        if n < 1e-3:
            return None, 'ARRIVED'
        # bearing direction toward GPS 6
        dirx, diry = dx / n, dy / n

        def forward(p):
            return (p[0] - bx) * dirx + (p[1] - by) * diry

        def side(p):
            # positive = LEFT of GPS-6 bearing, negative = RIGHT
            return dirx * (p[1] - by) - diry * (p[0] - bx)

        # ---- 1. Gate search ------------------------------------------
        # Require both buoys ahead and within range; width/skew checks confirm
        # they are a matched pair. Side convention (red-left, green-right) is
        # enforced by the world layout, not re-checked here, to avoid false
        # rejects when the boat is slightly off the centreline.
        gate_reds = [r for r in reds if 0 < forward(r) <= self.gate_commit_dist]
        gate_greens = [g for g in greens if 0 < forward(g) <= self.gate_commit_dist]

        best_gate = None
        best_gate_score = float('inf')
        for r in gate_reds:
            for g in gate_greens:
                if abs(forward(r) - forward(g)) > self.max_gate_skew:
                    continue
                if math.dist(r, g) > self.max_gate_width:
                    continue
                mid_fwd = (forward(r) + forward(g)) / 2.0
                score = abs(mid_fwd - self.gate_commit_dist / 2.0)
                if score < best_gate_score:
                    best_gate = (r, g)
                    best_gate_score = score

        if best_gate is not None:
            r, g = best_gate
            mid = ((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0)
            # channel direction: red->green is cross-channel (red left, green right); +90° is down-channel
            vx, vy = g[0] - r[0], g[1] - r[1]
            cx, cy = -vy, vx
            cn = math.hypot(cx, cy)
            cdir = (cx / cn, cy / cn)
            # ensure channel dir points toward GPS 6
            if cdir[0] * dirx + cdir[1] * diry < 0.0:
                cdir = (-cdir[0], -cdir[1])
            aim = (mid[0] + self.aim_beyond * cdir[0],
                   mid[1] + self.aim_beyond * cdir[1])
            return aim, 'GATE'

        # ---- 2. Single buoy ------------------------------------------
        # Collect all buoys within buoy_react_dist that are ahead
        candidates = []
        for r in reds:
            f = forward(r)
            if 0 < f <= self.buoy_react_dist:
                candidates.append(('red', r, f))
        for g in greens:
            f = forward(g)
            if 0 < f <= self.buoy_react_dist:
                candidates.append(('green', g, f))

        if candidates:
            # pick the nearest buoy ahead
            color, buoy, _ = min(candidates, key=lambda t: t[2])

            # lateral normal vectors (relative to GPS-6 bearing)
            #   left  normal = (-diry,  dirx)
            #   right normal = ( diry, -dirx)
            if color == 'red':
                # red stays on LEFT (port) -> pass on its RIGHT side
                nx, ny = diry, -dirx
            else:
                # green stays on RIGHT (starboard) -> pass on its LEFT side
                nx, ny = -diry, dirx

            pass_pt = (buoy[0] + self.pass_distance * nx,
                       buoy[1] + self.pass_distance * ny)
            # aim beyond the pass point along the GPS-6 bearing
            aim = (pass_pt[0] + self.aim_beyond * dirx,
                   pass_pt[1] + self.aim_beyond * diry)
            return aim, f'SINGLE-{color.upper()}'

        # ---- 3. Straight to GPS 6 ------------------------------------
        clamped = self._clamp_goal((gx, gy), bx, by)
        return clamped, 'GPS6'


    def _detect_buoys(self, mask, depth, tf, debug=None, draw_bgr=None, label=''):
        """Return list of (x, y) map-frame positions for detected buoys."""
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fx, fy, cx, cy = self.K
        h, w = depth.shape[:2]
        roi_top = int(self.roi_top_frac * h)
        positions = []

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])
            if v < roi_top:
                continue

            d = self._sample_depth(depth, u, v, w, h)
            too_far = d is not None and d > self.max_buoy_depth

            if debug is not None:
                bad = d is None or too_far
                col = (128, 128, 128) if bad else draw_bgr
                cv2.drawContours(debug, [c], -1, col, 2)
                cv2.circle(debug, (u, v), 4, col, -1)
                txt = ('no-depth' if d is None
                       else f'{label} {d:.1f}m FAR' if too_far
                       else f'{label} {d:.1f}m')
                cv2.putText(debug, txt, (u + 6, v - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

            if d is None or too_far:
                continue

            pt = PointStamped()
            pt.point.x = (u - cx) * d / fx
            pt.point.y = (v - cy) * d / fy
            pt.point.z = d
            mp = do_transform_point(pt, tf).point
            positions.append((mp.x, mp.y))

        return positions


    def _sample_depth(self, depth, u, v, w, h):
        r = self.depth_win // 2
        patch = depth[max(0, v - r):min(h, v + r + 1),
                      max(0, u - r):min(w, u + r + 1)].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        return float(np.median(valid)) if valid.size > 0 else None


    def _go_to(self, point, yaw):
        mx, my = point
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = mx
        pose.pose.position.y = my
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(pose)

        if not self.send_to_nav2:
            return
        if (self._last_sent_goal is not None
                and math.dist(self._last_sent_goal, point) < self.goal_update_thresh):
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose not ready',
                                   throttle_duration_sec=5.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self._last_sent_goal = point
        self.get_logger().info(f'Nav2 goal -> ({mx:.1f}, {my:.1f})')


    def _goal_response_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            return
        self._active_goal_handle = handle


    def _boat_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw


    def _reached(self, target, bx, by):
        return math.dist((bx, by), target) < self.goal_reached_dist


    def _clamp_goal(self, target, bx, by):
        d = math.dist((bx, by), target)
        if d <= self.max_goal_dist:
            return target
        s = self.max_goal_dist / d
        return (bx + (target[0] - bx) * s, by + (target[1] - by) * s)


    def _publish_debug(self, debug, header, reds, greens, mode):
        h, w = debug.shape[:2]
        roi_top = int(self.roi_top_frac * h)
        if roi_top > 0:
            cv2.line(debug, (0, roi_top), (w, roi_top), (255, 255, 0), 1)
        cv2.putText(debug,
                    f'R:{len(reds)} G:{len(greens)}  [{mode}]',
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


    def _publish_markers(self, reds, greens):
        arr = MarkerArray()
        for i, (color, pts) in enumerate([((1.0, 0.0, 0.0), reds),
                                          ((0.0, 1.0, 0.0), greens)]):
            for j, (x, y) in enumerate(pts):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'buoys'
                m.id = i * 1000 + j
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.6
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 1.0
                arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ChannelNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()