#!/usr/bin/env python3
"""
ArUco Triangulation Node for ROS2 Jazzy
Subscribes to two aruco_msgs/ArucoMarkers topics, triangulates pose using
both camera observations, and publishes:
  - /aruco/triangulated_pose  (geometry_msgs/PoseStamped)
  - /aruco/cam1_pose          (geometry_msgs/PoseStamped)  -- for comparison
  - /aruco/cam2_pose          (geometry_msgs/PoseStamped)  -- for comparison
  - /aruco/triangulation_error (std_msgs/Float32)          -- reprojection error / consistency
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 – registers transforms for PoseStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from ros2_aruco_interfaces.msg import ArucoMarkers
from scipy.spatial.transform import Rotation as R


def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose to 4x4 homogeneous matrix."""
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    rot = R.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = t
    return T


def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix to geometry_msgs/Pose."""
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = T[0, 3]
    pose.position.y = T[1, 3]
    pose.position.z = T[2, 3]
    q = R.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def average_poses(T1, T2, w1=0.5, w2=0.5):
    """
    Weighted average of two 4x4 SE(3) matrices.
    Translation: weighted linear average.
    Rotation: weighted SLERP via quaternion averaging.
    """
    t_avg = w1 * T1[:3, 3] + w2 * T2[:3, 3]
    q1 = R.from_matrix(T1[:3, :3]).as_quat()
    q2 = R.from_matrix(T2[:3, :3]).as_quat()
    # Ensure quaternions are in the same hemisphere
    if np.dot(q1, q2) < 0:
        q2 = -q2
    q_avg = w1 * q1 + w2 * q2
    q_avg /= np.linalg.norm(q_avg)
    rot_avg = R.from_quat(q_avg).as_matrix()
    T_avg = np.eye(4)
    T_avg[:3, :3] = rot_avg
    T_avg[:3, 3] = t_avg
    return T_avg


def pose_distance(T1, T2):
    """Translation distance between two poses (metres)."""
    return np.linalg.norm(T1[:3, 3] - T2[:3, 3])


class ArucoTriangulationNode(Node):

    def __init__(self):
        super().__init__('aruco_triangulation_node')

        # ---------- Parameters ----------
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('world_frame', 'world')
        # cam1 is the reference (world = cam1 frame unless TF provided)
        self.declare_parameter('cam1_frame', 'cam1_optical_frame')
        self.declare_parameter('cam2_frame', 'cam2_optical_frame')
        # If True, use TF to transform cam2 pose into cam1 frame;
        # If False, the node expects cam2 to already be described in world frame
        self.declare_parameter('use_tf', True)
        # Max allowed position discrepancy (m) to still fuse; above this → use
        # the camera with lower covariance (cam1 by default)
        self.declare_parameter('max_discrepancy_m', 0.15)
        self.declare_parameter('cam1_topic', '/cam1/aruco_markers')
        self.declare_parameter('cam2_topic', '/cam2/aruco_markers')

        self.marker_id = self.get_parameter('marker_id').value
        self.world_frame = self.get_parameter('world_frame').value
        self.cam1_frame = self.get_parameter('cam1_frame').value
        self.cam2_frame = self.get_parameter('cam2_frame').value
        self.use_tf = self.get_parameter('use_tf').value
        self.max_discrepancy = self.get_parameter('max_discrepancy_m').value
        cam1_topic = self.get_parameter('cam1_topic').value
        cam2_topic = self.get_parameter('cam2_topic').value

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- State ----------
        self.latest_cam1: ArucoMarkers | None = None
        self.latest_cam2: ArucoMarkers | None = None
        self.max_age_sec = 0.5  # discard stale detections older than this

        # ---------- Subscribers ----------
        self.sub1 = self.create_subscription(
            ArucoMarkers, cam1_topic, self._cb_cam1, 10)
        self.sub2 = self.create_subscription(
            ArucoMarkers, cam2_topic, self._cb_cam2, 10)

        # ---------- Publishers ----------
        self.pub_fused = self.create_publisher(
            PoseStamped, '/aruco/triangulated_pose', 10)
        self.pub_cam1 = self.create_publisher(
            PoseStamped, '/aruco/cam1_pose', 10)
        self.pub_cam2 = self.create_publisher(
            PoseStamped, '/aruco/cam2_pose', 10)
        self.pub_error = self.create_publisher(
            Float32, '/aruco/triangulation_error', 10)

        # ---------- Timer (fusion at 20 Hz) ----------
        self.timer = self.create_timer(0.05, self._fuse)

        self.get_logger().info(
            f'Triangulation node ready. Watching marker id={self.marker_id}')
        self.get_logger().info(f'  cam1 topic: {cam1_topic}')
        self.get_logger().info(f'  cam2 topic: {cam2_topic}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_cam1(self, msg: ArucoMarkers):
        self.latest_cam1 = msg

    def _cb_cam2(self, msg: ArucoMarkers):
        self.latest_cam2 = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_marker(self, msg: ArucoMarkers):
        """Return the Pose for self.marker_id in msg, or None."""
        for mid, pose in zip(msg.marker_ids, msg.poses):
            if mid == self.marker_id:
                return pose
        return None

    def _is_fresh(self, msg: ArucoMarkers) -> bool:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        msg_sec = (msg.header.stamp.sec +
                   msg.header.stamp.nanosec * 1e-9)
        return (now_sec - msg_sec) < self.max_age_sec

    def _transform_pose_to_world(self, pose, source_frame: str):
        """
        Transform a geometry_msgs/Pose from source_frame to world_frame.
        Returns 4x4 numpy matrix or None on failure.
        """
        ps = PoseStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        try:
            ps_world = self.tf_buffer.transform(
                ps, self.world_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
            return pose_to_matrix(ps_world.pose)
        except Exception as e:
            self.get_logger().warn(
                f'TF transform {source_frame}→{self.world_frame} failed: {e}')
            return None

    def _make_pose_stamped(self, T: np.ndarray, frame: str) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose = matrix_to_pose(T)
        return ps

    # ------------------------------------------------------------------
    # Fusion
    # ------------------------------------------------------------------

    def _fuse(self):
        now_str = self.get_clock().now().to_msg()

        # ---- Collect fresh detections ----
        pose1_raw = None
        pose2_raw = None

        if self.latest_cam1 and self._is_fresh(self.latest_cam1):
            pose1_raw = self._find_marker(self.latest_cam1)

        if self.latest_cam2 and self._is_fresh(self.latest_cam2):
            pose2_raw = self._find_marker(self.latest_cam2)

        if pose1_raw is None and pose2_raw is None:
            return  # nothing to publish

        # ---- Transform to world frame ----
        T1_world = None
        T2_world = None

        if pose1_raw is not None:
            if self.use_tf:
                T1_world = self._transform_pose_to_world(
                    pose1_raw, self.cam1_frame)
            else:
                T1_world = pose_to_matrix(pose1_raw)

        if pose2_raw is not None:
            if self.use_tf:
                T2_world = self._transform_pose_to_world(
                    pose2_raw, self.cam2_frame)
            else:
                T2_world = pose_to_matrix(pose2_raw)

        # ---- Publish individual poses (for comparison) ----
        if T1_world is not None:
            self.pub_cam1.publish(
                self._make_pose_stamped(T1_world, self.world_frame))

        if T2_world is not None:
            self.pub_cam2.publish(
                self._make_pose_stamped(T2_world, self.world_frame))

        # ---- Fuse ----
        if T1_world is not None and T2_world is not None:
            discrepancy = pose_distance(T1_world, T2_world)
            err_msg = Float32()
            err_msg.data = float(discrepancy)
            self.pub_error.publish(err_msg)

            if discrepancy > self.max_discrepancy:
                self.get_logger().warn(
                    f'Large discrepancy between cameras: {discrepancy:.3f} m '
                    f'(> {self.max_discrepancy} m). Trusting cam1 only.')
                T_fused = T1_world
            else:
                # Equal-weight fusion (can tune w1/w2 per camera quality)
                T_fused = average_poses(T1_world, T2_world, w1=0.5, w2=0.5)

        elif T1_world is not None:
            T_fused = T1_world
            self.get_logger().debug('Only cam1 visible — using cam1 pose')
        else:
            T_fused = T2_world
            self.get_logger().debug('Only cam2 visible — using cam2 pose')

        self.pub_fused.publish(
            self._make_pose_stamped(T_fused, self.world_frame))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTriangulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
