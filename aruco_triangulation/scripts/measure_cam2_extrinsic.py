#!/usr/bin/env python3
"""
measure_cam2_extrinsic.py

Place the ArUco marker at a FIXED, KNOWN position in the scene.
Run this script while BOTH cameras can see the marker.
It computes T_world_cam2 (= T_cam1_cam2) automatically from the
simultaneous observations — no ruler needed.

Usage:
    # In one terminal — launch cameras and aruco nodes only (no triangulation yet):
    ros2 launch aruco_triangulation dual_aruco_triangulation.launch.py

    # In another terminal:
    python3 measure_cam2_extrinsic.py

It will print the static_transform_publisher arguments to paste into the launch file.
"""

import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from scipy.spatial.transform import Rotation as R
import numpy as np


def pose_to_matrix(pose):
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    rot = R.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = t
    return T


class ExtrinsicMeasurer(Node):

    def __init__(self):
        super().__init__('extrinsic_measurer')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('n_samples', 30)

        self.marker_id = self.get_parameter('marker_id').value
        self.n_samples = self.get_parameter('n_samples').value

        self.samples_T_cam1_marker = []
        self.samples_T_cam2_marker = []

        self.latest_cam1 = None
        self.latest_cam2 = None

        self.sub1 = self.create_subscription(
            ArucoMarkers, '/cam1/aruco_markers', self._cb1, 10)
        self.sub2 = self.create_subscription(
            ArucoMarkers, '/cam2/aruco_markers', self._cb2, 10)

        self.timer = self.create_timer(0.1, self._collect)
        self.get_logger().info(
            f'Collecting {self.n_samples} samples... '
            'Keep the marker visible to BOTH cameras.')

    def _find(self, msg):
        for mid, pose in zip(msg.marker_ids, msg.poses):
            if mid == self.marker_id:
                return pose
        return None

    def _cb1(self, msg):
        self.latest_cam1 = msg

    def _cb2(self, msg):
        self.latest_cam2 = msg

    def _collect(self):
        if self.latest_cam1 is None or self.latest_cam2 is None:
            return
        p1 = self._find(self.latest_cam1)
        p2 = self._find(self.latest_cam2)
        if p1 is None or p2 is None:
            return

        self.samples_T_cam1_marker.append(pose_to_matrix(p1))
        self.samples_T_cam2_marker.append(pose_to_matrix(p2))

        n = len(self.samples_T_cam1_marker)
        self.get_logger().info(f'  Sample {n}/{self.n_samples}')

        if n >= self.n_samples:
            self._compute_and_print()
            rclpy.shutdown()

    def _compute_and_print(self):
        """
        T_cam1_marker and T_cam2_marker both express the marker in their
        respective camera frame.
        T_world_cam2 = T_cam1_cam2 = T_cam1_marker @ inv(T_cam2_marker)
        Average over all samples.
        """
        translations = []
        quaternions = []

        for T1, T2 in zip(self.samples_T_cam1_marker,
                          self.samples_T_cam2_marker):
            T_cam1_cam2 = T1 @ np.linalg.inv(T2)
            translations.append(T_cam1_cam2[:3, 3])
            q = R.from_matrix(T_cam1_cam2[:3, :3]).as_quat()
            quaternions.append(q)

        t_mean = np.mean(translations, axis=0)
        t_std = np.std(translations, axis=0)

        # Quaternion averaging — ensure consistent hemisphere
        qs = np.array(quaternions)
        if np.dot(qs[0], qs[-1]) < 0:
            qs[-1] = -qs[-1]
        q_mean = np.mean(qs, axis=0)
        q_mean /= np.linalg.norm(q_mean)

        print('\n' + '='*60)
        print('RESULT: cam2 extrinsic relative to cam1/world')
        print('='*60)
        print(f'  Translation (x y z): {t_mean[0]:.4f}  {t_mean[1]:.4f}  {t_mean[2]:.4f}')
        print(f'  Std dev      (x y z): {t_std[0]:.4f}  {t_std[1]:.4f}  {t_std[2]:.4f}')
        print(f'  Quaternion (x y z w): {q_mean[0]:.4f}  {q_mean[1]:.4f}  {q_mean[2]:.4f}  {q_mean[3]:.4f}')
        print()
        print('Paste these launch arguments into dual_aruco_triangulation.launch.py:')
        print(f"  cam2_tx:='{t_mean[0]:.4f}'")
        print(f"  cam2_ty:='{t_mean[1]:.4f}'")
        print(f"  cam2_tz:='{t_mean[2]:.4f}'")
        print(f"  cam2_qx:='{q_mean[0]:.4f}'")
        print(f"  cam2_qy:='{q_mean[1]:.4f}'")
        print(f"  cam2_qz:='{q_mean[2]:.4f}'")
        print(f"  cam2_qw:='{q_mean[3]:.4f}'")
        print()
        print('Or as a static_transform_publisher command:')
        print(f"  ros2 run tf2_ros static_transform_publisher "
              f"{t_mean[0]:.4f} {t_mean[1]:.4f} {t_mean[2]:.4f} "
              f"{q_mean[0]:.4f} {q_mean[1]:.4f} {q_mean[2]:.4f} {q_mean[3]:.4f} "
              f"world cam2_optical_frame")
        print('='*60)


def main():
    rclpy.init()
    node = ExtrinsicMeasurer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
