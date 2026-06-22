"""
dual_aruco_triangulation.launch.py

Launches:
  - usb_cam for cam1 (with calibration)
  - usb_cam for cam2 (with calibration)
  - aruco_recognition for cam1
  - aruco_recognition for cam2
  - aruco_triangulation_node
  - static_transform_publisher: world → cam1_optical_frame  (identity — cam1 IS world)
  - static_transform_publisher: world → cam2_optical_frame  (YOU MUST MEASURE THIS)

Usage:
  ros2 launch aruco_triangulation dual_aruco_triangulation.launch.py \
      cam1_device:=/dev/video0 \
      cam2_device:=/dev/video2 \
      cam1_calib:=file:///home/$USER/camera_calib/cam1.yaml \
      cam2_calib:=file:///home/$USER/camera_calib/cam2.yaml \
      marker_id:=0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Arguments
    # ------------------------------------------------------------------ #
    args = [
        DeclareLaunchArgument('cam1_device', default_value='/dev/video0'),
        DeclareLaunchArgument('cam2_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'cam1_calib',
            default_value='file:///home/user/camera_calib/cam1.yaml'),
        DeclareLaunchArgument(
            'cam2_calib',
            default_value='file:///home/user/camera_calib/cam2.yaml'),
        DeclareLaunchArgument('marker_id',    default_value='0'),
        DeclareLaunchArgument('marker_size',  default_value='0.05'),   # metres
        DeclareLaunchArgument('world_frame',  default_value='world'),
        # ArUco dictionary: DICT_4X4_50 = 0  (see ros2_aruco docs)
        DeclareLaunchArgument('aruco_dict',   default_value='DICT_5X5_250'),

        # ---- cam2 → world extrinsic (measure physically or via calibration) ----
        # Translation (metres): where is cam2 origin relative to cam1/world origin?
        DeclareLaunchArgument('cam2_tx', default_value='1.0'),
        DeclareLaunchArgument('cam2_ty', default_value='0.0'),
        DeclareLaunchArgument('cam2_tz', default_value='0.0'),
        # Rotation as quaternion x y z w
        DeclareLaunchArgument('cam2_qx', default_value='0.0'),
        DeclareLaunchArgument('cam2_qy', default_value='0.0'),
        DeclareLaunchArgument('cam2_qz', default_value='0.707'),
        DeclareLaunchArgument('cam2_qw', default_value='0.707'),
    ]

    # ------------------------------------------------------------------ #
    # Camera 1  (namespace: cam1)
    # ------------------------------------------------------------------ #
    cam1_group = GroupAction([
        PushRosNamespace('cam1'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device':        LaunchConfiguration('cam1_device'),
                'camera_info_url':     LaunchConfiguration('cam1_calib'),
                'publish_camera_info': True,
                'camera_name':         'cam1',
                'frame_id':            'cam1_optical_frame',
                'pixel_format':        'mjpeg2rgb',
            }],
            remappings=[
                ('image_raw',   '/cam1/image_raw'),
                ('camera_info', '/cam1/camera_info'),
            ],
        ),
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            parameters=[{
                'marker_size':        LaunchConfiguration('marker_size'),
                'aruco_dictionary_id': LaunchConfiguration('aruco_dict'),
                'image_topic':        '/cam1/image_raw',
                'camera_info_topic':  '/cam1/camera_info',
            }],
            remappings=[
                ('aruco_markers', '/cam1/aruco_markers'),
                ('aruco_image',   '/cam1/aruco_image'),
            ],
        ),
    ])

    # ------------------------------------------------------------------ #
    # Camera 2  (namespace: cam2)
    # ------------------------------------------------------------------ #
    cam2_group = GroupAction([
        PushRosNamespace('cam2'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device':        LaunchConfiguration('cam2_device'),
                'camera_info_url':     LaunchConfiguration('cam2_calib'),
                'publish_camera_info': True,
                'camera_name':         'cam2',
                'frame_id':            'cam2_optical_frame',
                'pixel_format':        'mjpeg2rgb',
            }],
            remappings=[
                ('image_raw',   '/cam2/image_raw'),
                ('camera_info', '/cam2/camera_info'),
            ],
        ),
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            parameters=[{
                'marker_size':        LaunchConfiguration('marker_size'),
                'aruco_dictionary_id': LaunchConfiguration('aruco_dict'),
                'image_topic':        '/cam2/image_raw',
                'camera_info_topic':  '/cam2/camera_info',
            }],
            remappings=[
                ('aruco_markers', '/cam2/aruco_markers'),
                ('aruco_image',   '/cam2/aruco_image'),
            ],
        ),
    ])

    # ------------------------------------------------------------------ #
    # Static TF:  world ← cam1_optical_frame  (cam1 is at world origin)
    # ------------------------------------------------------------------ #
    tf_world_cam1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_cam1',
        arguments=[
            # x  y  z  qx qy qz qw  parent         child
            '0', '0', '0', '0', '0', '0', '1',
            'world', 'cam1_optical_frame'
        ],
    )

    # ------------------------------------------------------------------ #
    # Static TF:  world ← cam2_optical_frame  (MEASURE PHYSICALLY!)
    # Defaults put cam2 1 m along X, rotated 90° around Z
    # ------------------------------------------------------------------ #
    tf_world_cam2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_cam2',
        arguments=[
            LaunchConfiguration('cam2_tx'),
            LaunchConfiguration('cam2_ty'),
            LaunchConfiguration('cam2_tz'),
            LaunchConfiguration('cam2_qx'),
            LaunchConfiguration('cam2_qy'),
            LaunchConfiguration('cam2_qz'),
            LaunchConfiguration('cam2_qw'),
            'world', 'cam2_optical_frame'
        ],
    )

    # ------------------------------------------------------------------ #
    # Triangulation Node
    # ------------------------------------------------------------------ #
    triangulation_node = Node(
        package='aruco_triangulation',
        executable='triangulation_node',
        name='aruco_triangulation_node',
        parameters=[{
            'marker_id':          LaunchConfiguration('marker_id'),
            'world_frame':        LaunchConfiguration('world_frame'),
            'cam1_frame':         'cam1_optical_frame',
            'cam2_frame':         'cam2_optical_frame',
            'use_tf':             True,
            'max_discrepancy_m':  0.15,
            'cam1_topic':         '/cam1/aruco_markers',
            'cam2_topic':         '/cam2/aruco_markers',
        }],
    )

    return LaunchDescription(
        args + [
            cam1_group,
            tf_world_cam1,
            tf_world_cam2,
            # Delay cam2 by 3 seconds so cam1 fully opens the device first
            TimerAction(period=3.0, actions=[cam2_group]),
            # Delay triangulation until both cameras are up
            TimerAction(period=6.0, actions=[triangulation_node]),
        ]
    )
