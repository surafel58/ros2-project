from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lab_1_package')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    
    # Process the URDF file
    robot_description_content = Command(['xacro ', urdf_file])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Launch Ignition Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # Bridge for cmd_vel and camera
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/depth_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/depth_camera/depth@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )

    # object_detection_node = Node(
    #     package='lab_1_package',
    #     executable='detect_objects',
    #     output='screen'
    # )

    # Convert depth image to point cloud
    depth_to_pointcloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        output='screen',
        remappings=[
            ('image_rect', '/depth_camera/image_rect'),
            ('camera_info', '/camera_info'),
            ('points', '/depth_camera/points'),
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        # object_detection_node,
        depth_to_pointcloud
    ])