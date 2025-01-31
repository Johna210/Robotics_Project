import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lane_painting_path_robot'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(pkg_dir, 'worlds', 'track_world.sdf'), '-r'],
        output='screen'
    )

    # Spawn the robot using SDF
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-world', 'track_world',
            '-name', 'robot',
            '-file', os.path.join(pkg_dir, 'models', 'robot.sdf'),
            '-x', '-5.0',
            '-y', '0.0',
            '-z', '0.3',
            '-R', '0',
            '-P', '0',
            '-Y', '0'
        ],
        output='screen'
    )

    # Bridge to connect ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera image
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # Command velocity
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry
            '/model/lane_following_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # TF
            '/model/lane_following_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # Joint states
            '/model/lane_following_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/model/lane_following_robot/odometry', '/odom'),
            ('/model/lane_following_robot/tf', '/tf'),
            ('/model/lane_following_robot/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # Lane detection node
    lane_detection = Node(
        package=pkg_name,
        executable='lane_detector_node',
        name='lane_detector',
        output='screen'
    )

    # Robot controller node
    robot_controller = Node(
        package=pkg_name,
        executable='robot_controller_node',
        name='robot_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge,
        lane_detection,
        robot_controller
    ])
