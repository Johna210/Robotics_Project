import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lane_painting_path_robot'
    pkg_dir = get_package_share_directory(pkg_name)

    # Load the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(pkg_dir, 'worlds', 'track_world.sdf'), '-r'],
        output='screen'
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'lane_following_robot',
                  '-topic', 'robot_description',
                  '-x', '0',
                  '-y', '0',
                  '-z', '0.1'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_file}]
    )

    # Bridge to connect ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                  '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
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
        robot_state_publisher,
        spawn_robot,
        bridge,
        lane_detection,
        robot_controller
    ])
