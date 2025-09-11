import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Check if use_sim_time should be true or false
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    # Check if file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"URDF file not found: {xacro_file}")
    
    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Create Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Create Joint State Publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform from world to base_link (optional but useful)
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Add nodes to launch
        node_static_tf,                    # Adds world->base_link transform
        node_joint_state_publisher,        # Publishes joint states
        node_robot_state_publisher         # Publishes transforms from URDF
    ])