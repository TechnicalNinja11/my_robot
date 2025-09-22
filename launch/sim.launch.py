import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Process the URDF file
    pkg_path = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Save URDF to temporary file and convert to SDF
    urdf_path = '/tmp/robot.urdf'
    sdf_path = '/tmp/robot.sdf'
    
    with open(urdf_path, 'w') as f:
        f.write(robot_description)
    
    # Convert URDF to SDF for Gazebo Harmonic
    os.system(f'gz sdf -p {urdf_path} > {sdf_path}')

    # Start Gazebo Harmonic
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot after a delay (wait for Gazebo to start)
    spawn_robot = ExecuteProcess(
        cmd=['bash', '-c', f'sleep 3 && gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req \'name: "my_robot", allow_renaming: false, sdf_filename: "{sdf_path}"\''],
        output='screen'
    )

    # Robot State Publisher (for RViz visualization)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    return LaunchDescription([
        gazebo_process,
        TimerAction(period=2.0, actions=[spawn_robot]),
        robot_state_publisher
    ])