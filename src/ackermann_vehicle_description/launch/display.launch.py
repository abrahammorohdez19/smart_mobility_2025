from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_description_pkg = get_package_share_path('ackermann_vehicle_description')

    urdf_path = os.path.join(get_package_share_path('ackermann_vehicle_description'),
                             'urdf', 'ackermann_robot.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('ackermann_vehicle_description'),
                                    'rviz', 'config.rviz')
    
    robot_parameters_file_path = os.path.join(robot_description_pkg,
                                        'config', 'params.yaml')
    
    parameter_file_arg = DeclareLaunchArgument(
        'robot_parameters_file',
        default_value=robot_parameters_file_path,
        description='Archive file to define our robot'
    )

    robot_file = LaunchConfiguration('robot_parameters_file')

    

    # This line will allow us to execute 'xacro' command and convert the output to a string
    robot_description = ParameterValue(
        Command(
            ['xacro ', 
             urdf_path,
             ' ',
             'robot_parameters_file:=', 
             robot_file

             ]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        parameter_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        
        rviz2_node
    ])