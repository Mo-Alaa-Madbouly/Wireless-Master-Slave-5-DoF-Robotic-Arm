import os
from pathlib import Path # to work with file paths
from ament_index_python.packages import get_package_share_directory
# Get Package Directory

from launch import LaunchDescription 
# collects everything that will be executed
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# used for arguments, launching other launch files, setting environment variables.
from launch.substitutions import Command, LaunchConfiguration 
# to execute commands during launch
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node 
# to start ROS 2 nodes
from launch_ros.parameter_descriptions import ParameterValue 
# to pass parameters safely to nodes


def generate_launch_description():
    jarvis_description = get_package_share_directory("jarvis_description") # package path

    model_arg = DeclareLaunchArgument(name="model",
				    default_value=os.path.join(jarvis_description,
				    "urdf",
				    "jarvis.urdf.xacro"),
				    description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(jarvis_description).parent.resolve())
            ]
        )
    
    # convert xacro to plain URDF file
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # launches another launch file (to launch Gazebo)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", " empty.sdf"]
                    )
                ]
             )

    # Spawns robot inside Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "jarvis"],
    )

    # Connect ROS 2 topics with Gazebo topics
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )


    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])