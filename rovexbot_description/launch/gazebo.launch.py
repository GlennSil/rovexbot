from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os import pathsep


def generate_launch_description():
    rovexbot_description_dir = get_package_share_directory("rovexbot_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Noms qui changent entre Ignition (Humble) et Gazebo Sim (versions récentes)
    resource_path_var = "IGN_GAZEBO_RESOURCE_PATH" if ros_distro == "humble" else "GZ_SIM_RESOURCE_PATH"
    gz_imu_type = "ignition.msgs.IMU" if ros_distro == "humble" else "gz.msgs.IMU"
    gz_laser_type = "ignition.msgs.LaserScan" if ros_distro == "humble" else "gz.msgs.LaserScan"
    gz_clock_type = "ignition.msgs.Clock" if ros_distro == "humble" else "gz.msgs.Clock"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rovexbot_description_dir, "urdf", "rovexbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
        rovexbot_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    robot_description = ParameterValue(
        Command(["xacro", " ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    model_path = str(Path(rovexbot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(rovexbot_description_dir, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        resource_path_var,
        model_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "rovexbot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[" + gz_clock_type,
            "/imu@sensor_msgs/msg/Imu[" + gz_imu_type,
            "/scan@sensor_msgs/msg/LaserScan[" + gz_laser_type,
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])