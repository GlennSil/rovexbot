import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Port série de l'Arduino
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Port série de l'Arduino",
    )

    # URDF du robot réel : is_sim:=False -> charge rovexbot_firmware/RovexbotInterface
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("rovexbot_description"),
                "urdf",
                "rovexbot.urdf.xacro",
            ),
            " is_sim:=False",
            " port:=", LaunchConfiguration("port"),
        ]),
        value_type=str,
    )

    # Publie les TF du robot à partir de l'URDF et de /joint_states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": False}],
    )

    # controller_manager : charge le plugin hardware et fait tourner la boucle read()/write()
    # (en simulation, c'est le plugin Gazebo qui joue ce rôle)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("rovexbot_controller"),
                "config",
                "rovexbot_controllers.yaml",
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        port_arg,
        robot_state_publisher_node,
        controller_manager,
    ])
