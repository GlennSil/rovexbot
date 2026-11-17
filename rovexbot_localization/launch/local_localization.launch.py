import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Fusion odometrie des roues + IMU (EKF de robot_localization).

    L'EKF est la seule source de la transformation odom -> base_footprint :
    diff_drive_controller a enable_odom_tf: false dans rovexbot_controllers.yaml.
    """

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("rovexbot_localization"),
                "config",
                "ekf.yaml",
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_localization,
    ])
