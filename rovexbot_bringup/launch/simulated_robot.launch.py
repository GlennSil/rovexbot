import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True",
            "use_sim_time": "True"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # --- Désactivé pour l'instant : décommenter ici ET dans la liste du return ---
    #
    # localization = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("rovexbot_localization"),
    #         "launch",
    #         "global_localization.launch.py"
    #     ),
    #     condition=UnlessCondition(use_slam)
    # )
    #
    # slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("rovexbot_mapping"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     condition=IfCondition(use_slam)
    # )
    #
    # navigation = IncludeLaunchDescription(   # le package rovexbot_navigation n'existe pas encore
    #     os.path.join(
    #         get_package_share_directory("rovexbot_navigation"),
    #         "launch",
    #         "navigation.launch.py"
    #     ),
    # )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        # localization,
        # slam,
        # navigation,
        rviz,
    ])
