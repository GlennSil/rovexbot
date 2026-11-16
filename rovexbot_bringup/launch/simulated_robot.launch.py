import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    world_name = LaunchConfiguration("world_name")
    map_name = LaunchConfiguration("map_name")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Le monde Gazebo et la carte d'AMCL doivent représenter le même environnement
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="Monde Gazebo (dossier worlds de rovexbot_description)"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Carte utilisee par AMCL (dossier maps de rovexbot_mapping)"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": world_name
        }.items(),
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

    # AMCL + map_server : localisation sur une carte connue
    # (la carte doit correspondre au monde Gazebo : world_name:=small_house + map_name:=small_house)
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "True"
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    # --- Désactivé pour l'instant : décommenter ici ET dans la liste du return ---
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
        arguments=["-d", os.path.join(
            get_package_share_directory("rovexbot_localization"),
            "rviz",
            "global_localization.rviz"
        )],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        use_slam_arg,
        world_name_arg,
        map_name_arg,
        gazebo,
        controller,
        joystick,
        localization,
        # slam,
        # navigation,
        rviz,
    ])
