import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    port = LaunchConfiguration("port")
    i2c_bus = LaunchConfiguration("i2c_bus")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Port série de l'Arduino"
    )

    i2c_bus_arg = DeclareLaunchArgument(
        "i2c_bus",
        default_value="7",
        description="Bus I2C du MPU6050 (Jetson Orin Nano, broches 3/5 -> bus 7)"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={
            "port": port
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
            "use_sim_time": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rovexbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="rovexbot_firmware",
        executable="mpu6050_driver.py",
        parameters=[{"i2c_bus": i2c_bus}],
        output="screen"
    )

    # --- Désactivé pour l'instant : décommenter ici ET dans la liste du return ---
    #
    # laser_driver = Node(   # nécessite rplidar_ros et le fichier config/rplidar_a1.yaml
    #     package="rplidar_ros",
    #     executable="rplidar_node",
    #     name="rplidar_node",
    #     parameters=[os.path.join(
    #         get_package_share_directory("rovexbot_bringup"),
    #         "config",
    #         "rplidar_a1.yaml"
    #     )],
    #     output="screen"
    # )
    #
    # localization = IncludeLaunchDescription(   # nécessite le lidar et une carte
    #     os.path.join(
    #         get_package_share_directory("rovexbot_localization"),
    #         "launch",
    #         "global_localization.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "False"
    #     }.items(),
    #     condition=UnlessCondition(use_slam)
    # )
    #
    # slam = IncludeLaunchDescription(   # nécessite le lidar
    #     os.path.join(
    #         get_package_share_directory("rovexbot_mapping"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "False"
    #     }.items(),
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

    return LaunchDescription([
        use_slam_arg,
        port_arg,
        i2c_bus_arg,
        hardware_interface,
        controller,
        joystick,
        imu_driver_node,
        # laser_driver,
        # localization,
        # slam,
        # navigation,
    ])
