from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    planner = LaunchConfiguration("planner")
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("shlab_control"), "launch", "spawn.launch.py"])),
        launch_arguments = {"world" : world}.items()
    )

    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("shlab_moveit_config"), "launch", "moveit.launch.py"])),
        launch_arguments = {"rviz_config" : rviz_config}.items()
    )

    executor_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("shlab_executor"), "launch", "executor.launch.py"])),
        launch_arguments = {"planner" : planner}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "planner",
            default_value = "RRTConnectkConfigDefault",
            description = "Motion planner to be used."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value = PathJoinSubstitution([FindPackageShare("shlab_description"), "rviz", "moveit.rviz"]),
            description = "Rviz config file (absolute path) to use when launching rviz."
        ),
        DeclareLaunchArgument(
            "world",
            default_value = PathJoinSubstitution([FindPackageShare("shlab_control"), "worlds", "test_world.sdf"]),
            description = "Gazebo world file (absolute path or filename from the gz sim worlds collection) containing a custom world.",
        ),
        robot_node,
        moveit_node,
        executor_node
    ])