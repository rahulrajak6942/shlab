from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    world = LaunchConfiguration("world")

    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("shlab_description"), "launch", "rsp.launch.py"])),
        launch_arguments = {
            "description_file" : PathJoinSubstitution([FindPackageShare("shlab_control"), "models", "robot.xacro"]),
            "controllers" : PathJoinSubstitution([FindPackageShare("shlab_control"), "config", "controllers.yaml"]),
        }.items()
    )

    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments = {
            "gz_args": ["-r -v4 ", world]
        }.items(),
    )

    ros_gz_bridge_node = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        arguments = [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output = "screen",
    )

    entity_spawner_node = Node(
        package = "ros_gz_sim",
        executable = "create",
        output = "screen",
        arguments = [
            "-name",
            "ur",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "true"
        ],
    )

    joint_state_broadcaster_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters = [{"use_sim_time" : True}]
    )

    joint_controller_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_trajectory_controller", "-c", "/controller_manager"],
        parameters = [{"use_sim_time" : True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value = PathJoinSubstitution([FindPackageShare("shlab_control"), "worlds", "test_world.sdf"]),
            description = "Gazebo world file (absolute path or filename from the gz sim worlds collection) containing a custom world.",
        ),
        rsp_node,
        gz_node,
        ros_gz_bridge_node,
        entity_spawner_node,
        joint_state_broadcaster_spawner_node,
        joint_controller_spawner_node
    ])