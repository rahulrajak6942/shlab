from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    description_file = LaunchConfiguration("description_file")
    controllers = LaunchConfiguration("controllers")

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name = "xacro")]),
        " ",
        description_file,
        " ",
        "controllers:=",
        controllers
    ])

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        output = "screen",
        parameters = [{"robot_description" : ParameterValue(robot_description, value_type = str), "use_sim_time" : True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "description_file",
            default_value = PathJoinSubstitution([FindPackageShare("shlab_description"), "models", "robot.xacro"]),
            description = "URDF/XACRO description file (absolute path) with the robot."
        ),
        DeclareLaunchArgument(
            "controllers",
            default_value = '""',
            description = "Absolute path to YAML file with the controllers configuration."
        ),
        robot_state_publisher_node
    ])