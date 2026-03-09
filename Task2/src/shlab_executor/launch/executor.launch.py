from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

def generate_launch_description():

    planner = LaunchConfiguration("planner")

    moveit_config = (
        MoveItConfigsBuilder(robot_name = "robot", package_name = "shlab_moveit_config")
        .robot_description_semantic(Path("models") / "robot.srdf")
        .trajectory_execution(Path("config") / "controllers.yaml")
        .joint_limits(Path("config") / "joint_limits.yaml")
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .planning_pipelines(
            pipelines = ["ompl"],
            default_planning_pipeline = "ompl"
        )
        .planning_scene_monitor(
            publish_robot_description = False,
            publish_robot_description_semantic = True,
            publish_planning_scene = True,
        )
        .to_moveit_configs()
    )

    executor_node = Node(
        package = "shlab_executor",
        executable = "executor",
        output = "screen",
        parameters = [moveit_config.to_dict(), {"planner" : planner, "use_sim_time": True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "planner",
            default_value = "RRTConnectkConfigDefault",
            description = "Motion planner to be used."
        ),
        executor_node
    ])