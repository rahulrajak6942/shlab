#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shlab_description/srv/goal.hpp>
#include <shlab_executor/hybrid_planner.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class SHLabExecutor : public rclcpp::Node {
private:
  std::unique_ptr<MoveGroupInterface> move_group_interface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Service<shlab_description::srv::Goal>::SharedPtr executor_service;
  std::string planner_id;
  std::shared_ptr<shlab_executor::HybridPlanner> hybrid_planner;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  void add_obstacles() {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.resize(10);
    auto frame = move_group_interface->getPlanningFrame();

    auto set_obj = [&](int i, std::string id, double x, double y, double z,
                       double r, double p, double yaw, uint8_t type,
                       std::vector<double> dims) {
      objects[i].id = id;
      objects[i].header.frame_id = frame;
      objects[i].primitives.resize(1);
      objects[i].primitives[0].type = type;
      objects[i].primitives[0].dimensions.assign(dims.begin(), dims.end());
      objects[i].primitive_poses.resize(1);
      objects[i].primitive_poses[0].position.x = x;
      objects[i].primitive_poses[0].position.y = y;
      objects[i].primitive_poses[0].position.z = z;

      tf2::Quaternion q;
      q.setRPY(r, p, yaw);
      objects[i].primitive_poses[0].orientation.x = q.x();
      objects[i].primitive_poses[0].orientation.y = q.y();
      objects[i].primitive_poses[0].orientation.z = q.z();
      objects[i].primitive_poses[0].orientation.w = q.w();

      objects[i].operation = moveit_msgs::msg::CollisionObject::ADD;
    };

    // Mirroring test_world.sdf exactly
    set_obj(0, "cyl_1", 0.4, 0.1, 0.15, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::CYLINDER, {0.3, 0.03});
    set_obj(1, "box_1", 0.5, -0.2, 0.05, 0, 0, 0.5,
            shape_msgs::msg::SolidPrimitive::BOX, {0.1, 0.1, 0.1});
    set_obj(2, "sph_1", 0.6, 0.2, 0.05, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::SPHERE, {0.05});
    set_obj(3, "cyl_2", 0.3, -0.3, 0.25, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::CYLINDER, {0.5, 0.02});
    set_obj(4, "box_2", 0.7, 0.0, 0.1, 0, 0, -0.2,
            shape_msgs::msg::SolidPrimitive::BOX, {0.15, 0.15, 0.2});
    set_obj(5, "sph_2", 0.45, -0.4, 0.5, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::SPHERE, {0.06});
    set_obj(6, "cyl_3", 0.2, 0.4, 0.15, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::CYLINDER, {0.3, 0.04});
    set_obj(7, "cyl_4", 0.6, -0.4, 0.25, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::CYLINDER, {0.5, 0.03});
    set_obj(8, "sph_3", 0.3, 0.3, 0.1, 0, 0, 0,
            shape_msgs::msg::SolidPrimitive::SPHERE, {0.08});
    set_obj(9, "box_3", 0.5, 0.5, 0.15, 0, 0, 0.8,
            shape_msgs::msg::SolidPrimitive::BOX, {0.1, 0.2, 0.3});

    planning_scene_interface.applyCollisionObjects(objects);
  }

  void executor_callback(
      const std::shared_ptr<shlab_description::srv::Goal::Request> request,
      std::shared_ptr<shlab_description::srv::Goal::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received goal request: [%f, %f, %f]",
                request->position[0], request->position[1],
                request->position[2]);

    auto current_state = move_group_interface->getCurrentState(5.0);
    if (!current_state) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state.");
      response->success = false;
      return;
    }
    std::vector<double> start_joints;
    current_state->copyJointGroupPositions("manipulator", start_joints);

    auto robot_model = move_group_interface->getRobotModel();
    const moveit::core::JointModelGroup *joint_model_group =
        robot_model->getJointModelGroup("manipulator");

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = request->position[0];
    goal_pose.position.y = request->position[1];
    goal_pose.position.z = request->position[2];
    goal_pose.orientation.w = 0.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 1.0;
    goal_pose.orientation.z = 0.0;

    auto planning_scene =
        std::make_shared<planning_scene::PlanningScene>(robot_model);
    auto objects = planning_scene_interface.getObjects();
    for (const auto &pair : objects)
      planning_scene->processCollisionObjectMsg(pair.second);

    moveit::core::RobotState goal_state(robot_model);
    bool ik_success =
        goal_state.setFromIK(joint_model_group, goal_pose, "tool0", 1.0);

    if (ik_success) {
      collision_detection::CollisionResult res;
      collision_detection::CollisionRequest req;
      req.group_name = "manipulator";
      req.contacts = true;
      planning_scene->checkCollision(req, res, goal_state);
      if (res.collision) {
        RCLCPP_ERROR(this->get_logger(), "Goal IK is in COLLISION!");
        for (const auto &contact : res.contacts)
          RCLCPP_ERROR(this->get_logger(), "Contact: %s vs %s",
                       contact.first.first.c_str(),
                       contact.first.second.c_str());
        ik_success = false;
      }
    }

    if (!ik_success) {
      RCLCPP_ERROR(this->get_logger(), "IK failed or in collision.");
      response->success = false;
      return;
    }

    std::vector<double> goal_joints;
    goal_state.copyJointGroupPositions(joint_model_group, goal_joints);

    RCLCPP_INFO(this->get_logger(), "Planning path...");
    hybrid_planner = std::make_shared<shlab_executor::HybridPlanner>(
        robot_model, planning_scene, "manipulator");
    auto path = hybrid_planner->plan(start_joints, goal_joints);

    if (path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Planner failed.");
      response->success = false;
      return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names =
        joint_model_group->getActiveJointModelNames();
    for (size_t i = 0; i < path.size(); ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(path[i].begin(), path[i].end());
      point.time_from_start = rclcpp::Duration::from_seconds(i * 0.2);
      trajectory.joint_trajectory.points.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Executing...");
    auto result = move_group_interface->execute(trajectory);
    response->success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  }

public:
  SHLabExecutor()
      : Node("shlab_executor_node",
             rclcpp::NodeOptions()
                 .automatically_declare_parameters_from_overrides(true)) {
    this->get_parameter("planner", planner_id);
    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->executor_service = this->create_service<shlab_description::srv::Goal>(
        "/executor",
        std::bind(&SHLabExecutor::executor_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_);
  }

  void initialize() {
    move_group_interface =
        std::make_unique<MoveGroupInterface>(shared_from_this(), "manipulator");
    add_obstacles();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SHLabExecutor>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}