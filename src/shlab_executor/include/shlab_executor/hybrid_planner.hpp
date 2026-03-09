#ifndef SHLAB_EXECUTOR_HYBRID_PLANNER_HPP
#define SHLAB_EXECUTOR_HYBRID_PLANNER_HPP

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace shlab_executor {

struct Node {
  std::vector<double> joint_config;
  int parent_index = -1;
  Node(const std::vector<double> &config) : joint_config(config) {}
};

class HybridPlanner {
public:
  HybridPlanner(
      const std::shared_ptr<const moveit::core::RobotModel> &robot_model,
      const std::shared_ptr<planning_scene::PlanningScene> &planning_scene,
      const std::string &group_name);

  std::vector<std::vector<double>> plan(const std::vector<double> &start,
                                        const std::vector<double> &goal);

  std::vector<std::vector<double>>
  smooth_path(const std::vector<std::vector<double>> &path);

private:
  std::shared_ptr<const moveit::core::RobotModel> robot_model_;
  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
  const moveit::core::JointModelGroup *joint_model_group_;
  moveit::core::RobotStatePtr
      robot_state_; // Pre-allocated state for performance

  double step_size_ = 0.15;
  double goal_bias_ = 0.40;
  int max_iterations_ = 30000;
  double k_att_ = 0.6;
  double k_rep_ = 0.02;
  double d0_ = 0.30;

  std::vector<double> sample_config(const std::vector<double> &goal);
  int find_nearest_node(const std::vector<Node> &tree,
                        const std::vector<double> &target);
  std::vector<double> steer(const std::vector<double> &from,
                            const std::vector<double> &target,
                            const std::vector<double> &goal);
  bool is_state_valid(const std::vector<double> &config);
  bool is_path_valid(const std::vector<double> &from,
                     const std::vector<double> &to);

  double distance(const std::vector<double> &q1, const std::vector<double> &q2);
  std::vector<double> calculate_apf_gradient(const std::vector<double> &q,
                                             const std::vector<double> &q_goal);

  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_01_;
};

} // namespace shlab_executor

#endif
