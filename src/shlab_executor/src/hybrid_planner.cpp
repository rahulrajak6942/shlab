#include <algorithm>
#include <cmath>
#include <iostream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <shlab_executor/hybrid_planner.hpp>

namespace shlab_executor {

HybridPlanner::HybridPlanner(
    const std::shared_ptr<const moveit::core::RobotModel> &robot_model,
    const std::shared_ptr<planning_scene::PlanningScene> &planning_scene,
    const std::string &group_name)
    : robot_model_(robot_model), planning_scene_(planning_scene) {
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);

  // Pre-allocate a mutable robot state for internal calculations
  auto mutable_robot_model =
      std::const_pointer_cast<moveit::core::RobotModel>(robot_model_);
  robot_state_ =
      std::make_shared<moveit::core::RobotState>(mutable_robot_model);

  rng_.seed(std::random_device()());
}

std::vector<std::vector<double>>
HybridPlanner::plan(const std::vector<double> &start,
                    const std::vector<double> &goal) {
  std::vector<Node> tree;
  tree.emplace_back(start);

  for (int i = 0; i < max_iterations_; ++i) {
    std::vector<double> q_rand;
    if (dist_01_(rng_) < goal_bias_) {
      q_rand = goal;
    } else {
      q_rand = sample_config(goal);
    }

    int nearest_idx = find_nearest_node(tree, q_rand);
    std::vector<double> q_near = tree[nearest_idx].joint_config;

    std::vector<double> q_new = steer(q_near, q_rand, goal);

    if (is_state_valid(q_new) && is_path_valid(q_near, q_new)) {
      tree.emplace_back(q_new);
      tree.back().parent_index = nearest_idx;

      if (distance(q_new, goal) < step_size_ * 2.0) { // Reached vicinity
        std::vector<std::vector<double>> path;
        int curr = tree.size() - 1;
        while (curr != -1) {
          path.push_back(tree[curr].joint_config);
          curr = tree[curr].parent_index;
        }
        std::reverse(path.begin(), path.end());
        path.push_back(goal);
        return smooth_path(path);
      }
    }

    if (i % 500 == 0) {
      std::cout << "Planning iteration: " << i << " Tree size: " << tree.size()
                << std::endl;
    }
  }

  return {};
}

std::vector<double>
HybridPlanner::sample_config(const std::vector<double> &goal) {
  std::vector<double> q(goal.size());
  const std::vector<std::string> &joint_names =
      joint_model_group_->getActiveJointModelNames();
  for (size_t i = 0; i < joint_names.size(); ++i) {
    auto bounds =
        robot_model_->getJointModel(joint_names[i])->getVariableBounds()[0];
    std::uniform_real_distribution<double> joint_dist(bounds.min_position_,
                                                      bounds.max_position_);
    q[i] = joint_dist(rng_);
  }
  return q;
}

int HybridPlanner::find_nearest_node(const std::vector<Node> &tree,
                                     const std::vector<double> &target) {
  int best = 0;
  double min_dist = distance(tree[0].joint_config, target);
  for (size_t i = 1; i < tree.size(); ++i) {
    double d = distance(tree[i].joint_config, target);
    if (d < min_dist) {
      min_dist = d;
      best = i;
    }
  }
  return best;
}

std::vector<double> HybridPlanner::steer(const std::vector<double> &from,
                                         const std::vector<double> &target,
                                         const std::vector<double> &goal) {
  std::vector<double> dir(from.size());
  double d_target = distance(from, target);
  for (size_t i = 0; i < from.size(); ++i) {
    dir[i] = (target[i] - from[i]) / d_target;
  }

  std::vector<double> apf_grad = calculate_apf_gradient(from, goal);

  std::vector<double> combined_dir(from.size());
  double norm = 0;
  for (size_t i = 0; i < from.size(); ++i) {
    combined_dir[i] = dir[i] - apf_grad[i];
    norm += combined_dir[i] * combined_dir[i];
  }
  norm = std::sqrt(norm);
  if (norm < 1e-6)
    norm = 1.0;

  std::vector<double> q_new(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    q_new[i] = from[i] + (combined_dir[i] / norm) * step_size_;
  }

  return q_new;
}

std::vector<double>
HybridPlanner::calculate_apf_gradient(const std::vector<double> &q,
                                      const std::vector<double> &q_goal) {
  std::vector<double> grad(q.size(), 0.0);

  // 1. Attractive force
  for (size_t i = 0; i < q.size(); ++i) {
    grad[i] += k_att_ * (q[i] - q_goal[i]);
  }

  // 2. Repulsive force (reusing robot_state_)
  robot_state_->setJointGroupPositions(joint_model_group_, q);
  double dist = planning_scene_->distanceToCollision(*robot_state_);

  if (dist < d0_ && dist > 1e-4) {
    double eps = 0.01;
    for (size_t i = 0; i < q.size(); ++i) {
      std::vector<double> q_eps = q;
      q_eps[i] += eps;

      // Temporary set for finite difference
      robot_state_->setJointGroupPositions(joint_model_group_, q_eps);
      double dist_eps = planning_scene_->distanceToCollision(*robot_state_);

      double d_dist_dq = (dist_eps - dist) / eps;
      grad[i] += k_rep_ * (1.0 / dist - 1.0 / d0_) * (-1.0 / (dist * dist)) *
                 d_dist_dq;
    }
  }

  return grad;
}

bool HybridPlanner::is_state_valid(const std::vector<double> &config) {
  robot_state_->setJointGroupPositions(joint_model_group_, config);
  return !planning_scene_->isStateColliding(*robot_state_,
                                            joint_model_group_->getName());
}

bool HybridPlanner::is_path_valid(const std::vector<double> &from,
                                  const std::vector<double> &to) {
  int steps = 10;
  for (int i = 1; i <= steps; ++i) {
    std::vector<double> q(from.size());
    for (size_t j = 0; j < from.size(); ++j) {
      q[j] = from[j] + (to[j] - from[j]) * (static_cast<double>(i) / steps);
    }
    if (!is_state_valid(q))
      return false;
  }
  return true;
}

double HybridPlanner::distance(const std::vector<double> &q1,
                               const std::vector<double> &q2) {
  double sum = 0;
  for (size_t i = 0; i < q1.size(); ++i) {
    sum += (q1[i] - q2[i]) * (q1[i] - q2[i]);
  }
  return std::sqrt(sum);
}

std::vector<std::vector<double>>
HybridPlanner::smooth_path(const std::vector<std::vector<double>> &path) {
  if (path.size() < 3)
    return path;

  std::vector<std::vector<double>> smoothed = path;
  int smoothing_iterations = 50;
  double alpha = 0.1;
  double beta = 0.1;

  for (int iter = 0; iter < smoothing_iterations; ++iter) {
    for (size_t i = 1; i < smoothed.size() - 1; ++i) {
      for (size_t j = 0; j < smoothed[i].size(); ++j) {
        double prev = smoothed[i - 1][j];
        double next = smoothed[i + 1][j];
        double orig = path[i][j];
        smoothed[i][j] += alpha * (prev + next - 2 * smoothed[i][j]) +
                          beta * (orig - smoothed[i][j]);
      }
      if (!is_state_valid(smoothed[i])) {
        smoothed[i] = path[i];
      }
    }
  }
  return smoothed;
}

} // namespace shlab_executor
