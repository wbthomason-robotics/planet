#include "solver.hh"
#include "common.hh"

#include <limits>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include <armadillo>

#include "optim.hpp"

#include <boost/math/constants/constants.hpp>

#include "scenegraph.hh"

namespace planner::solver {
namespace {
  std::mutex bounds_mutex;
  std::unique_ptr<arma::vec> lower_bounds;
  std::unique_ptr<arma::vec> upper_bounds;

  void make_bounds(const int dim, const bool robot_movable, const Transform3r* const base_pose) {
    std::lock_guard<std::mutex> bounds_lock(bounds_mutex);
    if (lower_bounds != nullptr && upper_bounds != nullptr) {
      return;
    }

    lower_bounds = std::make_unique<arma::vec>(dim);
    upper_bounds = std::make_unique<arma::vec>(dim);
    // TODO(Wil): Spooky scary global state
    const int bounds_size = cspace::workspace_bounds->low.size();
    int offset            = 0;
    if (robot_movable) {
      lower_bounds->subvec(0, bounds_size - 1) = arma::vec(cspace::workspace_bounds->low);
      upper_bounds->subvec(0, bounds_size - 1) = arma::vec(cspace::workspace_bounds->high);
      // Fix the Z coordinate of the robot translation
      lower_bounds->at(2) = base_pose->translation().z();
      upper_bounds->at(2) = base_pose->translation().z();
      lower_bounds->at(3) = -boost::math::double_constants::pi;
      upper_bounds->at(3) = boost::math::double_constants::pi;
      offset += bounds_size + 1;
    }

    // Add pi bounds for the continuous joints
    for (size_t i = 0; i < cspace::cont_joint_idxs.size(); ++i) {
      lower_bounds->at(offset + i) = -boost::math::double_constants::pi;
      upper_bounds->at(offset + i) = boost::math::double_constants::pi;
    }

    offset += cspace::cont_joint_idxs.size();

    // Handle remaining joints
    for (size_t i = 0; i < cspace::joint_bounds.size(); ++i) {
      lower_bounds->at(offset + i) = cspace::joint_bounds[i].first;
      upper_bounds->at(offset + i) = cspace::joint_bounds[i].second;
    }
  }

  // Use Adam
  constexpr int GD_METHOD     = 6;
  constexpr double GD_ERR_TOL = 0.001;
}  // namespace

bool gradient_solve(const ob::StateSpace* const space,
                    const pred::LuaEnv<double>& grad_env,
                    const structures::robot::Robot* const robot,
                    const cspace::CompositeSpace::StateType* start,
                    spec::Formula* formula,
                    ob::State* result,
                    double& last_value) {
  // Separate out the state spaces for dimension and index information
  auto full_space  = space->as<ob::CompoundStateSpace>();
  auto robot_space = full_space->getSubspace(cspace::ROBOT_SPACE)->as<ob::CompoundStateSpace>();

  // Make sure we have the gradient function for the formula already in the environment
  grad_env.load_gradient(formula, cspace::num_dims);

  // Make a double vector of the continuous parts of the state.
  // It will have the form [robot_state... obj1... ... objn...]
  arma::vec state(pred::generate_state_vector(space, start, robot->base_movable));

  // Make the space bounds (only happens once)
  make_bounds(cspace::num_dims, robot->base_movable, robot->base_pose.get());

  auto grad_fn = [&](const arma::vec& inp_val, arma::vec* grad_out, void* opt_data) -> double {
    auto grad_result = grad_env.call_gradient(formula, inp_val);
    if (!grad_result) {
      return std::numeric_limits<double>::infinity();
    }

    auto [gradient, value] = *grad_result;
    if (grad_out != nullptr) {
      *grad_out = gradient;
    }

    last_value = value;
    return value;
  };

  optim::algo_settings_t settings;
  settings.gd_method = GD_METHOD;
  settings.err_tol   = GD_ERR_TOL;

  // Set bounds for state space
  settings.vals_bound   = true;
  settings.lower_bounds = *lower_bounds;
  settings.upper_bounds = *upper_bounds;

  // Run gradient descent
  bool success = optim::gd(state, grad_fn, nullptr, settings);

  // If we succeeded, copy the result into the output
  if (success) {
    // NOTE: We assume that `result` has been initialized to be the same space as `start` by the
    // caller
    // This copies the discrete part of the state (and the rest, but we overwrite that), which
    // hasn't changed
    // NOTE: This does *not* update the object poses (for manipulated objects)
    full_space->copyState(result, start);
    auto robot_state = result->as<ob::CompoundState>()->as<ob::CompoundState>(
    full_space->getSubspaceIndex(cspace::ROBOT_SPACE));
    int offset = 0;
    if (robot->base_movable) {
      auto robot_base_state = robot_state->as<cspace::RobotBaseSpace::StateType>(
      robot_space->getSubspaceIndex(cspace::BASE_SPACE));
      robot_base_state->setXYZ(state[0], state[1], state[2]);

      ob::SO2StateSpace::StateType& base_rotation = robot_base_state->rotation();
      base_rotation.value                         = state[3];
      offset += 4;
    }

    for (size_t i = 0; i < cspace::cont_joint_idxs.size(); ++i) {
      const auto idx          = cspace::cont_joint_idxs[i];
      auto* cont_joint_state  = robot_state->as<ob::SO2StateSpace::StateType>(idx);
      cont_joint_state->value = state[offset + i];
    }

    offset += cspace::cont_joint_idxs.size();

    auto* joint_state = robot_state->as<cspace::RobotJointSpace::StateType>(
    robot_space->getSubspaceIndex(cspace::JOINT_SPACE));
    for (size_t i = 0; i < cspace::joint_bounds.size(); ++i) {
      joint_state->values[i] = state[offset + i];
    }
  }

  return success;
}
}  // namespace planner::solver
