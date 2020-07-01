#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <nlopt.hpp>
#include <vector>

#include "common.hh"
#include "scenegraph.hh"
#include "solver.hh"

namespace planner::solver {
namespace {
  std::unique_ptr<Vec<double>> lower_bounds;
  std::unique_ptr<Vec<double>> upper_bounds;

  void make_bounds(const int dim, const bool robot_movable, const Transform3r* const base_pose) {
    if (lower_bounds != nullptr && upper_bounds != nullptr) {
      return;
    }

    lower_bounds = std::make_unique<Vec<double>>(dim, 0.0);
    upper_bounds = std::make_unique<Vec<double>>(dim, 0.0);
    // TODO(Wil): Spooky scary global state
    const size_t bounds_size = cspace::workspace_bounds->low.size();
    int offset            = 0;
    if (robot_movable) {
      for (size_t i = 0; i < bounds_size; ++i) {
        lower_bounds->at(i) = cspace::workspace_bounds->low.at(i);
        upper_bounds->at(i) = cspace::workspace_bounds->high.at(i);
      }

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
  struct OptData {
    OptData(Vec<addn::DN>& diff_state,
            const structures::robot::Robot* const robot,
            Transform3<addn::DN>& base_tf,
            structures::scenegraph::Graph* const sg,
            const addn::DN* const cont_vals,
            const addn::DN* const joint_vals,
            pred::LuaEnv<double>& grad_env,
            const spec::Formula& formula)
    : diff_state(diff_state)
    , robot(robot)
    , base_tf(base_tf)
    , sg(sg)
    , cont_vals(cont_vals)
    , joint_vals(joint_vals)
    , grad_env(grad_env)
    , formula(formula) {}

    Vec<addn::DN>& diff_state;
    const structures::robot::Robot* const robot;
    Transform3<addn::DN>& base_tf;
    structures::scenegraph::Graph* const sg;
    const addn::DN* const cont_vals;
    const addn::DN* const joint_vals;
    pred::LuaEnv<double>& grad_env;
    const spec::Formula& formula;
  };
}  // namespace

bool gradient_solve(const ob::StateSpace* const space,
                    pred::LuaEnv<double>& grad_env,
                    const structures::robot::Robot* const robot,
                    const cspace::CompositeSpace::StateType* start,
                    const spec::Formula& formula,
                    const Map<Str, Str>& bindings,
                    ob::State* result,
                    double& last_value) {
  // Separate out the state spaces for dimension and index information
  auto full_space  = space->as<ob::CompoundStateSpace>();
  auto robot_space = full_space->getSubspace(cspace::ROBOT_SPACE)->as<ob::CompoundStateSpace>();

  auto& sg = start->sg;

  // Make a double vector of the continuous parts of the state.
  // It will have the form [robot base pose (if it exists), continuous joints, other joints]
  auto state = pred::generate_state_vector(space, start, robot->base_movable);

  // Make the space bounds (only happens once)
  make_bounds(cspace::num_dims, robot->base_movable, robot->base_pose.get());

  Vec<addn::DN> diff_state(state.size(), addn::DN());
  const addn::DN* const cont_vals  = &(*diff_state.begin()) + (robot->base_movable ? 4 : 0);
  const addn::DN* const joint_vals = cont_vals + cspace::cont_joint_idxs.size();

  Transform3<addn::DN> base_tf(*(robot->base_pose));
  auto grad_fn = [](const Vec<double>& inp_val, Vec<double>& grad_out, void* f_data) -> double {
    auto* data = static_cast<OptData*>(f_data);
    auto& [diff_state, robot, base_tf, sg, cont_vals, joint_vals, grad_env, formula] = *data;
    // Convert to DNs
    for (size_t i = 0; i < diff_state.size(); ++i) {
      diff_state[i] = inp_val[i];
    }

    if (robot->base_movable) {
      pred::make_base_tf(diff_state, base_tf);
    }

    // Update the pose cache
    if (!grad_out.empty()) {
      sg->update_transforms(cont_vals,
                            joint_vals,
                            base_tf,
                            [](const structures::scenegraph::Node* const node,
                               const bool new_value,
                               const Transform3<addn::DN>& pose,
                               const Transform3r& coll_pose) {});
    } else {
      sg->update_transforms(cont_vals,
                            joint_vals,
                            base_tf,
                            [&](const structures::scenegraph::Node* const node,
                                const bool new_value,
                                const Transform3<addn::DN>& pose,
                                const Transform3r& coll_pose) {
                              data->grad_env.update_object(node->name, pose);
                            });
    }

    addn::DN diff_result;
    if (!grad_out.empty()) {
      // Compute each component of the gradient
      for (size_t i = 0; i < diff_state.size(); ++i) {
        diff_state[i].a = 1.0;
        // Rebuild the base pose when we're computing those components of the gradient, then once
        // after to set it back to normal
        if (robot->base_movable && i <= 4) {
          pred::make_base_tf(diff_state, base_tf);
        }

        sg->update_transforms(
        cont_vals,
        joint_vals,
        base_tf,
        [&](const structures::scenegraph::Node* const node,
            const bool new_value,
            const Transform3<addn::DN>& pose,
            const Transform3r& coll_pose) { data->grad_env.update_object(node->name, pose); },
        false);

        diff_result     = grad_env.call_formula<addn::DN>(formula);
        grad_out[i]     = diff_result.a;
        diff_state[i].a = 0.0;
      }
    } else {
      diff_result = grad_env.call_formula<addn::DN>(formula);
    }

    return diff_result.v;
  };

  nlopt::opt opt_problem(nlopt::AUGLAG, state.size());
  OptData opt_data(diff_state, robot, base_tf, sg, cont_vals, joint_vals, grad_env, formula);
  opt_problem.set_min_objective(grad_fn, &opt_data);
  opt_problem.set_lower_bounds(*lower_bounds);
  opt_problem.set_upper_bounds(*upper_bounds);
  opt_problem.set_xtol_rel(1e-6);
  opt_problem.set_xtol_abs(1e-6);
  nlopt::opt local_opt(nlopt::LD_LBFGS, state.size());
  local_opt.set_lower_bounds(*lower_bounds);
  local_opt.set_upper_bounds(*upper_bounds);
  local_opt.set_xtol_rel(1e-6);
  local_opt.set_xtol_abs(1e-6);
  opt_problem.set_local_optimizer(local_opt);

  // Set up Lua objects
  grad_env.setup_world<addn::DN>(formula, bindings, sg);

  // Run gradient descent
  nlopt::result opt_result = nlopt::FAILURE;
  try {
    opt_result = opt_problem.optimize(state, last_value);
  } catch (const std::runtime_error& e) {
    // dbg(e.what());
    // dbg(opt_problem.get_errmsg());
  }

  // Tear down Lua objects
  grad_env.teardown_world(formula);

  // If we succeeded, copy the result into the output
  if (opt_result > 0) {
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

  return opt_result > 0;
}
}  // namespace planner::solver
