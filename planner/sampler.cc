#include "sampler.hh"

#include <algorithm>
#include <stdexcept>

#include <fmt/ostream.h>
#include <functional>

#include <ompl/base/ScopedState.h>

#include <fplus/fplus.hpp>

#include "heuristic.hh"
#include "scenegraph.hh"
#include "solver.hh"
#include "specification.hh"

namespace planner::sampler {
// Initialize the extern'd global state
ActionLog* action_log = nullptr;
UniversePtr goal_universe_data;
Counters sample_counter{0, 0, 0, 0, 0};
std::mutex counter_mutex;

util::UniverseMap* TampSampler::universe_map = nullptr;
std::mutex TampSampler::universe_mutex;
unsigned int TampSampler::sampler_count = 0;
unsigned int TampSampler::NUM_GD_TRIES  = 0;
double TampSampler::COIN_BIAS           = 0.0;

ob::StateSamplerPtr allocTampSampler(const ob::StateSpace* space,
                                     const spec::Domain* const domain,
                                     const structures::robot::Robot* const robot) {
  return ob::StateSamplerPtr(new TampSampler(space, domain, robot));
}

void TampSampler::pose_objects(structures::scenegraph::Graph* const sg,
                               const ob::CompoundState* const robot_state,
                               const ob::CompoundState* const object_poses,
                               ob::CompoundState* const objects_state,
                               Map<Str, Transform3r>* const pose_map) {
  bool update_pose_map = pose_map != nullptr;
  if (object_poses != nullptr) {
    objects_space->copyState(objects_state, object_poses);
  }

  const auto poser = [&](const structures::scenegraph::Node* const node,
                         const bool robot_ancestor,
                         const auto& tf,
                         const auto& _) {
    if (robot_ancestor) {
      if (node->is_object) {
        auto* object_state = objects_state->as<cspace::ObjectSpace::StateType>(
        objects_space->getSubspaceIndex(node->name));
        const auto& translation = tf.translation();
        Eigen::Quaterniond rotation(tf.linear());
        object_state->setXYZ(translation.x(), translation.y(), translation.z());
        auto& object_rotation = object_state->rotation();
        object_rotation.x     = rotation.x();
        object_rotation.y     = rotation.y();
        object_rotation.z     = rotation.z();
        object_rotation.w     = rotation.w();
      }

      // log->info("{} is at\n{}", node->name, tf.matrix());
      // NOTE: This only updates the values for *manipulated* objects (as well as robot links)
      if (update_pose_map) {
        pose_map->insert_or_assign(node->name, tf);
      }
    }
  };

  double cont_vals[cspace::cont_joint_idxs.size()];
  double* joint_vals      = nullptr;
  const auto& joint_state = robot_state->as<cspace::RobotJointSpace::StateType>(joint_space_idx);
  Transform3r base_pose   = *robot->base_pose;
  state_to_pose_data(robot_state,
                     joint_state,
                     cspace::cont_joint_idxs,
                     space_->as<cspace::CompositeSpace>()->base_space_idx,
                     cont_vals,
                     &joint_vals,
                     &base_pose);

  sg->update_transforms<double>(cont_vals, joint_vals, base_pose, poser);
}

TampSampler::TampSampler(const ob::StateSpace* si,
                         const spec::Domain* const domain,
                         const structures::robot::Robot* const robot)
: ob::StateSampler(si)
, name(fmt::format("sampler-{}-{}", std::this_thread::get_id(), sampler_count++))
, objects_space_idx(space_->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::OBJECT_SPACE))
, universe_space_idx(space_->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::EQCLASS_SPACE))
, discrete_space_idx(
  space_->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::DISCRETE_SPACE))
, robot_space_idx(space_->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::ROBOT_SPACE))
, objects_space(
  space_->as<ob::CompoundStateSpace>()->as<ob::CompoundStateSpace>(cspace::OBJECT_SPACE))
, universe_space(
  space_->as<ob::CompoundStateSpace>()->as<ob::CompoundStateSpace>(cspace::EQCLASS_SPACE))
, discrete_space(
  space_->as<ob::CompoundStateSpace>()->as<ob::CompoundStateSpace>(cspace::DISCRETE_SPACE))
, robot_space(
  space_->as<ob::CompoundStateSpace>()->as<ob::CompoundStateSpace>(cspace::ROBOT_SPACE))
, joint_space_idx(robot_space->getSubspaceIndex(cspace::JOINT_SPACE))
, robot_state(robot_space->allocState()->as<ob::CompoundState>())
, robot_near_state(robot_space->allocState()->as<ob::CompoundState>())
, objects_state(objects_space->allocState()->as<ob::CompoundState>())
, universe_state(universe_space->allocState()->as<ob::CompoundState>())
, discrete_state(discrete_space->allocState()->as<ob::CompoundState>())
, start_state(space_->allocState()->as<cspace::CompositeSpace::StateType>())
, domain(domain)
, robot(robot) {
  log = spdlog::stdout_color_mt(name);
  // Make the Lua Environments
  correctness_env =
  std::make_unique<symbolic::predicate::LuaEnv<bool>>(name + "-test",
                                                      symbolic::predicate::BOOL_PRELUDE_PATH);
  gradient_env = std::make_unique<symbolic::predicate::LuaEnv<double>>(
  name + "-grad", symbolic::predicate::GRADIENT_PRELUDE_PATH);

  // Add formulae and predicates to the environments
  correctness_env->load_predicates(domain->predicates_file);
  gradient_env->load_predicates(domain->predicates_file);
  for (const auto& action : domain->actions) {
    for (auto& [formula, _] : action->precondition) {
      correctness_env->load_formula(&formula);
      gradient_env->load_formula(&formula);
      gradient_env->load_gradient(&formula, cspace::num_dims);
    }
  }

  robot_config_sampler = space_->allocSubspaceStateSampler(robot_space);
}

void TampSampler::sampleUniform(ob::State* state) {
  // Null the action pointer on state since OMPL reuses the same pointer for state
  state->as<util::HashableStateSpace::StateType>()->action = nullptr;
  // Flip a biased coin to decide if we use heuristic or normal sampling
  // ordinary_sample(state);
  const auto coin_val = (rng_.uniform01() <= COIN_BIAS);
  coin_val ? heuristic_sample(state) : ordinary_sample(state);
  ++sample_counter.uniformTotal;
}

void TampSampler::sampleUniformNear(ob::State* state, const ob::State* near, double distance) {
  throw std::logic_error("sampleUniformNear is not implemented!");
}

void TampSampler::sampleGaussian(ob::State* state, const ob::State* mean, double stdDev) {
  throw std::logic_error("sampleGaussian is not implemented!");
}

void TampSampler::ordinary_sample_with_uni(ob::State* state,
                                           const Universe* universe,
                                           const Config* config,
                                           bool copy_uni,
                                           bool update_sg_objs) {
  // Treat the state like a more useful type
  auto* full_state = state->as<cspace::CompositeSpace::StateType>();
  // Set the SG pointer
  full_state->sg = universe->sg.get();
  if (copy_uni) {
    // Copy the discrete state over
    // TODO(Wil): It would be better to construct these *once* and store the states on the
    // universe/config objects resp.
    state_of_discrete(universe->sig, full_state->as<ob::CompoundState>(universe_space_idx));
    state_of_discrete(config->sig, full_state->as<ob::CompoundState>(discrete_space_idx));
    // universe_space->copyState(full_state->as<cspace::DiscreteSpace::StateType>(universe_space_idx),
    //                           universe_state);
    // discrete_space->copyState(full_state->as<cspace::DiscreteSpace::StateType>(discrete_space_idx),
    //                           discrete_state);
  }

  // Sample a valid pose in that universe
  const Pose& pose = config->valid_poses[rng_.uniformInt(0, config->valid_poses.size() - 1)];
  full_state->object_poses = pose;

  std::unique_ptr<Map<Str, Transform3r>> pose_map_ptr = nullptr;
  if (update_sg_objs) {
    pose_map_ptr = std::make_unique<Map<Str, Transform3r>>();
    pose_map_ptr->reserve(objects_space->getSubspaceCount());
    state_to_pose_map(pose, objects_space, *pose_map_ptr);
  }

  // Sample a robot configuration
  robot_config_sampler->sampleUniform(full_state);

  // Do FK to pose manipulated objects
  pose_objects(universe->sg.get(),
               full_state->as<ob::CompoundState>(robot_space_idx),
               pose,
               full_state->as<ob::CompoundState>(objects_space_idx),
               pose_map_ptr.get());

  if (update_sg_objs) {
    universe->sg->pose_objects(*pose_map_ptr);
  }
}

void TampSampler::ordinary_sample(ob::State* state) {
  const auto& [universe, config, _] = universe_map->sample()->data;
  ordinary_sample_with_uni(state, universe, config, true, false);
  ++sample_counter.uniformNormal;
}

void TampSampler::heuristic_sample(ob::State* state) {
  // Treat the state like a more useful type
  auto* cstate = state->as<cspace::CompositeSpace::StateType>();
  // Sample a universe, config, and action
  const auto& sample = universe_map->sample();
  Universe* uni;
  Config* cf;
  Action action;
  std::tie(uni, cf, action) = sample->data;
  // Handle the case of there being no suggested actions (i.e. we're already in the goal
  // universe)
  if (action == nullptr) {
    ordinary_sample_with_uni(state, uni, cf, true, false);
    ++sample_counter.uniformNormal;
    return;
  }

  bool grad_success = false;
  // Set the universe for SG use
  gradient_env->set_universe(uni);
  correctness_env->set_universe(uni);

  // Make sure binding mappings exist in the Lua environments
  gradient_env->set_bindings(action->bindings);
  correctness_env->set_bindings(action->bindings);

  auto& precondition   = action->action->precondition;
  auto precon_branches = fplus::numbers(0, static_cast<int>(precondition.size()));
  unsigned int iters   = 0;
  Map<Str, Transform3r> pose_map;
  auto try_gradient = [&](int idx) {
    auto& formula = precondition[idx].first;
    // Check that the branch is valid in this universe/config
    for (const auto& [dim, val] : action->bound_precondition[idx]) {
      // Either a dim is kinematic
      if (fplus::map_contains(domain->eqclass_dimension_names, dim)) {
        // In which case it has to have the right value in the Universe
        if (uni->sig[dim] != val) {
          return false;
        }
        // ...or that dim is ordinary discrete, and must have the right value in the config
      } else if (cf->sig[dim - domain->num_eqclass_dims] != val) {
        return false;
      }
      // If those checks passed, this dim is fine
    }

    // Construct a random starting state, copying the universe & config only the first iteration
    // and updating the object poses in the scenegraph
    ordinary_sample_with_uni(start_state, uni, cf, iters == 0, true);
    state_to_pose_map(start_state->as<ob::CompoundState>(objects_space_idx),
                      objects_space,
                      pose_map);

    // We need gradient descent to succeed *and* the final state to actually satisfy the
    // formula
    double last_value;
    const auto solver_success = solver::gradient_solve(
    space_, *gradient_env, robot, start_state, &formula, cstate, last_value);
    if (solver_success) {
      if (last_value > 0) {
        log->warn("Non-zero value from GD: {}", last_value);
      }

      // Update object poses
      pose_objects(uni->sg.get(),
                   cstate->as<ob::CompoundState>(robot_space_idx),
                   nullptr,
                   cstate->as<cspace::ObjectSpace::StateType>(objects_space_idx),
                   &pose_map);
      const auto correctness =
      (*correctness_env)(formula.normal_fn_name, space_, cstate, robot->base_movable);
      return correctness;
    }

    log->error("Gradient descent failed!");
    return solver_success;
  };

  for (; iters < NUM_GD_TRIES; ++iters) {
    // log->info("Working on {}({}): {}", action->action->name, action->bindings, action->priority);
    if (std::none_of(precon_branches.cbegin(), precon_branches.cend(), try_gradient)) {
      log->warn("Solving for precondition for {}({}) failed; trying again.",
                action->action->name,
                action->bindings);
    } else {
      grad_success = true;
      break;
    }
  }

  if (!grad_success) {
    // We tried all the actions and none succeeded
    log->error("All the branches failed, every iteration. Had to do a uniform sample...");
    sample->update(action->update_failure());
    ordinary_sample_with_uni(state, uni, cf, true, false);
    ++sample_counter.uniformNormal;
    return;
  }

  // Gradient descent succeeded! cstate now has the new poses
  // Propagate the symbolic changes and return the state
  UniverseSig new_universe(uni->sig);
  ConfigSig new_config(cf->sig);
  apply_action(action, new_universe, new_config);

  // Add the initial pose
  auto* new_pose_state =
  objects_space->cloneState(cstate->as<cspace::ObjectSpace::StateType>(objects_space_idx))
  ->as<ob::CompoundState>();
  if (!objects_space->satisfiesBounds(new_pose_state)) {
    sample->update(action->update_failure());
    ordinary_sample_with_uni(state, uni, cf, true, false);
    ++sample_counter.uniformNormal;
    return;
    // log->critical("Created invalid state:\n");
    // objects_space->printState(new_pose_state, std::cout);
    // std::ofstream json_file("invalid_state.json");
    // json output;
    // for (const auto& [name, pose] : pose_map) {
    //   Eigen::Quaterniond rotation(pose.linear());
    //   output.emplace_back(json{
    //   {"name", name},
    //   {"translation", {pose.translation().x(), pose.translation().y(), pose.translation().z()}},
    //   {"rotation", {rotation.x(), rotation.y(), rotation.z(), rotation.w()}}});
    // }
    //
    // json_file << output;
    // json_file.close();
    // throw std::runtime_error("Well fuck");
  }

  // Update the maps
  auto state_copy = space_->cloneState(state);
  action_log->emplace(state_copy, action.get());

  const auto& [new_uni_data, new_cf_data] =
  universe_map->add_transition({uni->sig, cf->sig},
                               {new_universe, new_config},
                               state_copy->as<util::HashableStateSpace::StateType>(),
                               action.get());
  // log->info("Taking action {}({}) succeeded", action->action->name, action->bindings);
  if (new_uni_data->sg == nullptr) {
    auto new_sg              = std::make_shared<structures::scenegraph::Graph>(*uni->sg);
    const auto& universe_sig = uni->sig;
    for (size_t i = 0; i < new_universe.size(); ++i) {
      domain->handle_kin_effect(i, pose_map, new_sg.get(), universe_sig[i], new_universe[i]);
    }

    universe_map->graph.at(new_universe)->sg = std::move(new_sg);
  }

  const auto* pose_ptr = new_cf_data->valid_poses.emplace_back(new_pose_state);

  // Add the new pose as valid to the old universe
  // uni->valid_poses.emplace_back(new_pose_state);

  // Add the valuedata and pose pointers to the state
  cstate->action       = sample;
  cstate->object_poses = pose_ptr;
  ++sample_counter.heuristic;
}

inline void TampSampler::apply_action(const Action& action,
                                      UniverseSig& universe,
                                      ConfigSig& result_config) const {
  for (const auto& [dim, val] : action->bound_effect) {
    if (dim < domain->num_eqclass_dims) {
      universe.set(dim, val);
    } else {
      result_config.set(dim - domain->num_eqclass_dims, val);
    }
  }
}

void TampSampler::cleanup() const {
  correctness_env->cleanup();
  gradient_env->cleanup();
}
}  // namespace planner::sampler
