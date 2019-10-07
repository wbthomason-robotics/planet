#include "goal.hh"

#include <iostream>
#include <random>
#include <utility>

#include <fmt/ostream.h>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include "fplus/fplus.hpp"

#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>

#include "planner_utils.hh"
#include "sampler.hh"
#include "solver.hh"

namespace planner {
namespace goal {
  namespace {
    auto log = spdlog::stdout_color_mt("goal");
  }
  unsigned int MAX_SAMPLES = 0;
  // std::shared_ptr<ob::GoalLazySamples>
  // make_lazy_sampler(const ob::SpaceInformationPtr& si,
  //                   const spec::Domain* const domain,
  //                   const structures::robot::Robot* const robot,
  //                   spec::Goal& goal_spec) {
  //   log->debug("Constructing sampler function");
  //
  //   auto eval_env =
  //   std::make_shared<pred::LuaEnv<double>>("goal-grad",
  //                                          symbolic::predicate::GRADIENT_PRELUDE_PATH);
  //   eval_env->load_predicates(domain->predicates_file);
  //   for (auto& [formula, _] : goal_spec) {
  //     eval_env->load_formula(&formula);
  //     eval_env->load_gradient(&formula, cspace::num_dims);
  //   }
  //
  //   // NOTE Same errors with the distributions being unique_ptrs (also for the random_device,
  //   but
  //   // that makes sense as it is explicitly neither copyable nor movable)
  //   auto rando_d    = std::make_shared<std::random_device>();
  //   auto goal_rando = std::make_shared<std::uniform_int_distribution<>>(0, goal_spec.size());
  //
  //   ob::GoalSamplingFn sampler = [rando_d    = std::move(rando_d),
  //                                 goal_rando = std::move(goal_rando),
  //                                 eval_env   = std::move(eval_env),
  //                                 full_space =
  //                                 si->getStateSpace()->as<cspace::CompositeSpace>(), domain,
  //                                 robot,
  //                                 &goal_spec](const ob::GoalLazySamples* gls, ob::State* result)
  //                                 {
  //     auto full_state       = full_space->allocState()->as<cspace::CompositeSpace::StateType>();
  //     auto obj_space        = full_space->getSubspace(cspace::OBJECT_SPACE);
  //     auto obj_idx          = full_space->getSubspaceIndex(cspace::OBJECT_SPACE);
  //     auto obj_sampler      = full_space->allocSubspaceStateSampler(obj_space.get());
  //     auto robot_space      = full_space->getSubspace(cspace::ROBOT_SPACE);
  //     auto robot_space_idx  = full_space->getSubspaceIndex(cspace::ROBOT_SPACE);
  //     auto robot_sampler    = full_space->allocSubspaceStateSampler(robot_space.get());
  //     auto eqclass_space    = full_space->getSubspace(cspace::EQCLASS_SPACE);
  //     auto eqclass_idx      = full_space->getSubspaceIndex(cspace::EQCLASS_SPACE);
  //     auto eqclass_sampler  = full_space->allocSubspaceStateSampler(eqclass_space.get());
  //     auto discrete_space   = full_space->getSubspace(cspace::DISCRETE_SPACE);
  //     auto discrete_idx     = full_space->getSubspaceIndex(cspace::DISCRETE_SPACE);
  //     auto discrete_sampler = full_space->allocSubspaceStateSampler(discrete_space.get());
  //
  //     bool continue_sampling = false;
  //     for (int i = 0; i < 100; ++i) {
  //       // Try to find a solution
  //       // Initialize everything to random
  //       eqclass_sampler->sampleUniform(full_state);
  //       discrete_sampler->sampleUniform(full_state);
  //       obj_sampler->sampleUniform(full_state);
  //       robot_sampler->sampleUniform(full_state);
  //
  //       auto eqclass_state =
  //       eqclass_space->allocState()->as<ob::CompoundStateSpace::StateType>(); auto
  //       discrete_state = discrete_space->allocState()->as<ob::CompoundStateSpace::StateType>();
  //
  //       // Sample a random branch of the goal
  //       spec::Configuration goal_variant =
  //       fplus::random_element((*goal_rando)(*rando_d), goal_spec);
  //
  //       // Put the discrete parts of the sampled state into the full state
  //       for (const auto& disc_sub : eqclass_space->as<ob::CompoundStateSpace>()->getSubspaces())
  //       {
  //         const int idx =
  //         eqclass_space->as<ob::CompoundStateSpace>()->getSubspaceIndex(disc_sub->getName());
  //         if (fplus::map_contains(goal_variant.second, disc_sub->getName())) {
  //           // We manually specify 1 and 0 here because I don't trust C++'s type coercion to not
  //           do
  //           // something subtly wrong
  //           eqclass_state->as<ob::DiscreteStateSpace::StateType>(idx)->value =
  //           goal_variant.second[disc_sub->getName()] ? 1 : 0;
  //         }
  //       }
  //
  //       for (const auto& disc_sub :
  //       discrete_space->as<ob::CompoundStateSpace>()->getSubspaces()) {
  //         const int idx =
  //         discrete_space->as<ob::CompoundStateSpace>()->getSubspaceIndex(disc_sub->getName());
  //         if (fplus::map_contains(goal_variant.second, disc_sub->getName())) {
  //           // We manually specify 1 and 0 here because I don't trust C++'s type coercion to not
  //           do
  //           // something subtly wrong
  //           discrete_state->as<ob::DiscreteStateSpace::StateType>(idx)->value =
  //           goal_variant.second[disc_sub->getName()] ? 1 : 0;
  //         }
  //       }
  //
  //       eqclass_space->copyState(full_state->as<ob::CompoundStateSpace::StateType>(eqclass_idx),
  //                                eqclass_state);
  //       discrete_space->copyState(full_state->as<ob::CompoundStateSpace::StateType>(discrete_idx),
  //                                 discrete_state);
  //
  //       // Free allocated states
  //       eqclass_space->freeState(eqclass_state);
  //       discrete_space->freeState(discrete_state);
  //
  //       // Set the universe for SG use
  //       if (sampler::goal_universe_data.get() == nullptr) {
  //         log->critical("Goal universe pointer is null! You're about to segfault!");
  //       }
  //
  //       eval_env->set_universe(sampler::goal_universe_data.get());
  //
  //       // Run gradient descent
  //       bool grad_result = solver::gradient_solve(
  //       full_space, *eval_env, robot, full_state, &(goal_variant.first), result);
  //       if (grad_result) {
  //         // NOTE/TODO(Wil): Could get rid of this duplication by putting this code into solver
  //         double cont_vals[cspace::cont_joint_idxs.size()];
  //         const auto& robot_state =
  //         result->as<ob::CompoundStateSpace::StateType>()->as<ob::CompoundStateSpace::StateType>(
  //         robot_space_idx);
  //         for (size_t j = 0; j < cspace::cont_joint_idxs.size(); ++j) {
  //           const auto idx = cspace::cont_joint_idxs[j];
  //           cont_vals[j]   = robot_state->as<ob::SO2StateSpace::StateType>(idx)->value;
  //         }
  //
  //         const auto& joint_state = robot_state->as<ob::RealVectorStateSpace::StateType>(
  //         robot_space->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::JOINT_SPACE));
  //         double* joint_vals  = joint_state->values;
  //         auto new_pose_state =
  //         obj_space->allocState()->as<ob::CompoundStateSpace::StateType>();
  //
  //         Transform3r base_tf = *(robot->base_pose.get());
  //         if (robot->base_movable) {
  //           const auto& base_state = robot_state->as<cspace::RobotBaseSpace::StateType>(
  //           robot_space->as<ob::CompoundStateSpace>()->getSubspaceIndex(cspace::BASE_SPACE));
  //
  //           base_tf.translation().x() = base_state->getX();
  //           base_tf.translation().y() = base_state->getY();
  //           base_tf.translation().z() = base_state->getZ();
  //
  //           const ob::SO2StateSpace::StateType& base_rotation = base_state->rotation();
  //           base_tf.linear() =
  //           Eigen::AngleAxisd(base_rotation.value, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  //         }
  //
  //         const auto make_poses = [&](const structures::scenegraph::Node* const node,
  //                                     const bool robot_ancestor,
  //                                     const auto& tf,
  //                                     const auto& coll_tf) {
  //           if (obj_space->as<ob::CompoundStateSpace>()->hasSubspace(node->name)) {
  //             auto obj_pose = new_pose_state->as<cspace::ObjectSpace::StateType>(
  //             obj_space->as<ob::CompoundStateSpace>()->getSubspaceIndex(node->name));
  //             obj_pose->setXYZ(tf.translation().x(), tf.translation().y(),
  //             tf.translation().z()); Eigen::Quaterniond rotation(tf.linear()); auto&
  //             obj_pose_rot = obj_pose->rotation(); obj_pose_rot.x     = rotation.x();
  //             obj_pose_rot.y     = rotation.y();
  //             obj_pose_rot.z     = rotation.z();
  //             obj_pose_rot.w     = rotation.w();
  //           }
  //         };
  //
  //         // TODO(Wil): Uncomment the below when goal sampling is fixed
  //         // sampler::goal_universe_data->sg->pose_roots(poses, base_tf);
  //         sampler::goal_universe_data->sg->update_transforms<double>(
  //         cont_vals, joint_vals, base_tf, make_poses);
  //
  //         util::ConfigSig config(domain->num_symbolic_dims, 0);
  //         for (const auto& [dim, val] : goal_variant.second) {
  //           config.set(domain->discrete_dimension_ids.at(dim), val);
  //         }
  //
  //         auto& goal_poses = sampler::goal_universe_data->valid_poses;
  //         auto pose_copy   = obj_space->allocState()->as<ob::CompoundState>();
  //         obj_space->copyState(pose_copy, new_pose_state);
  //         goal_poses.emplace_back(pose_copy);
  //
  //         continue_sampling = true;
  //         obj_space->copyState(result->as<ob::CompoundState>()->as<ob::CompoundState>(obj_idx),
  //                              new_pose_state);
  //         obj_space->freeState(new_pose_state);
  //         break;
  //       }
  //     }
  //
  //     // Free allocated states
  //     full_space->freeState(full_state);
  //
  //     return continue_sampling && gls->maxSampleCount() < MAX_SAMPLES;
  //   };
  //
  //   return std::make_unique<ob::GoalLazySamples>(si, sampler);
  // }

  util::UniverseMap* CompositeGoal::universe_map = nullptr;

  bool CompositeGoal::discrete_satisfied(const util::UniverseSig& uni_sig,
                                         const util::ConfigSig& config_sig,
                                         const Map<Str, bool>& branch_config) const {
    bool satisfied = true;
    for (const auto& [dim, val] : branch_config) {
      bool state_val = false;
      if (fplus::map_contains(domain->eqclass_dimension_ids, dim)) {
        state_val = uni_sig[(domain->eqclass_dimension_ids.at(dim))];
      } else {
        state_val = config_sig[(domain->discrete_dimension_ids.at(dim))];
      }

      if (state_val != val) {
        satisfied = false;
        break;
      }
    }

    return satisfied;
  }

  bool CompositeGoal::isSatisfied(const ob::State* state, double* distance) const {
    if (distance != nullptr) {
      *distance = std::numeric_limits<double>::infinity();
    }

    return isSatisfied(state);
  }

  bool CompositeGoal::isSatisfied(const ob::State* state) const {
    const auto cstate          = state->as<cspace::CompositeSpace::StateType>();
    const auto& eqclass_state  = cstate->as<ob::CompoundState>(eqclass_space_idx);
    const auto& discrete_state = cstate->as<ob::CompoundState>(discrete_space_idx);
    const auto& objects_state  = cstate->as<ob::CompoundState>(objects_space_idx);
    const auto& uni_sig        = util::discrete_of_state(eqclass_state, num_eqclass_dims);
    const auto& config_sig     = util::discrete_of_state(discrete_state, num_discrete_dims);
    Map<Str, Transform3r> pose_map;
    pose_map.reserve(objects_space->getSubspaceCount());
    util::state_to_pose_map(objects_state, objects_space, pose_map);
    const auto& uni_data = universe_map->graph.at(uni_sig);
    uni_data->sg->pose_objects(pose_map);

    for (const auto& [formula, branch_config] : *goal) {
      if (discrete_satisfied(uni_sig, config_sig, branch_config)) {
        log->info("Goal branch {} is satisfied by ({}, {})", branch_config, uni_sig, config_sig);
        log->info("Now checking formula {}", formula.normal_def);
        correctness_env->set_universe(uni_data.get());
        const auto correctness =
        (*correctness_env)(formula.normal_fn_name, space, cstate, robot->base_movable);
        if (correctness) {
          log->info("Formula satisfied!");
          return true;
        }
      }
    }

    return false;
  }
}  // namespace goal
}  // namespace planner
