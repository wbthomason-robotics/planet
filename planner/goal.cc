#include <fmt/ostream.h>

#include <iostream>
#include <random>
#include <utility>

#include "goal.hh"

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

namespace planner::goal {
namespace {
  auto log = spdlog::stdout_color_st("goal");
}

unsigned int MAX_SAMPLES = 0;

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
  // const auto& objects_state  = cstate->as<ob::CompoundState>(objects_space_idx);
  const auto& uni_sig        = util::discrete_of_state(eqclass_state, num_eqclass_dims);
  const auto& config_sig     = util::discrete_of_state(discrete_state, num_discrete_dims);
  // Map<Str, Transform3r> pose_map;
  // pose_map.reserve(objects_space->getSubspaceCount());
  // util::state_to_pose_map(objects_state, objects_space, pose_map);
  const auto& uni_data = universe_map->graph.at(uni_sig);
  // uni_data->sg->pose_objects(pose_map);

  for (const auto& [formula, branch_config] : *goal) {
    if (discrete_satisfied(uni_sig, config_sig, branch_config)) {
      log->info("Goal branch {} is satisfied by ({}, {})", branch_config, uni_sig, config_sig);
      log->info("Now checking formula {}", formula.normal_def);
      // For the goal formula, all bindings are top-level objects. We make this to play nice with
      // the expected call form for an action formula
      const auto& bindings = fplus::create_unordered_map(formula.bindings, formula.bindings);
      if (correctness_env->call(
          formula, bindings, uni_data->sg.get(), state, space, robot->base_movable)) {
        log->info("Formula satisfied!");
        return true;
      }
    }
  }

  return false;
}
}  // namespace planner::goal
