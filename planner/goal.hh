#pragma once
#ifndef GOAL_HH
#define GOAL_HH

#include "common.hh"

#include <memory>

#include <ompl/base/Goal.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalLazySamples.h>

#include "cspace.hh"
#include "heuristic.hh"
#include "planner_utils.hh"
#include "predicate.hh"
#include "robot.hh"
#include "specification.hh"

namespace planner::goal {
namespace ob        = ompl::base;
namespace spec      = input::specification;
namespace pred      = symbolic::predicate;
namespace heuristic = symbolic::heuristic;

extern unsigned int MAX_SAMPLES;

// std::shared_ptr<ob::GoalLazySamples> make_lazy_sampler(const ob::SpaceInformationPtr& si,
//                                                        const spec::Domain* const domain,
//                                                        const structures::robot::Robot* const robot,
//                                                        Vec<spec::Configuration>& goal_spec);

struct CompositeGoal : public ob::Goal {
  CompositeGoal(const ob::SpaceInformationPtr& si,
                const spec::Domain* const domain,
                spec::Goal* const goal,
                const structures::robot::Robot* const robot)
  : ob::Goal(si)
  , space(si_->getStateSpace()->as<ob::CompoundStateSpace>())
  , objects_space(space->getSubspace(cspace::OBJECT_SPACE)->as<ob::CompoundStateSpace>())
  , objects_space_idx(space->getSubspaceIndex(cspace::OBJECT_SPACE))
  , domain(domain)
  , goal(goal)
  , robot(robot)
  , eqclass_space_idx(space->getSubspaceIndex(cspace::EQCLASS_SPACE))
  , num_eqclass_dims(
    space->getSubspace(eqclass_space_idx)->as<ob::CompoundStateSpace>()->getSubspaceCount())
  , discrete_space_idx(space->getSubspaceIndex(cspace::DISCRETE_SPACE))
  , num_discrete_dims(
    space->getSubspace(discrete_space_idx)->as<ob::CompoundStateSpace>()->getSubspaceCount()) {
    correctness_env =
    std::make_unique<symbolic::predicate::LuaEnv<bool>>("goal-test",
                                                        symbolic::predicate::BOOL_PRELUDE_PATH);
    correctness_env->load_predicates(domain->predicates_file);
    for (const auto& action : domain->actions) {
      for (auto& [formula, _] : action->precondition) {
        correctness_env->load_formula(&formula);
      }
    }

    for (auto& [formula, _] : *goal) {
      correctness_env->load_formula(&formula);
    }
  }

  virtual bool isSatisfied(const ob::State* state) const override;
  virtual bool isSatisfied(const ob::State* state, double* distance) const override;
  static util::UniverseMap* universe_map;

 protected:
  bool discrete_satisfied(const util::UniverseSig& uni_sig,
                          const util::ConfigSig& config_sig,
                          const Map<Str, bool>& branch_config) const;
  const ob::CompoundStateSpace* space;
  const ob::CompoundStateSpace* objects_space;
  const int objects_space_idx;
  const spec::Domain* const domain;
  const spec::Goal* const goal;
  const structures::robot::Robot* const robot;
  std::unique_ptr<symbolic::predicate::LuaEnv<bool>> correctness_env;

  const int eqclass_space_idx;
  const int num_eqclass_dims;
  const int discrete_space_idx;
  const int num_discrete_dims;
};
}  // namespace planner::goal
#endif
