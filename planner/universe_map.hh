#pragma once
#ifndef UNIVERSE_MAP_HH
#define UNIVERSE_MAP_HH

#include "common.hh"

#include <memory>

#include <boost/dynamic_bitset.hpp>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include <tsl/robin_map.h>
#include <tsl/robin_set.h>

#include "hash_helpers.hh"
#include "hashable_statespace.hh"
#include "predicate.hh"
#include "signatures.hh"
#include "specification.hh"

namespace planner::util {
namespace spec = input::specification;
namespace ob   = ompl::base;

using Pose             = const ob::CompoundState*;
using SceneGraph       = std::shared_ptr<structures::scenegraph::Graph>;
using TransitionStates = tsl::robin_set<HashableStateSpace::StateType>;

extern double GOAL_WEIGHT;

struct Transition {
  TransitionStates known_states;
  Vec<symbolic::heuristic::PrioritizedAction*> actions;
};

using TransitionPtr = std::shared_ptr<Transition>;

struct Config {
  Config() = default;
  Config(Config&& other) noexcept
  : sig(std::move(other.sig))
  , viable_transitions(std::move(other.viable_transitions))
  , state_transitions(std::move(other.state_transitions))
  , valid_poses(std::move(other.valid_poses))
  , actually_reached(other.actually_reached) {}
  ConfigSig sig;
  tsl::robin_map<std::pair<UniverseSig, ConfigSig>, TransitionPtr> viable_transitions;
  // We keep this reverse index to make it more efficient to check what universe/config is
  // reachable when a state is added to the tree
  // NOTE/TODO(Wil): If the same state is used for multiple transitions in the same universe &
  // config, then this could cause bugs
  tsl::robin_map<HashableStateSpace::StateType, std::pair<UniverseSig, ConfigSig>>
  state_transitions;
  Vec<Pose> valid_poses;
  bool actually_reached = false;
};

using ConfigPtr = std::shared_ptr<Config>;

struct Universe {
  Universe() = default;
  Universe(Universe&& other) noexcept
  : sig(std::move(other.sig))
  , configs(std::move(other.configs))
  , sg(std::move(other.sg))
  , actually_reached(other.actually_reached) {}
  // std::mutex uni_mutex;
  UniverseSig sig;
  tsl::robin_map<ConfigSig, ConfigPtr> configs;
  SceneGraph sg;
  bool actually_reached = false;
};

using UniversePtr = std::shared_ptr<Universe>;

struct UniverseMap {
  UniverseMap(const spec::Initial& init_atoms,
              HashableStateSpace::StateType* init_state,
              SceneGraph init_sg,
              spec::Goal* goal,
              spec::Domain* domain,
              ob::CompoundStateSpace* objects_space,
              ob::StateSpace* space_,
              unsigned int eqclass_space_idx,
              unsigned int discrete_space_idx,
              unsigned int objects_space_idx,
              bool robot_base_movable);

  std::pair<Universe*, Config*>
  add_transition(const std::pair<const UniverseSig&, const ConfigSig&>& from,
                 const std::pair<const UniverseSig&, const ConfigSig&>& to,
                 HashableStateSpace::StateType* at_state,
                 symbolic::heuristic::PrioritizedAction* with_action);
  bool check_valid_transition(const HashableStateSpace::StateType* const s1,
                              const HashableStateSpace::StateType* const s2);
  bool check_precondition(const HashableStateSpace::StateType* const state,
                          symbolic::heuristic::PrioritizedAction* action) const;
  void added_state(HashableStateSpace::StateType* state);
  ActionDistribution::ValueData* const sample();
  std::pair<Universe*, Config*>
  get_data(const std::pair<const UniverseSig&, const ConfigSig&>& sig);

  void clear(const spec::Initial& init_atoms,
             HashableStateSpace::StateType* init_state,
             SceneGraph init_sg,
             spec::Domain* domain,
             spec::Goal* goal,
             ob::CompoundStateSpace* objects_space,
             unsigned int objects_space_idx);
  void add_actions(Universe* uni, Config* cf);

  tsl::robin_map<UniverseSig, UniversePtr> graph;
  ActionDistribution distribution;
  const unsigned int num_eqclass_dims;
  const unsigned int num_discrete_dims;
  const unsigned int eqclass_space_idx;
  const unsigned int discrete_space_idx;
  const bool base_movable;
  const ob::StateSpace* const space_;
  ompl::RNG rng;
  std::unique_ptr<symbolic::predicate::LuaEnv<double>> predicate_env;
  std::unique_ptr<symbolic::heuristic::FFLikeHeuristic> heuristic;
};
}  // namespace planner::util
#endif /* end of include guard */
