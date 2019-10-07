#pragma once
#ifndef PLANNER_UTILS_HH
#define PLANNER_UTILS_HH

#include "common.hh"

#include <memory>
#include <utility>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <boost/dynamic_bitset.hpp>

#include "discrete_distribution.hh"
#include "hash_helpers.hh"
#include "hashable_statespace.hh"
#include "heuristic.hh"
#include "predicate.hh"
#include "scenegraph.hh"
#include "signatures.hh"
#include "specification.hh"
#include "universe_map.hh"

namespace planner::util {
namespace ob = ompl::base;

using ActionLog = Map<ob::State*, const symbolic::heuristic::PrioritizedAction*>;
extern ActionLog* action_log;

DiscreteSig discrete_of_state(const ob::CompoundState* const state, const int num_dims);
void state_of_discrete(const DiscreteSig& discrete, const ob::CompoundState* state);
void state_to_pose_data(const ob::CompoundState* robot_state,
                        const ob::RealVectorStateSpace::StateType* joint_state,
                        const Vec<int>& cont_joint_idxs,
                        int base_space_idx,
                        double* const cont_vals,
                        double** joint_vals,
                        Transform3r* base_tf);
void state_to_pose_map(const ob::CompoundState* objects_state,
                       const ob::CompoundStateSpace* objects_space,
                       Map<Str, Transform3r>& pose_map);
}  // namespace planner::util
#endif
