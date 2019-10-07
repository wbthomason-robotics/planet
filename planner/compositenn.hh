#pragma once
#ifndef COMPOSITE_NN_HH
#define COMPOSITE_NN_HH
#include "common.hh"

#include <limits>
#include <stdexcept>
#include <utility>

#include <ompl/base/State.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <tsl/robin_map.h>
#include <boost/dynamic_bitset.hpp>

#include "cspace.hh"
#include "hash_helpers.hh"
#include "planner_utils.hh"
#include "signatures.hh"
#include "universe_map.hh"

namespace planner::nn {
namespace ob = ompl::base;
template <typename MotionType>
struct CompositeNearestNeighbors : public ompl::NearestNeighbors<MotionType> {
  explicit CompositeNearestNeighbors(util::UniverseMap* const uni_map)
  : ompl::NearestNeighbors<MotionType>(), uni_map(uni_map) {}

  ~CompositeNearestNeighbors() override = default;

  constexpr bool reportsSortedResults() const override { return true; }

  void clear() override { local_nns.clear(); }

  std::size_t size() const override { return counter; }

  void add(const MotionType& data) override {
    const auto& child_signature = get_full_signature(data->state);
    auto [nn_it, is_new]        = local_nns.try_emplace(
    child_signature, std::make_unique<ompl::NearestNeighborsGNATNoThreadSafety<MotionType>>());
    if (is_new) {
      nn_it.value()->setDistanceFunction(this->distFun_);
    }

    nn_it.value()->add(data);
    const auto& [uni_data, cf_data] = uni_map->get_data(child_signature);
    const auto& transition_sig_it   = cf_data->state_transitions.find(
    *data->state->template as<cspace::CompositeSpace::StateType>());
    if (transition_sig_it != cf_data->state_transitions.end()) {
      auto [transitions_it, _] =
      transition_motions.try_emplace(transition_sig_it->second, Vec<MotionType>{});
      transitions_it.value().push_back(data);
    }

    ++counter;
  }

  bool remove(const MotionType& data) override {
    const auto& signature = get_full_signature(data->state);
    auto nn_it            = local_nns.find(signature);
    if (nn_it != local_nns.end()) {
      nn_it.value()->remove(data);
      --counter;
      return true;
    }

    return false;
  }

  void list(Vec<MotionType>& data) const override {
    for (const auto& [_, local_nn] : local_nns) {
      Vec<MotionType> local_data;
      local_nn->list(local_data);
      data.insert(data.end(), local_data.begin(), local_data.end());
    }
  }

  MotionType nearest(const MotionType& data) const override {
    const auto& signature      = get_full_signature(data->state);
    const auto& local_nn_it    = local_nns.find(signature);
    const auto& transitions_it = transition_motions.find(signature);
    auto minimum_dist          = std::numeric_limits<double>::infinity();
    MotionType result;
    if (local_nn_it != local_nns.end() && local_nn_it->second->size() > 0) {
      result = local_nn_it->second->nearest(data);
      // Needing to re-run the distance function like this is annoying, but there's no way to get a
      // distance value from the secondary NN structure directly
      minimum_dist = this->distFun_(result, data);
    }

    if (transitions_it != transition_motions.end()) {
      const auto& transitions = transitions_it->second;
      for (const auto& transition : transitions) {
        const double candidate_distance = this->distFun_(transition, data);
        if (candidate_distance < minimum_dist) {
          result       = transition;
          minimum_dist = candidate_distance;
        }
      }
    }

    return result;
  }

  void nearestK(const MotionType& data, std::size_t k, Vec<MotionType>& nbh) const override {
    throw std::logic_error("nearestK isn't implemented yet!");
  }

  void nearestR(const MotionType& data, double radius, Vec<MotionType>& nbh) const override {
    throw std::logic_error("nearestR isn't implemented yet!");
  }

 private:
  std::pair<util::DiscreteSig, util::DiscreteSig>
  get_full_signature(const ob::State* const state) const {
    util::DiscreteSig uni_sig(uni_map->num_eqclass_dims, 0);
    util::DiscreteSig cf_sig(uni_map->num_discrete_dims, 0);
    const auto* uni_state =
    state->as<ob::CompoundState>()->as<ob::CompoundState>(uni_map->eqclass_space_idx);
    const auto* cf_state =
    state->as<ob::CompoundState>()->as<ob::CompoundState>(uni_map->discrete_space_idx);
    for (int i = 0; i < uni_sig.size(); ++i) {
      uni_sig.set(i, uni_state->as<cspace::DiscreteSpace::StateType>(i)->value == 1);
    }

    for (int i = 0; i < cf_sig.size(); ++i) {
      cf_sig.set(i, cf_state->as<cspace::DiscreteSpace::StateType>(i)->value == 1);
    }

    return {uni_sig, cf_sig};
  }

  util::UniverseMap* const uni_map;
  tsl::robin_map<std::pair<util::DiscreteSig, util::DiscreteSig>,
                 std::unique_ptr<ompl::NearestNeighborsGNATNoThreadSafety<MotionType>>>
  local_nns;
  tsl::robin_map<std::pair<util::DiscreteSig, util::DiscreteSig>, Vec<MotionType>>
  transition_motions;
  std::size_t counter = 0;
};
}  // namespace planner::nn
#endif
