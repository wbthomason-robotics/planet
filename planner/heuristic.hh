#pragma once
#ifndef HEURISTIC_HH
#define HEURISTIC_HH
#include <tsl/robin_map.h>
#include <tsl/robin_set.h>

#include <boost/dynamic_bitset.hpp>
#include <memory>
#include <optional>
#include <utility>

#include "common.hh"
#include "hash_helpers.hh"
#include "specification.hh"

namespace symbolic::heuristic {
namespace spec = input::specification;
using State    = std::unordered_multimap<spec::DimId, bool>;
extern double SUCCESS_SCALE;

struct GroundAction {
  std::shared_ptr<spec::Action> action;
  Vec<Map<spec::DimId, bool>> bound_precondition;
  Map<spec::DimId, bool> bound_effect;
  Map<Str, Str> bindings;
  GroundAction(const std::shared_ptr<spec::Action>& action,
               Map<Str, Str> bindings,
               const spec::Domain* const domain);
};

struct PrioritizedAction {
  explicit PrioritizedAction(const GroundAction* const action)
  : ground_action(action)
  , action(ground_action->action.get())
  , bound_precondition(ground_action->bound_precondition)
  , bound_effect(ground_action->bound_effect)
  , bindings(ground_action->bindings) {}
  const GroundAction* const ground_action;
  // HACK: This field duplication is really dumb but I'm in a rush
  // TODO(Wil): Think of a better design
  spec::Action* const action;
  const Vec<Map<spec::DimId, bool>>& bound_precondition;
  const Map<spec::DimId, bool>& bound_effect;
  const Map<Str, Str>& bindings;
  double priority        = 0.0;
  unsigned int failures  = 0;
  unsigned int successes = 0;
  double update_failure() {
    ++failures;
    return update();
  }

  double update_success() {
    ++successes;
    return update();
  }

 private:
  double update() {
    double failure_term = static_cast<double>(failures);
    double success_term = SUCCESS_SCALE * static_cast<double>(successes);

    return priority / (1.0 + failure_term + success_term);
  }
};

using GroundActionPtr      = std::shared_ptr<GroundAction>;
using PrioritizedActionPtr = std::shared_ptr<PrioritizedAction>;

struct Heuristic {
  virtual Vec<PrioritizedActionPtr> suggest(const boost::dynamic_bitset<>& state) = 0;
  virtual ~Heuristic()                                                            = default;
};

struct FFLikeHeuristic : public Heuristic {
  FFLikeHeuristic(const Vec<std::shared_ptr<spec::Action>>& actions,
                  const Map<Str, Vec<Str>>& typed_objects,
                  const spec::Goal& goal,
                  const spec::Domain* const domain)
  : goal(goal), domain(domain) {
    ground_actions(actions, typed_objects);
  };

  struct GraphLayer {
    State state;
    Set<GroundAction*> actions;
  };

  struct Graph {
    Vec<GraphLayer> layers;
    Vec<size_t> feasible_goal_branches;
  };

  Vec<PrioritizedActionPtr> suggest(const boost::dynamic_bitset<>& state) override;

 private:
  void apply_action(boost::dynamic_bitset<>& state, const PrioritizedActionPtr& action);
  void ground_actions(const Vec<std::shared_ptr<spec::Action>>& actions,
                      const Map<Str, Vec<Str>>& typed_objects);
  [[nodiscard]] Vec<size_t> goal_met(const spec::Goal& goal, const State& state) const;
  std::optional<Graph> makeGraph(const boost::dynamic_bitset<>& state);
  std::pair<Set<GroundAction*>, size_t> relaxed_plan(const Graph& graph);
  double estimate_distances(const boost::dynamic_bitset<>& state,
                            const Vec<PrioritizedActionPtr>& actions);

  const spec::Goal& goal;
  Vec<GroundActionPtr> grounded_actions;
  const spec::Domain* const domain;
  struct ActionData {
    Vec<PrioritizedActionPtr> helpful_actions;
    Vec<PrioritizedActionPtr> other_actions;
    Vec<PrioritizedActionPtr> all_actions;
    bool distances_estimated;
  };

  // NOTE: This is a Map to get around annoying issues with iterator invalidation
  Map<boost::dynamic_bitset<>, ActionData> action_cache;
  tsl::robin_map<boost::dynamic_bitset<>, unsigned int> distance_cache;

  std::pair<Map<boost::dynamic_bitset<>, ActionData>::iterator, size_t>
  update_caches(const FFLikeHeuristic::Graph& graph, const boost::dynamic_bitset<>& action_state);
};

bool precondition_satisfiable(GroundAction* action, const State& state);
bool meaningful_effect(GroundAction* action, const State& state);
void add_new_atoms(State* next_state, const GroundAction* action);

}  // namespace symbolic::heuristic
#endif
