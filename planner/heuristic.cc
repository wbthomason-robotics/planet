#include "heuristic.hh"

#include <iostream>
#include <list>

#include "fmt/ostream.h"
// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include "fplus/fplus.hpp"

namespace symbolic::heuristic {
namespace {
  auto log      = spdlog::stdout_color_mt("heuristic");
  namespace fwd = fplus::fwd;

  Str bind_dim_name(const Vec<Str>& names, const Map<Str, Str>& bindings) {
    return fplus::head(names) + "_" +
           fwd::apply(names,
                      fwd::tail(),
                      fwd::transform([&](const auto& name) {
                        if (fplus::map_contains(bindings, name)) {
                          return bindings.at(name);
                        }

                        return name;
                      }),
                      fwd::join(Str("_")));
  }
}  // namespace

double SUCCESS_SCALE;

GroundAction::GroundAction(const std::shared_ptr<spec::Action>& action,
                           Map<Str, Str> bindings,
                           const spec::Domain* const domain) {
  this->action   = action;
  this->bindings = std::move(bindings);

  // Construct the grounded precondition
  const auto& precondition = action->precondition;
  for (const auto& [_, branch] : precondition) {
    Map<spec::DimId, bool> bound_branch;
    for (const auto& dim : branch) {
      const auto dim_name = bind_dim_name(dim.first, this->bindings);
      spec::DimId dim_id;
      int offset = 0;
      if (fplus::map_contains(domain->eqclass_dimension_ids, dim_name)) {
        dim_id = domain->eqclass_dimension_ids.at(dim_name);
      } else {
        dim_id = domain->discrete_dimension_ids.at(dim_name);
        offset = domain->num_eqclass_dims;
      }

      bound_branch[dim_id + offset] = dim.second;
    }

    bound_precondition.push_back(bound_branch);
  }

  // Construct the grounded effect
  for (const auto& [names, value] : action->effect) {
    const auto dim_name = bind_dim_name(names, this->bindings);
    spec::DimId dim_id;
    int offset = 0;
    if (fplus::map_contains(domain->eqclass_dimension_ids, dim_name)) {
      dim_id = domain->eqclass_dimension_ids.at(dim_name);
    } else {
      dim_id = domain->discrete_dimension_ids.at(dim_name);
      offset = domain->num_eqclass_dims;
    }

    bound_effect[dim_id + offset] = value;
  }
}

std::optional<FFLikeHeuristic::Graph>
FFLikeHeuristic::makeGraph(const boost::dynamic_bitset<>& state) {
  // We haven't made a graph for this state yet
  // Translate from a configuration to a State
  State initial_atoms;
  for (size_t i = 0; i < state.size(); ++i) {
    initial_atoms.insert({i, state[i]});
  }

  // TODO(Wil): See about a good way to memoize/grow toward a single plan graph?
  // Maybe a Procrustes graph would work well here?
  Set<GroundAction*> empty_set;
  Vec<GraphLayer> plan_graph;
  State current_state(initial_atoms);
  // plan_graph.emplace_back(GraphLayer{initial_atoms, empty_set});

  while (true) {
    Set<GroundAction*> viable_actions;
    // The state only grows in terms of atoms which can have known values
    State next_state(current_state);
    // Take actions from current state
    for (const auto& action : grounded_actions) {
      if (precondition_satisfiable(action.get(), current_state) &&
          meaningful_effect(action.get(), current_state)) {
        viable_actions.insert(action.get());
        add_new_atoms(&next_state, action.get());
      }
    }

    if (viable_actions.empty()) {
      log->debug("No more viable actions! State graph has converged at iteration {}",
                 plan_graph.size());
      plan_graph.emplace_back(GraphLayer{next_state, empty_set});
      break;
    }

    plan_graph.emplace_back(GraphLayer{current_state, std::move(viable_actions)});
    current_state = next_state;
  }

  // If we met at least one goal, we're done here
  auto goal_check = goal_met(goal, plan_graph.back().state);
  if (!goal_check.empty()) {
    log->debug("Goals #{} are feasible after {} steps", goal_check, plan_graph.size());
    return std::make_optional(Graph{std::move(plan_graph), std::move(goal_check)});
  }

  // We met no goals and will not be able to meet any goals ever
  log->critical("There appears to be no solution to the task-level problem!");
  return std::nullopt;
}

// TODO(Wil): Use action counts to suggest probable pairs
// TODO(Wil): Use branch count to suggest probable successors?
// TODO(Wil): Suggest helpful actions sorted by edit distance for precondition states?
// NOTE: This is not completeness-preserving
Vec<PrioritizedActionPtr> FFLikeHeuristic::suggest(const boost::dynamic_bitset<>& state) {
  auto action_data = action_cache.find(state);
  if (action_data == action_cache.end()) {
    // Form the plan graph from the current state
    const auto& graph_result = makeGraph(state);
    if (!graph_result) {
      log->error("Failed to get plan graph!");
      return {};
    }

    const auto& viable_actions               = graph_result->layers.front();
    const auto& [plan_actions, action_count] = relaxed_plan(*graph_result);
    Set<GroundAction*> helpful_actions;
    helpful_actions.insert(plan_actions.begin(), plan_actions.end());
    Vec<GroundAction*> other_actions;
    std::copy_if(viable_actions.actions.begin(),
                 viable_actions.actions.end(),
                 std::back_inserter(other_actions),
                 [&helpful_actions](auto* action) { return helpful_actions.count(action) == 0; });
    auto prioritized_helpful_actions = fplus::transform(
    [](const auto& action) { return std::make_shared<PrioritizedAction>(action); }, plan_actions);
    auto prioritized_other_actions = fplus::transform(
    [](const auto& action) { return std::make_shared<PrioritizedAction>(action); }, other_actions);
    Vec<PrioritizedActionPtr> all_actions;
    all_actions.reserve(prioritized_helpful_actions.size() + prioritized_other_actions.size());
    all_actions.insert(all_actions.end(),
                       prioritized_helpful_actions.begin(),
                       prioritized_helpful_actions.end());
    all_actions.insert(all_actions.end(),
                       prioritized_other_actions.begin(),
                       prioritized_other_actions.end());
    distance_cache.emplace(state, action_count);
    action_data =
    action_cache
    .emplace(
    state, ActionData{prioritized_helpful_actions, prioritized_other_actions, all_actions, false})
    .first;
  }

  const auto& actions      = action_data->second.all_actions;
  bool distances_estimated = action_data->second.distances_estimated;

  if (actions.empty()) {
    log->warn("No helpful actions for state: {}", state);
  }

  if (!distances_estimated) {
    const auto min_pos_priority = estimate_distances(state, action_data->second.helpful_actions);
    const double other_action_priority = min_pos_priority * 0.05;
    for (auto& action : action_data->second.other_actions) {
      action->priority = other_action_priority;
    }

    action_data->second.distances_estimated = true;
  }

  return action_data->second.all_actions;
}

std::pair<Vec<GroundAction*>, size_t> FFLikeHeuristic::relaxed_plan(const Graph& graph) {
  // const auto& humanize_atom = [&](const auto& e) {
  //   if (e.first < domain->num_eqclass_dims) {
  //     return std::make_pair(domain->eqclass_dimension_names.at(e.first), e.second);
  //   }
  //   return std::make_pair(
  //   domain->discrete_dimension_names.at(e.first - domain->num_eqclass_dims), e.second);
  // };

  const auto& [plan_graph, viable_goal_branches] = graph;
  // for (const auto& [state, actions] : plan_graph) {
  //   log->critical(
  //   "===Layer===\n\nState: {}\n\nActions: {}\n",
  //   fplus::transform_convert<Vec<std::pair<Str, bool>>>(humanize_atom, state),
  //   fplus::transform_convert<Vec<Str>>(
  //   [](const auto& a) { return fmt::format("{}({})", a->action->name, a->bindings); },
  //   actions));
  // }

  // Extract a relaxed plan following the FF approach
  State current_goals;
  size_t action_count = 0;
  for (const auto i : viable_goal_branches) {
    const auto& necessary_atoms = goal[i].second;
    for (const auto& atom : necessary_atoms) {
      spec::DimId idx;
      if (fplus::map_contains(domain->eqclass_dimension_ids, atom.first)) {
        idx = domain->eqclass_dimension_ids.at(atom.first);
      } else {
        idx = domain->discrete_dimension_ids.at(atom.first) + domain->num_eqclass_dims;
      }

      current_goals.insert({idx, atom.second});
    }
  }

  Vec<GroundAction*> helpful_actions;
  for (auto it = plan_graph.crbegin(); it != plan_graph.crend(); ++it) {
    // log->critical("Current goals: {}",
    //               fplus::transform_convert<Vec<std::pair<Str, bool>>>(humanize_atom,
    //                                                                   current_goals));

    const auto& level_atoms   = it->state;
    const auto& level_actions = it->actions;
    State next_goals;
    for (const auto& goal_atom : current_goals) {
      const auto& [elem_start, elem_end] = level_atoms.equal_range(goal_atom.first);
      if (std::find(elem_start, elem_end, goal_atom) != elem_end) {
        // log->critical("{} is in this layer; deferring", humanize_atom(goal_atom));
        next_goals.insert(goal_atom);
      } else {
        // Find an action to accomplish the goal
        // log->critical("Finding an action for {} from the set {}",
        //               humanize_atom(goal_atom),
        //               fplus::transform_convert<Vec<Str>>(
        //               [](const auto& a) {
        //                 return fmt::format("{}({})", a->action->name, a->bindings);
        //               },
        //               level_actions));
        for (const auto& action : level_actions) {
          const auto& atom_it = action->bound_effect.find(goal_atom.first);
          // Does the action contain the goal atom in its effect?
          if (atom_it != action->bound_effect.end() && atom_it->second == goal_atom.second) {
            // log->critical("{} causes {}",
            //               fmt::format("{}({})", action->action->name, action->bindings),
            //               humanize_atom(goal_atom));

            // Add to the action tally
            ++action_count;
            // Add the action as a helpful action if we're at the last layer
            if (it + 1 == plan_graph.crend()) {
              // log->critical("Adding {} as helpful",
              //               fmt::format("{}({})", action->action->name, action->bindings));
              helpful_actions.push_back(action);
            }

            // Add the action's precondition atoms as new goals
            for (const auto& precon_branch : action->bound_precondition) {
              for (const auto& precon_atom : precon_branch) {
                next_goals.insert(precon_atom);
              }
            }
          }
        }
      }
    }
    current_goals.swap(next_goals);
  }

  return {helpful_actions, action_count};
}

double FFLikeHeuristic::estimate_distances(const boost::dynamic_bitset<>& state,
                                           const Vec<PrioritizedActionPtr>& actions) {
  double min_pos_priority = std::numeric_limits<double>::max();
  for (const auto& action : actions) {
    // Apply action
    boost::dynamic_bitset<> action_state(state);
    apply_action(action_state, action);
    const auto& graph_result = makeGraph(action_state);
    if (!graph_result) {
      log->critical("No graph, so assuming infinite distance and thus zero priority!");
      distance_cache.emplace(action_state, std::numeric_limits<unsigned int>::max());
      continue;
    }
    const auto& viable_actions              = graph_result->layers.front();
    const auto& [new_actions, action_count] = relaxed_plan(*graph_result);
    Set<GroundAction*> helpful_actions;
    helpful_actions.insert(new_actions.begin(), new_actions.end());
    Vec<GroundAction*> other_actions;
    std::copy_if(viable_actions.actions.begin(),
                 viable_actions.actions.end(),
                 std::back_inserter(other_actions),
                 [&helpful_actions](auto* action) { return helpful_actions.count(action) == 0; });
    auto prioritized_new_actions = fplus::transform(
    [](const auto& action) { return std::make_shared<PrioritizedAction>(action); }, new_actions);
    auto prioritized_other_actions = fplus::transform(
    [](const auto& action) { return std::make_shared<PrioritizedAction>(action); }, other_actions);
    Vec<PrioritizedActionPtr> all_actions;
    all_actions.reserve(prioritized_new_actions.size() + prioritized_other_actions.size());
    all_actions.insert(all_actions.end(),
                       prioritized_new_actions.begin(),
                       prioritized_new_actions.end());
    all_actions.insert(all_actions.end(),
                       prioritized_other_actions.begin(),
                       prioritized_other_actions.end());
    distance_cache.emplace(action_state, action_count);
    action_cache.emplace(
    action_state,
    ActionData{prioritized_new_actions, prioritized_other_actions, all_actions, false});
    // action_count will be 0 iff action_state is a valid goal state. That means that action leads
    // directly to the goal and should be highly prioritized_action
    // TODO(Wil): Should I use a value > 1 for these actions (where action_count = 0)?
    action->priority = (action_count != 0) ? 1.0 / (action_count * action_count) : 1.0;
    if (action->priority < min_pos_priority) {
      min_pos_priority = action->priority;
    }
  }

  return min_pos_priority;
}

void FFLikeHeuristic::apply_action(boost::dynamic_bitset<>& state,
                                   const PrioritizedActionPtr& action) {
  const auto& effect = action->bound_effect;
  for (const auto& [dim, val] : effect) {
    state[dim] = val;
  }
}

void FFLikeHeuristic::ground_actions(const Vec<std::shared_ptr<spec::Action>>& actions,
                                     const Map<Str, Vec<Str>>& typed_objects) {
  for (const auto& action : actions) {
    // Map over params, match types. For every option for each param, make a new grounding
    Vec<Str> arg_types = fplus::get_map_values(action->param_map);
    auto num_args      = action->param_map.size();
    auto gen_dims      = [&](const auto& f, uint64_t n) {
      const auto& object_choices = typed_objects.at(arg_types[n]);
      if (n == num_args - 1) {
        return fplus::transform([](const auto& e) { return std::list<Str>(1, e); },
                                object_choices);
      }

      // NOTE: This is a candidate for speedup if all the prepending is slow
      return fplus::carthesian_product_with_where(
      [](const auto& a, auto b) {
        b.push_front(a);
        return b;
      },
      [](const auto& a, const auto& b) { return !fplus::is_elem_of(a, b); },
      object_choices,
      f(f, n + 1));
    };

    auto combos = fplus::transform_convert<Vec<GroundActionPtr>>(
    [&](const auto& binding) {
      return std::make_shared<GroundAction>(
      action,
      fplus::create_unordered_map(fplus::get_map_keys(action->param_map), binding),
      domain);
    },
    gen_dims(gen_dims, 0));
    grounded_actions.reserve(grounded_actions.size() + combos.size());
    grounded_actions.insert(grounded_actions.end(), combos.begin(), combos.end());
  }
}

/// Test if a goal has been achieved in a state
Vec<size_t> FFLikeHeuristic::goal_met(const spec::Goal& goal, const State& state) const {
  Vec<size_t> results;
  for (size_t i = 0; i < goal.size(); ++i) {
    const auto& branch    = goal[i];
    bool satisfied        = true;
    const auto& req_atoms = branch.second;
    for (const auto& [dim, value] : req_atoms) {
      spec::DimId idx;
      if (fplus::map_contains(domain->eqclass_dimension_ids, dim)) {
        idx = domain->eqclass_dimension_ids.at(dim);
      } else {
        idx = domain->discrete_dimension_ids.at(dim) + domain->num_eqclass_dims;
      }
      if (!value && state.count(idx) == 0) {
        continue;
      }

      auto dim_vals       = state.equal_range(idx);
      bool atom_satisfied = false;
      for (auto it = dim_vals.first; it != dim_vals.second; ++it) {
        atom_satisfied = atom_satisfied || (it->second == value);
      }

      if (!atom_satisfied) {
        satisfied = false;
        break;
      }
    }

    if (satisfied) {
      results.push_back(i);
    }
  }

  return results;
}

/// Test if the discrete precondition for an action is satistfied in a given graph layer
bool precondition_satisfiable(GroundAction* action, const State& state) {
  const auto& precondition = action->bound_precondition;
  for (const auto& branch : precondition) {
    bool valid = true;
    for (const auto& [dim_name, value] : branch) {
      bool one_matches = false;
      // A predicate which requires a false value can be satisfied if it has no entry in the map
      // saying "true"
      if (!value && state.count(dim_name) == 0) {
        one_matches = true;
      } else {
        auto dim_vals = state.equal_range(dim_name);
        for (auto it = dim_vals.first; it != dim_vals.second; ++it) {
          if (it->second == value) {
            one_matches = true;
            break;
          }
        }
      }

      valid = valid && one_matches;
    }

    if (valid) {
      return true;
    }
  }

  return false;
}

/// Test if an effect adds any new dimensions or new values for known dimensions
bool meaningful_effect(GroundAction* action, const State& state) {
  const auto& effect = action->bound_effect;
  for (const auto& [dim, value] : effect) {
    auto num_matches = state.count(dim);
    if (num_matches == 0) {
      return true;
    }

    if (num_matches == 1) {
      auto match = state.find(dim);
      if (match->second != value) {
        return true;
      }
    } else if (num_matches >= 3) {
      log->warn("Too many values ({}) for {}!", num_matches, dim);
    }
  }

  return false;
}

/// Add any new dimensions or values for dimensions to a layer after applying an action
void add_new_atoms(State* next_state, const GroundAction* action) {
  for (const auto& atom : action->bound_effect) {
    // Add the dimension and value to the state map
    auto num_matches = next_state->count(atom.first);
    if (num_matches == 0) {
      next_state->insert(atom);
      if (atom.second) {
        // To preserve the implicit false that was present before
        next_state->insert({atom.first, false});
      }
    } else if (num_matches == 1) {
      auto match = next_state->find(atom.first);
      if (match->second != atom.second) {
        next_state->insert(atom);
      }
    }
    // Otherwise, we will already have {dim, true} and {dim, false} in the map and don't want to
    // add duplicates
  }
}
}  // namespace symbolic::heuristic
