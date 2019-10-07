#include "universe_map.hh"

#include "planner_utils.hh"

namespace planner::util {

double GOAL_WEIGHT;
bool UniverseMap::check_precondition(const HashableStateSpace::StateType* const state,
                                     symbolic::heuristic::PrioritizedAction* action) const {
  predicate_env->set_bindings(action->bindings);
  const auto& precondition_branches = action->action->precondition;
  return std::any_of(
  precondition_branches.begin(), precondition_branches.end(), [&](const auto& branch) {
    return (*predicate_env)(branch.first.normal_fn_name, space_, state, base_movable);
  });
}

bool UniverseMap::check_valid_transition(const HashableStateSpace::StateType* const s1,
                                         const HashableStateSpace::StateType* const s2) {
  // Check if a transition between two states is valid in universe and config transitions
  UniverseSig u1(num_eqclass_dims, 0);
  UniverseSig u2(num_eqclass_dims, 0);
  const auto& us1 = s1->components[eqclass_space_idx]->as<ob::CompoundState>();
  const auto& us2 = s2->components[eqclass_space_idx]->as<ob::CompoundState>();
  for (unsigned int i = 0; i < num_eqclass_dims; ++i) {
    u1.set(i, us1->as<ob::DiscreteStateSpace::StateType>(i)->value == 1);
    u2.set(i, us2->as<ob::DiscreteStateSpace::StateType>(i)->value == 1);
  }

  ConfigSig c1(num_discrete_dims, 0);
  ConfigSig c2(num_discrete_dims, 0);
  const auto& cs1 = s1->components[discrete_space_idx]->as<ob::CompoundState>();
  const auto& cs2 = s2->components[discrete_space_idx]->as<ob::CompoundState>();

  for (unsigned int i = 0; i < num_discrete_dims; ++i) {
    c1.set(i, cs1->as<ob::DiscreteStateSpace::StateType>(i)->value == 1);
    c2.set(i, cs2->as<ob::DiscreteStateSpace::StateType>(i)->value == 1);
  }

  // NOTE/TODO(Wil): This is only necessary because we don't precompile the effect map
  // A transition is valid only if (a) it is the identity transition or (b) it is known viable
  // and we are starting at a state at which this transition is known to happen
  if (u1 == u2 && c1 == c2) {
    return true;
  }

  // NOTE: We don't need nullptr checks here because the `at` method throws an exception if the
  // key is not present, which should never be possible
  const auto& [uni_data, cf_data] = get_data({u1, c1});
  const auto& transition          = cf_data->viable_transitions.find({u2, c2});
  if (transition != cf_data->viable_transitions.end()) {
    const auto& [_, transition_states] = *transition;
    if (transition_states->known_states.find(*s1) != transition_states->known_states.end()) {
      return true;
    }

    predicate_env->set_universe(uni_data);
    for (const auto& action : transition_states->actions) {
      if (check_precondition(s1, action)) {
        auto s1_copy           = space_->cloneState(s1);
        (*action_log)[s1_copy] = action;
        add_transition({u1, c1}, {u2, c2}, s1_copy->as<HashableStateSpace::StateType>(), action);
        return true;
      }
    }
  }

  return false;
}

std::pair<Universe*, Config*>
UniverseMap::add_transition(const std::pair<const UniverseSig&, const ConfigSig&>& from,
                            const std::pair<const UniverseSig&, const ConfigSig&>& to,
                            HashableStateSpace::StateType* at_state,
                            symbolic::heuristic::PrioritizedAction* with_action) {
  // NOTE: For now, I'm ignoring the possibility of multithreading b/c none of the planners I'm
  // using right now are multithreaded.

  if (at_state == nullptr) {
    spdlog::critical("Passed a null state pointer!");
    throw std::runtime_error("Added a transition with a null state!");
  }

  if (with_action == nullptr) {
    spdlog::critical("Passed a null action pointer!");
    throw std::runtime_error("Added a transition with a null action!");
  }

  const auto& [u1, c1] = from;
  const auto& [u2, c2] = to;

  // NOTE: (u1, c1) should *always* already exist in the maps
  const auto& u1_data = graph.at(u1);
  const auto& c1_data = u1_data->configs.at(c1);

  // NOTE: Either or both of (u2, c2) may not exist yet
  const auto& [u2_data_it, new_uni] = graph.try_emplace(u2, std::make_shared<Universe>());
  const auto& u2_data               = u2_data_it->second;
  if (new_uni) {
    u2_data->sig = u2;
  }

  const auto& [c2_data_it, new_cf] = u2_data->configs.try_emplace(c2, std::make_shared<Config>());
  const auto& c2_data              = c2_data_it->second;
  if (new_cf) {
    c2_data->sig = c2;
  }

  const auto& [viable_transitions_it, _] =
  c1_data->viable_transitions.try_emplace(to, std::make_shared<Transition>());
  const auto& [_sig, viable_transitions] = *viable_transitions_it;
  viable_transitions->known_states.insert(*at_state);
  viable_transitions->actions.push_back(with_action);
  if (fplus::map_contains(c1_data->state_transitions, *at_state)) {
    throw std::runtime_error("State is re-used for multiple transitions in the same universe & "
                             "config! You have bugs to fix");
  }

  c1_data->state_transitions[*at_state] = to;
  return {u2_data.get(), c2_data.get()};
}

std::pair<Universe*, Config*>
UniverseMap::get_data(const std::pair<const UniverseSig&, const ConfigSig&>& sig) {
  const auto& uni_data = graph[sig.first];
  if (uni_data == nullptr) {
    throw std::runtime_error("Requested an undiscovered universe!");
  }

  const auto& cf_data = uni_data->configs[sig.second];
  if (cf_data == nullptr) {
    throw std::runtime_error("Requested an undiscovered config!");
  }

  return {uni_data.get(), cf_data.get()};
}

void UniverseMap::added_state(HashableStateSpace::StateType* state) {
  // Flip the "reached" bit for newly reached universes/configs when a state transitioning to
  // them is actually added to the tree
  const UniverseSig u1_sig =
  discrete_of_state(state->as<ob::CompoundState>(eqclass_space_idx), num_eqclass_dims);
  const ConfigSig c1_sig =
  discrete_of_state(state->as<ob::CompoundState>(discrete_space_idx), num_discrete_dims);

  const auto& [u1_data, c1_data] = get_data({u1_sig, c1_sig});
  const auto& transition_it      = c1_data->state_transitions.find(*state);
  if (transition_it != c1_data->state_transitions.end()) {
    const auto& [u2_sig, c2_sig]   = transition_it->second;
    const auto& [u2_data, c2_data] = get_data({u2_sig, c2_sig});
    if (!u2_data->actually_reached || !c2_data->actually_reached) {
      u2_data->actually_reached = true;
      c2_data->actually_reached = true;
      add_actions(u2_data, c2_data);
    }
  }
}

ActionDistribution::ValueData* const UniverseMap::sample() {
  return distribution.sample(rng.uniform01());
}

UniverseMap::UniverseMap(const spec::Initial& init_atoms,
                         HashableStateSpace::StateType* init_state,
                         SceneGraph init_sg,
                         spec::Goal* goal,
                         spec::Domain* domain,
                         ob::CompoundStateSpace* objects_space,
                         ob::StateSpace* space,
                         unsigned int eqclass_space_idx,
                         unsigned int discrete_space_idx,
                         unsigned int objects_space_idx,
                         bool robot_base_movable)
: num_eqclass_dims(domain->num_eqclass_dims)
, num_discrete_dims(domain->num_symbolic_dims)
, eqclass_space_idx(eqclass_space_idx)
, discrete_space_idx(discrete_space_idx)
, base_movable(robot_base_movable)
, space_(space) {
  // Create predicate testing environment
  predicate_env =
  std::make_unique<symbolic::predicate::LuaEnv<double>>("universe_map-predicate",
                                                        symbolic::predicate::BOOL_PRELUDE_PATH);
  predicate_env->load_predicates(domain->predicates_file);
  for (const auto& action : domain->actions) {
    for (auto& [formula, _] : action->precondition) {
      predicate_env->load_formula(&formula);
    }
  }

  UniverseSig init_universe(num_eqclass_dims, 0);
  ConfigSig init_config(num_discrete_dims, 0);
  for (const auto& dim : init_atoms) {
    if (fplus::map_contains(domain->eqclass_dimension_ids, dim)) {
      init_universe[domain->eqclass_dimension_ids.at(dim)] = true;
    } else {
      init_config[domain->discrete_dimension_ids.at(dim)] = true;
    }
  }

  // Add initial universe
  const auto& [uni_data_it, _uni_added] =
  graph.emplace(init_universe, std::make_shared<Universe>());
  const auto& [_uni_sig, uni_data] = *uni_data_it;
  uni_data->actually_reached       = true;
  uni_data->sig                    = init_universe;

  // Add initial config
  const auto& [cf_data_it, _cf_added] =
  uni_data->configs.emplace(init_config, std::make_unique<Config>());
  const auto& [_cf_sig, cf_data] = *cf_data_it;
  cf_data->sig                   = init_config;
  cf_data->actually_reached      = true;

  // Add initial poses
  auto initial_pose =
  objects_space->cloneState(init_state->as<ob::CompoundState>(objects_space_idx))
  ->as<ob::CompoundState>();
  cf_data->valid_poses.emplace_back(initial_pose);
  init_state->object_poses = cf_data->valid_poses.back();

  uni_data->sg = std::move(init_sg);

  // Make the heuristic
  heuristic = std::make_unique<symbolic::heuristic::FFLikeHeuristic>(
  domain->actions, domain->typed_objects, *goal, domain);

  // Add the initial sampleable actions
  add_actions(uni_data.get(), cf_data.get());

  // TODO(Wil): Add the goal universe & config once goal sampling is fixed
}

void UniverseMap::add_actions(Universe* uni, Config* cf) {
  const auto& uni_sig = uni->sig;
  const auto& cf_sig  = cf->sig;
  DiscreteSig symbolic_state(cf_sig.size() + uni_sig.size(), 0);

  // Add all of the true atoms from the uni_sigverse
  for (size_t i = 0; i < uni_sig.size(); ++i) {
    symbolic_state.set(i, uni_sig[i]);
  }

  // Add all of the true atoms from the config
  for (size_t i = 0; i < cf_sig.size(); ++i) {
    symbolic_state.set(i + num_eqclass_dims, cf_sig[i]);
  }

  // Request helpful actions and add them to the action set
  const auto& suggested_actions = heuristic->suggest(symbolic_state);
  // NOTE: The goal universe may have no action suggestions, so we check for this and add a null
  // action
  if (suggested_actions.empty()) {
    // TODO(Wil): This seems like the wrong way to handle the goal universe...
    // Is there a more principled way to weight it?
    distribution.add(GOAL_WEIGHT, std::make_tuple(uni, cf, nullptr));
    return;
  }

  for (const auto& action : suggested_actions) {
    distribution.add(action->priority, std::make_tuple(uni, cf, action));
  }
}

void UniverseMap::clear(const spec::Initial& init_atoms,
                        HashableStateSpace::StateType* init_state,
                        SceneGraph init_sg,
                        spec::Domain* domain,
                        spec::Goal* goal,
                        ob::CompoundStateSpace* objects_space,
                        unsigned int objects_space_idx) {
  graph.clear();
  UniverseSig init_universe(num_eqclass_dims, 0);
  ConfigSig init_config(num_discrete_dims, 0);
  for (const auto& dim : init_atoms) {
    if (fplus::map_contains(domain->eqclass_dimension_ids, dim)) {
      init_universe[domain->eqclass_dimension_ids.at(dim)] = true;
    } else {
      init_config[domain->discrete_dimension_ids.at(dim)] = true;
    }
  }

  // Add initial universe
  const auto& [uni_data_it, _uni_added] =
  graph.emplace(init_universe, std::make_shared<Universe>());
  const auto& [_uni_sig, uni_data] = *uni_data_it;
  uni_data->actually_reached       = true;
  uni_data->sig                    = init_universe;

  // Add initial config
  const auto& [cf_data_it, _cf_added] =
  uni_data->configs.emplace(init_config, std::make_unique<Config>());
  const auto& [_cf_sig, cf_data] = *cf_data_it;
  cf_data->sig                   = init_config;
  cf_data->actually_reached      = true;

  // Add initial poses
  auto initial_pose =
  objects_space->cloneState(init_state->as<ob::CompoundState>(objects_space_idx))
  ->as<ob::CompoundState>();
  cf_data->valid_poses.emplace_back(initial_pose);
  init_state->object_poses = cf_data->valid_poses.back();

  uni_data->sg = init_sg;

  // Make the heuristic
  heuristic = std::make_unique<symbolic::heuristic::FFLikeHeuristic>(
  domain->actions, domain->typed_objects, *goal, domain);

  // Add the initial sampleable actions
  distribution.clear();
  add_actions(uni_data.get(), cf_data.get());
}
}  // namespace planner::util
