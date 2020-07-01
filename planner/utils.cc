#include <iostream>
#include <utility>

#include "common.hh"
#include "debug.hh"
#include "fmt/format.h"
#include "sampler.hh"

namespace debug {
void print_unimap_data(const input::specification::Domain& domain,
                       const std::shared_ptr<spdlog::logger>& log) {
  for (const auto& [_, uni] : planner::sampler::TampSampler::universe_map->graph) {
    const auto& universe = uni->sig;

    log->critical("Have {}, {} with configs:",
                  universe,
                  humanize_discrete_state(domain.eqclass_dimension_names, universe));
    for (const auto& cf : uni->configs) {
      log->critical("{}, {}",
                    cf.first,
                    humanize_discrete_state(domain.discrete_dimension_names, cf.first));
    }

    std::cout << "==========================================\n";
  }
}

std::tuple<Str, size_t, bool>
humanize_atom(const tsl::hopscotch_map<input::specification::DimId, Str>& name_index,
              const std::pair<size_t, bool>& atom) {
  return std::make_tuple(name_index.at(atom.first), atom.first, atom.second);
}

Vec<std::tuple<Str, size_t, bool>>
humanize_discrete_state(const tsl::hopscotch_map<input::specification::DimId, Str>& name_index,
                        const planner::util::DiscreteSig& discrete_state,
                        const std::optional<size_t> first_idx,
                        const std::optional<size_t> last_idx) {
  Vec<std::tuple<Str, size_t, bool>> result;
  result.reserve(discrete_state.size());
  for (size_t i = first_idx.value_or(0); i < last_idx.value_or(discrete_state.size()); ++i) {
    result.push_back(humanize_atom(name_index, {i - first_idx.value_or(0), discrete_state[i]}));
  }

  return result;
}

void GraphLog::write(const Str& output_path) {
  std::ofstream output(output_path);
  output << std::setw(4) << graph_events;
  output.close();
}

void GraphLog::add_action(const symbolic::heuristic::PrioritizedAction* const action,
                          const planner::util::DiscreteSig& base_uni,
                          const planner::util::DiscreteSig& base_cf) {
  const auto& effect = action->bound_effect;
  planner::util::DiscreteSig base_state;
  base_state.resize(base_uni.size() + base_cf.size());
  for (size_t i = 0; i < base_uni.size(); ++i) {
    base_state.set(i, base_uni[i]);
  }

  for (size_t i = 0; i < base_cf.size(); ++i) {
    base_state.set(i + base_uni.size(), base_cf[i]);
  }

  planner::util::DiscreteSig effect_state(base_state);
  for (const auto& [dim, val] : effect) {
    effect_state[dim] = val;
  }

  Str effect_state_str;
  boost::to_string(effect_state, effect_state_str);
  const auto& universe_names =
  fplus::fwd::apply(humanize_discrete_state(domain->eqclass_dimension_names,
                                            effect_state,
                                            std::nullopt,
                                            std::make_optional(domain->num_eqclass_dims)),
                    fplus::fwd::keep_if([](const auto& elem) { return std::get<2>(elem); }),
                    fplus::fwd::transform([](const auto& elem) { return std::get<0>(elem); }));

  const auto& config_names =
  fplus::fwd::apply(humanize_discrete_state(domain->discrete_dimension_names,
                                            effect_state,
                                            std::make_optional(domain->num_eqclass_dims),
                                            std::nullopt),
                    fplus::fwd::keep_if([](const auto& elem) { return std::get<2>(elem); }),
                    fplus::fwd::transform([](const auto& elem) { return std::get<0>(elem); }));

  Str uni_str;
  Str cf_str;
  Str base_state_str;
  boost::to_string(base_uni, uni_str);
  boost::to_string(base_cf, cf_str);
  boost::to_string(base_state, base_state_str);
  Str action_name = fmt::format(
  "{}({})",
  action->action->name,
  fplus::fwd::apply(action->bindings, fplus::fwd::get_map_values(), fplus::fwd::join(Str(", "))));
  graph_events["steps"].push_back(
  {{"type", "add"},
   {"base_state", base_state_str},
   {"effect_state", effect_state_str},
   {"effect_state_name", fplus::concat(Vec<Vec<Str>>({universe_names, config_names}))},
   {"priority", action->priority},
   {"action_id", fmt::format("{}-({}, {})", action_name, uni_str, cf_str)},
   {"action_name", action_name}});
}

void GraphLog::update_failure(
const planner::util::ActionDistribution::ValueData* const action_data) {
  const auto& [uni, cf, action] = action_data->data;
  Str uni_str;
  boost::to_string(uni->sig, uni_str);
  Str cf_str;
  boost::to_string(cf->sig, cf_str);
  Str action_name = fmt::format(
  "{}({})",
  action->action->name,
  fplus::fwd::apply(action->bindings, fplus::fwd::get_map_values(), fplus::fwd::join(Str(", "))));
  graph_events["steps"].push_back(
  {{"type", "failure"},
   {"action_id", fmt::format("{}-({}, {})", action_name, uni_str, cf_str)},
   {"priority", action_data->mass}});
}

void GraphLog::update_success(
const planner::util::ActionDistribution::ValueData* const action_data) {
  const auto& [uni, cf, action] = action_data->data;
  Str uni_str;
  boost::to_string(uni->sig, uni_str);
  Str cf_str;
  boost::to_string(cf->sig, cf_str);
  Str action_name = fmt::format(
  "{}({})",
  action->action->name,
  fplus::fwd::apply(action->bindings, fplus::fwd::get_map_values(), fplus::fwd::join(Str(", "))));
  graph_events["steps"].push_back(
  {{"type", "success"},
   {"action_id", fmt::format("{}-({}, {})", action_name, uni_str, cf_str)},
   {"priority", action_data->mass}});
}

void GraphLog::add_sample(const planner::util::ActionDistribution::ValueData* const action_data) {
  const auto& [uni, cf, action] = action_data->data;
  Str uni_str;
  boost::to_string(uni->sig, uni_str);
  Str cf_str;
  boost::to_string(cf->sig, cf_str);
  Str action_name = fmt::format(
  "{}({})",
  action->action->name,
  fplus::fwd::apply(action->bindings, fplus::fwd::get_map_values(), fplus::fwd::join(Str(", "))));
  graph_events["steps"].push_back({
  {"type", "sample"},
  {"action_id", fmt::format("{}-({}, {})", action_name, uni_str, cf_str)},
  });
}
}  // namespace debug
