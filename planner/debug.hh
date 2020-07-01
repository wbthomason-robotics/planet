#pragma once
#ifndef DEBUG_HH
#define DEBUG_HH
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <utility>

#include "common.hh"
#include "fplus/fplus.hpp"
#include "hashable_statespace.hh"
#include "heuristic.hh"
#include "signatures.hh"
#include "spdlog/spdlog.h"
#include "specification.hh"

namespace debug {
void print_unimap_data(const input::specification::Domain& domain,
                       const std::shared_ptr<spdlog::logger>& log);

std::tuple<Str, size_t, bool>
humanize_atom(const tsl::hopscotch_map<input::specification::DimId, Str>& name_index,
              const std::pair<size_t, bool>& atom);

Vec<std::tuple<Str, size_t, bool>>
humanize_discrete_state(const tsl::hopscotch_map<input::specification::DimId, Str>& name_index,
                        const planner::util::DiscreteSig& discrete_state,
                        const std::optional<size_t> first_idx = std::nullopt,
                        const std::optional<size_t> last_idx  = std::nullopt);

struct GraphLog {
  nlohmann::json graph_events;
  const input::specification::Domain* const domain;
  GraphLog(const input::specification::Initial& initial_atoms,
           const input::specification::Domain* const domain)
  : domain(domain) {
    planner::util::DiscreteSig initial_state(domain->num_eqclass_dims + domain->num_symbolic_dims,
                                             0);
    for (const auto& dim : initial_atoms) {
      if (fplus::map_contains(domain->eqclass_dimension_ids, dim)) {
        initial_state.set(domain->eqclass_dimension_ids.at(dim), true);
      } else {
        initial_state.set(domain->discrete_dimension_ids.at(dim) + domain->num_eqclass_dims, true);
      }
    }

    Str bit_string;
    boost::to_string(initial_state, bit_string);
    graph_events["initial"] = {{"bits", bit_string}, {"names", initial_atoms}};
    graph_events["steps"]   = nlohmann::json::array();
  }

  void clear() { graph_events["steps"].clear(); }
  void write(const Str& output_path);
  void add_action(const symbolic::heuristic::PrioritizedAction* const action,
                  const planner::util::DiscreteSig& base_uni,
                  const planner::util::DiscreteSig& base_cf);
  void update_failure(const planner::util::ActionDistribution::ValueData* const action_data);
  void update_success(const planner::util::ActionDistribution::ValueData* const action_data);
  void add_sample(const planner::util::ActionDistribution::ValueData* const action_data);
};
}  // namespace debug
#endif /* end of include guard: DEBUG_HH */
