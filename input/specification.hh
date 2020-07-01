#pragma once
#ifndef SPECIFICATION_HH
#define SPECIFICATION_HH

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "common.hh"

#define SEXPRESSO_OPT_OUT_PIKESTYLE

#ifndef SEXPRESSO_HEADER
#include "sexpresso.hpp"
#endif

#include <boost/dynamic_bitset.hpp>

#include "formula.hh"
#include "scenegraph.hh"
#include "tsl/hopscotch_map.h"
#include "tsl/hopscotch_set.h"

namespace input::specification {
using BitVecSizeT = boost::dynamic_bitset<>::size_type;
using DimId       = BitVecSizeT;
namespace {
  auto swap_keys_and_values(const tsl::hopscotch_map<Str, DimId>& m) {
    tsl::hopscotch_map<DimId, Str> result;
    for (const auto& v : m) {
      result.emplace(v.second, v.first);
    }

    return result;
  }
}  // namespace

// TODO(Wil): Don't use strings/lists etc for dims
using UnboundDiscreteDim  = std::pair<Vec<Str>, bool>;
using UnboundDiscreteDims = Vec<UnboundDiscreteDim>;

// TODO(Wil): There is probably a better way to separate these types from the
// Configuration/ConfigurationSet types. The distinction is that these represent formulae as
// raw strings, whereas the Configuration/ConfigurationSet use Formula objects
using StrConfiguration = std::pair<Str, UnboundDiscreteDims>;

using UnboundConfiguration = std::pair<Formula, UnboundDiscreteDims>;

using Configuration = std::pair<Formula, Map<Str, bool>>;

struct Action {
  Action(const Str& name,
         const Vec<StrConfiguration>& precondition,
         const Map<Str, Str>& param_map,
         const UnboundDiscreteDims& effect)
  : name(name), param_map(param_map), precondition(make_formulae(precondition)), effect(effect) {}
  const Str name;
  const Map<Str, Str> param_map;
  Vec<UnboundConfiguration> precondition;
  const UnboundDiscreteDims effect;

 private:
  Vec<UnboundConfiguration> make_formulae(const Vec<StrConfiguration>& str_configs);
};

// Used to encode the effects of toggling a kinematic predicate
struct KinematicEffect {
  KinematicEffect(const Map<Str, int>& order,
                  const Vec<std::pair<Str, Str>>& link,
                  const Vec<std::pair<Str, Str>>& unlink)
  : order(std::move(order)), link(std::move(link)), unlink(std::move(unlink)) {}
  const Map<Str, int> order;
  const Vec<std::pair<Str, Str>> link;
  const Vec<std::pair<Str, Str>> unlink;
};

struct Domain {
  Domain(const Map<Str, Vec<Str>>& typed_objects,
         const int object_count,
         const Vec<std::shared_ptr<Action>>& actions,
         const tsl::hopscotch_map<Str, DimId>& discrete_dimensions,
         const tsl::hopscotch_map<Str, DimId>& eqclass_dimensions,
         const Map<Str, std::shared_ptr<KinematicEffect>>& eqclass_effects,
         const Str& predicates_file)
  : typed_objects(std::move(typed_objects))
  , object_count(object_count)
  , actions(std::move(actions))
  , discrete_dimension_ids(std::move(discrete_dimensions))
  , discrete_dimension_names(swap_keys_and_values(discrete_dimension_ids))
  , eqclass_dimension_ids(std::move(eqclass_dimensions))
  , eqclass_dimension_names(swap_keys_and_values(eqclass_dimension_ids))
  , num_eqclass_dims(eqclass_dimension_ids.size())
  , num_symbolic_dims(discrete_dimension_ids.size())
  , eqclass_effects(std::move(eqclass_effects))
  , predicates_file(std::move(predicates_file)) {}

  const Map<Str, Vec<Str>> typed_objects;
  const int object_count;
  const Vec<std::shared_ptr<Action>> actions;
  const tsl::hopscotch_map<Str, DimId> discrete_dimension_ids;
  const tsl::hopscotch_map<DimId, Str> discrete_dimension_names;
  const tsl::hopscotch_map<Str, DimId> eqclass_dimension_ids;
  const tsl::hopscotch_map<DimId, Str> eqclass_dimension_names;
  const DimId num_eqclass_dims;
  const DimId num_symbolic_dims;
  const Map<Str, std::shared_ptr<KinematicEffect>> eqclass_effects;
  const Str predicates_file;
  bool handle_kin_effect(const DimId dimension_id,
                         const Map<Str, Transform3r>& pose_map,
                         structures::scenegraph::Graph* sg,
                         const bool old_val,
                         const bool new_val) const;
};

// We assume the goal is separated into a formula of continuous predicates and a map of discrete
// predicates to desired values
using Goal = Vec<Configuration>;

// Set of true discrete atoms. Unmentioned discrete atoms are assumed to be false; continuous
// predicates are tested from the scene graph.
using Initial = Set<Str>;

std::optional<std::tuple<Domain, Initial, Goal>>
load(sexpresso::Sexp* domain_sexp, sexpresso::Sexp* problem_sexp, const Str& predicate_filename);
}  // namespace input::specification
#endif
