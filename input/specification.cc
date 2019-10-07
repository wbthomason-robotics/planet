#include "specification.hh"

#include <list>
#include <regex>
#include <stdexcept>

#include <fmt/format.h>
#include "fplus/fplus.hpp"

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace input::specification {
namespace fwd = fplus::fwd;
namespace {
  auto log = spdlog::stdout_color_mt("specification");

  auto convert_sexp(const sexpresso::Sexp& sexp) {
    // NOTE: We have to replace the \\\? pattern because of escaping sexpresso does. It also
    // injects some unwanted quotes we need to remove
    // NOTE/TODO(Wil): Might be better to construct these once outside the function
    const std::regex unescape_pattern(R"pat(\\\\\\\?)pat");
    const std::regex quote_pattern(R"pat(")pat");
    // NOTE: We replace with an underscore to ensure names aren't keywords
    return std::regex_replace(std::regex_replace(sexp.toString(), unescape_pattern, "_"),
                              quote_pattern,
                              "");
  }

  auto sexp_filter(const Str& keyword, const Vec<sexpresso::Sexp>& sexps) {
    return fplus::keep_if(
    [&](auto x) -> bool { return x.isSexp() && x.value.sexp[0].toString() == keyword; }, sexps);
  }

  Str dimension_of_ground_atom(const sexpresso::Sexp& atom) {
    return fwd::apply(atom.value.sexp, fwd::transform(convert_sexp), fwd::join_elem('_'));
  }

  StrConfiguration empty_helper(const StrConfiguration& elem) {
    if (elem.first.length() == 0) {
      return {"True()", elem.second};
    }

    return elem;
  }

  // Note that we pass eff_sexp by value here and in the below lambdas because the type has to
  // match between the argument to UnaryF and the contained object in Container in
  // transform_reduce_1
  UnboundDiscreteDims parse_effect(sexpresso::Sexp eff_sexp, const Set<Str>& disc_preds) {
    /// Parse a PDDL effect formula into a set of discrete dimensions and corresponding values
    auto formula_operator = eff_sexp.value.sexp[0].toString();

    auto parse_helper = fplus::bind_2nd_of_2(parse_effect, disc_preds);

    if (formula_operator == "and") {
      auto effect = fwd::apply(eff_sexp.value.sexp,
                               fwd::tail(),
                               fwd::transform(parse_helper),
                               fwd::keep_if([](const auto& elem) { return elem.size() > 0; }),
                               fwd::concat());
      return effect;
    }

    if (formula_operator == "not") {
      auto effect = fwd::apply(eff_sexp.value.sexp,
                               fwd::tail(),
                               fwd::transform(parse_helper),
                               fwd::keep_if([](const auto& elem) { return elem.size() > 0; }),
                               fwd::concat());
      return fplus::transform(fwd::transform_snd(std::logical_not<>()), effect);
    }

    // Note that we assume there will never be "where", "forall", or other unsupported keywords
    if (disc_preds.find(formula_operator) != disc_preds.end()) {
      return UnboundDiscreteDims(
      {std::make_pair(fplus::transform(convert_sexp, eff_sexp.value.sexp), true)});
    }

    return UnboundDiscreteDims();
  }

  // NOTE: This is a very naive, probably inefficient implementation
  Vec<StrConfiguration> parse_formula_sexp(sexpresso::Sexp formula, const Set<Str>& disc_preds) {
    /// Parse a PDDL precondition or goal formula into a set of satisfying configurations
    auto formula_operator = formula.value.sexp[0].toString();
    auto curried_parser   = fplus::bind_2nd_of_2(parse_formula_sexp, disc_preds);
    auto merge_helper     = [](auto const& config, auto const& elem) -> StrConfiguration {
      Str new_formula;
      if (!config.first.empty()) {
        new_formula =
        !elem.first.empty() ? fmt::format("And({}, {})", config.first, elem.first) : config.first;
      } else {
        new_formula = elem.first;
      }

      return {new_formula, fplus::append(config.second, elem.second)};
    };

    if (formula_operator == "and") {
      auto subsets = fplus::transform(curried_parser, fplus::tail(formula.value.sexp));
      auto nondisjunctive_subsets =
      fplus::keep_if([](auto const& elem) { return elem.size() == 1; }, subsets);
      auto disjunctive_subsets =
      fplus::keep_if([](auto const& elem) { return elem.size() > 1; }, subsets);
      auto nondisjunctive_config =
      fwd::apply(nondisjunctive_subsets, fwd::concat(), fwd::reduce_1(merge_helper));
      if (!disjunctive_subsets.empty()) {
        return fwd::apply(disjunctive_subsets,
                          fwd::concat(),
                          fwd::transform([&](auto const& elem) {
                            auto result = merge_helper(nondisjunctive_config, elem);
                            return result;
                          }));
      }

      return {nondisjunctive_config};
    }

    if (formula_operator == "not") {
      return fplus::transform(
      [](auto const& elem) -> StrConfiguration {
        return {elem.first.empty() ? elem.first : fmt::format("Not({})", elem.first),
                fplus::transform(fwd::transform_snd(std::logical_not<>()), elem.second)};
      },
      parse_formula_sexp(formula.value.sexp[1], disc_preds));
    }

    if (formula_operator == "or") {
      const auto& all_branches =
      fwd::apply(formula.value.sexp, fwd::tail(), fwd::transform_and_concat(curried_parser));

      // NOTE: This is inefficient but concise to write
      const auto& [pure_continuous_branches, rest_branches] =
      fplus::partition([](const auto& elem) { return elem.second.empty(); }, all_branches);
      StrConfiguration pure_continuous_formula;
      if (!pure_continuous_branches.empty()) {
        pure_continuous_formula = fplus::reduce_1(
        [](const auto& acc, const auto& elem) {
          return StrConfiguration(fmt::format("Or({}, {})", acc.first, elem.first), acc.second);
        },
        pure_continuous_branches);
      }

      return fplus::append_elem(pure_continuous_formula, rest_branches);
    }

    // Predicate case
    if (disc_preds.find(formula_operator) != disc_preds.end()) {
      return {{"",
               UnboundDiscreteDims(
               {std::make_pair(fplus::transform(convert_sexp, formula.value.sexp), true)})}};
    }

    return {{fmt::format(
             "{}({})",
             formula_operator,
             fwd::apply(formula.value.sexp,
                        fwd::tail(),
                        fwd::transform_reduce_1(convert_sexp,
                                                [](const auto& arg_str, const auto& new_arg) {
                                                  return fmt::format("{}, {}", arg_str, new_arg);
                                                }))),
             UnboundDiscreteDims()}};
  }

  std::pair<Vec<std::pair<Str, Str>>, Vec<std::pair<Str, Str>>>
  parse_kinematic_effect(Vec<sexpresso::Sexp>::iterator& pred_args) {
    Vec<std::pair<Str, Str>> link_effect;
    Vec<std::pair<Str, Str>> unlink_effect;
    auto& clauses = pred_args->value.sexp;

    for (auto& clause : clauses) {
      Vec<std::pair<Str, Str>>* effect_vec;
      if (clause.value.sexp[0].toString() == "link") {
        effect_vec = &link_effect;
      } else if (clause.value.sexp[0].toString() == "unlink") {
        effect_vec = &unlink_effect;
      } else {
        log->warn("Unexpected clause type in kinematic effect: {}", clause.toString());
        continue;
      }

      const auto pairs = fwd::apply(clause.arguments().sexp.value.sexp,
                                    fwd::drop(2),
                                    fwd::zip(fplus::tail(clause.arguments().sexp.value.sexp)),
                                    fwd::transform([](const auto& p) -> std::pair<Str, Str> {
                                      return {convert_sexp(p.first), convert_sexp(p.second)};
                                    }));

      effect_vec->insert(effect_vec->end(), pairs.begin(), pairs.end());
    }

    return {std::move(link_effect), std::move(unlink_effect)};
  }
}  // namespace

using AtomMap = Map<Str, sexpresso::Sexp>;
AtomMap sexp_to_dict(const sexpresso::Sexp& sexp) {
  // Note: This function heavily assumes that it is only passed s-expressions that look like
  // (:key value :key value...)
  // Type deduction fails for some reason, so we have to write it like this
  return fplus::pairs_to_map<AtomMap>(fwd::apply(
  sexp.value.sexp, fwd::adjacent_pairs(), fwd::transform(fwd::transform_fst(convert_sexp))));
}

Vec<UnboundConfiguration> Action::make_formulae(const Vec<StrConfiguration>& str_configs) {
  Vec<UnboundConfiguration> result;
  for (size_t i = 0; i < str_configs.size(); ++i) {
    const auto& config = str_configs[i];
    result.emplace_back(
    Formula(fmt::format("{}_pre_{}", name, i), fplus::get_map_keys(param_map), config.first),
    config.second);
  }

  return result;
}

bool Domain::handle_kin_effect(const DimId dimension_id,
                               const Map<Str, Transform3r>& pose_map,
                               structures::scenegraph::Graph* sg,
                               const bool old_val,
                               const bool new_val) const {
  if (old_val == new_val) {
    // We only change the graph if the dimension was toggled
    return false;
  }

  const auto& ground_dimension = eqclass_dimension_names.at(dimension_id);
  const auto components        = fplus::split('_', false, ground_dimension);
  const auto& pred_name        = fplus::head(components);
  const auto& node_names       = fplus::tail(components);

  const auto& kin_effect                     = eqclass_effects.at(pred_name);
  const Vec<std::pair<Str, Str>>* effect_vec = nullptr;
  const bool toggled_off                     = old_val && !new_val;
  if (toggled_off) {
    effect_vec = &kin_effect->unlink;
  } else {
    effect_vec = &kin_effect->link;
  }

  for (const auto& [parent, child] : *effect_vec) {
    const auto parent_name = node_names[kin_effect->order.at(parent)];
    const auto child_name  = node_names[kin_effect->order.at(child)];
    // log->info("{} {} and {}", toggled_off ? "unlinking" : "linking", parent_name, child_name);
    if (toggled_off) {
      auto& child_node     = sg->extract(child_name);
      child_node.transform = pose_map.at(child_name);
      sg->add_tree(child_node.self_idx);
      // log->info("Put down {} with transform\n{}",
      //           child_node->name,
      //           child_node->transform.matrix());
    } else {
      auto& parent_node = sg->find(parent_name);
      // Eigen::Vector3d origin(0.0, 0.0, 0.0);
      // const auto parent_point = pose_map.at(parent_name) * origin;
      // log->info(
      // "{} is at\n{}\nwith\n{}", parent_name, parent_point, pose_map.at(parent_name).matrix());
      auto& child_node = sg->extract(child_name);
      // const auto child_point = pose_map.at(child_name) * origin;
      // log->info(
      // "{} is at\n{}\nwith\n{}", child_name, child_point, pose_map.at(child_name).matrix());
      // const auto diff = parent_point - child_point;
      // log->info("They are {} far away", diff.norm());
      // Compute the relative pose
      child_node.transform = pose_map.at(parent_name).inverse() * pose_map.at(child_name);
      // log->info("{} picked up {} with transform\n{}\ngetting\n{}\nand\n{}",
      //           parent_node->name,
      //           child_node->name,
      //           child_node->transform.matrix(),
      //           (pose_map.at(parent_name) * child_node->transform) * origin,
      //           (pose_map.at(parent_name) * child_node->transform).matrix());
      sg->reparent_child(parent_node, child_node);
    }
  }

  return true;
}

std::tuple<std::optional<Domain>, std::optional<Initial>, std::optional<Goal>>
load(sexpresso::Sexp* domain_sexp, sexpresso::Sexp* problem_sexp, const Str& predicate_filename) {
  const Str separator = ", ";
  // Construct the domain
  std::optional<Domain> domain_result;
  tsl::hopscotch_map<Str, DimId> discrete_dimensions;
  tsl::hopscotch_map<Str, DimId> eqclass_dimensions;
  Set<Str> discrete_predicates;
  Map<Str, Vec<Str>> typed_objects;
  Map<Str, std::shared_ptr<KinematicEffect>> kinematic_effects;

  try {
    // Get object names and types
    int object_count = 0;
    Vec<Str> object_labels;
    for (auto& obj_elt : problem_sexp->getChildByPath("define/:objects")->arguments()) {
      auto elt = obj_elt.toString();
      if (elt[0] == '-') {
        typed_objects.insert({elt, std::move(object_labels)});
        object_labels.clear();
      } else {
        ++object_count;
        object_labels.push_back(elt);
      }
    }

    // The last thing in the object list may not be a type, in which case we give the generic
    // "obj" type to the remaining untyped objects
    if (!object_labels.empty()) {
      typed_objects.insert({"obj", std::move(object_labels)});
    }

    log->debug("Built object map");
    log->debug("Found objects: {}", fplus::show_cont(typed_objects));

    // Handle discrete predicates and dimensions
    DimId sym_dim_idx = 0;
    DimId eq_dim_idx  = 0;
    for (auto& pred : domain_sexp->getChildByPath("define/:predicates")->arguments()) {
      auto pred_name = pred.value.sexp[0].toString();
      Vec<Str> arg_types;
      Vec<Str> ordered_arg_names;
      Vec<std::pair<Str, Str>> link_effect;
      Vec<std::pair<Str, Str>> unlink_effect;
      int arg_count     = 0;
      bool is_discrete  = false;
      bool is_kinematic = false;
      auto pred_args    = pred.value.sexp.begin();
      ++pred_args;
      for (auto end = pred.value.sexp.end(); pred_args != end; ++pred_args) {
        auto arg = pred_args->toString();
        if (arg[0] == '-') {
          auto repeated_types = fplus::repeat<Vec<Str>>(arg_count, {arg});
          arg_types.insert(arg_types.end(), repeated_types.begin(), repeated_types.end());
          arg_count = 0;
        } else if (arg == ":discrete") {
          is_discrete = true;
        } else if (arg == ":kinematic") {
          is_kinematic = true;
          // Kinematic predicates are required to annotate the link and unlink effects of their
          // toggling, so we can parse the rest of the sexp
          const auto effects = parse_kinematic_effect(++pred_args);
          link_effect        = effects.first;
          unlink_effect      = effects.second;
        } else {
          ordered_arg_names.push_back(convert_sexp(*pred_args));
          ++arg_count;
        }
      }

      if (arg_count > 0) {
        auto repeated_types = fplus::repeat<Vec<Str>>(arg_count, {"obj"});
        arg_types.insert(arg_types.end(), repeated_types.begin(), repeated_types.end());
      }

      // I _think_ that we only need to care about discrete predicates here. Any continuous
      // predicate should be called correctly by the actions that use them in preconditions
      if (is_discrete || is_kinematic) {
        // TODO(Wil): Constructing this many dimensions is bad, can we do better? Probably by
        // being lazy - we could make a function to test if a predicate call could possibly be a
        // discrete dimension. But then we can't construct the effect map without enumerating
        // possible argument combinations *anyway*. It's not clear how much that would/wouldn't
        // help.

        // Construct all possible correctly-typed groundings of the predicate
        int num_args = arg_types.size();
        log->debug("Number of args for '{}' is {}", pred_name, num_args);
        auto gen_dims = [&](const auto& f, int n) {
          const auto& object_choices = typed_objects[arg_types[n]];
          if (n == num_args - 1) {
            return fplus::transform([](const auto& e) { return std::list<Str>(1, e); },
                                    object_choices);
          }

          return fplus::carthesian_product_with_where(
          [](const auto& a, auto b) {
            b.push_front(a);
            return b;
          },
          [](const auto& a, const auto& b) { return !fplus::is_elem_of(a, b); },
          object_choices,
          f(f, n + 1));
        };

        auto combos = fplus::pairs_to_map<Map<Str, DimId>>(
        fwd::apply(gen_dims(gen_dims, 0),
                   fwd::transform(fwd::prepend_elem(pred_name)),
                   fwd::transform(fwd::join(Str("_"))),
                   fwd::transform([&](const auto& dim_name) {
                     return std::make_pair(dim_name, is_kinematic ? eq_dim_idx++ : sym_dim_idx++);
                   })));

        discrete_predicates.insert(pred_name);

        if (!is_kinematic) {
          discrete_dimensions.reserve(discrete_dimensions.size() + combos.size());
          discrete_dimensions.insert(combos.begin(), combos.end());
        } else {
          eqclass_dimensions.insert(combos.begin(), combos.end());
          int idx = 0;
          kinematic_effects.emplace(
          pred_name,
          new KinematicEffect(
          fplus::pairs_to_map<Map<Str, int>>(
          fplus::zip(ordered_arg_names,
                     fplus::transform([&](const auto& n) { return idx++; }, ordered_arg_names))),
          link_effect,
          unlink_effect));
        }
      }
    }

    log->debug("Found {} discrete predicates and {} discrete dimensions",
               discrete_predicates.size(),
               discrete_dimensions.size());
    log->debug("Predicates are {}; dimensions are {}", discrete_predicates, discrete_dimensions);
    log->debug("There are {} kinematic dimensions: {}",
               eqclass_dimensions.size(),
               eqclass_dimensions);

    // Build the action objects
    Vec<std::shared_ptr<Action>> actions;
    for (const auto& action_map : fplus::transform(
         sexp_to_dict, sexp_filter(":action", domain_sexp->value.sexp[0].value.sexp))) {
      auto action_name              = action_map.at(":action").toString();
      auto action_precondition_sexp = action_map.at(":precondition");
      auto untyped_params =
      fplus::transform(convert_sexp, action_map.at(":parameters").value.sexp);

      Map<Str, Str> action_params;
      Vec<Str> param_names;

      for (auto const& param : untyped_params) {
        if (param[0] == '-') {
          auto type_pairs =
          fplus::zip(param_names, fplus::repeat(param_names.size(), Vec<Str>(1, param)));
          action_params.insert(type_pairs.begin(), type_pairs.end());
          param_names.clear();
        } else {
          param_names.push_back(param);
        }
      }

      if (!param_names.empty()) {
        auto type_pairs =
        fplus::zip(param_names, fplus::repeat(param_names.size(), Vec<Str>(1, "obj")));
        action_params.insert(type_pairs.begin(), type_pairs.end());
      }

      auto action_effect = parse_effect(action_map.at(":effect"), discrete_predicates);
      auto action_precondition =
      fplus::transform(empty_helper,
                       parse_formula_sexp(action_precondition_sexp, discrete_predicates));
      actions.emplace_back(
      std::make_shared<Action>(action_name, action_precondition, action_params, action_effect));
      log->debug("Got action: {}", action_name);
    }

    log->debug("Constructed {} actions", actions.size());

    // Put it all together in a domain object
    domain_result.emplace(typed_objects,
                          object_count,
                          actions,
                          discrete_dimensions,
                          eqclass_dimensions,
                          kinematic_effects,
                          predicate_filename);

    log->debug("Constructed domain");
  } catch (std::out_of_range& e) {
    domain_result = std::nullopt;
    log->error("Failed to construct domain!");
  }

  // Construct the initial conditions
  std::optional<Initial> init_result;
  try {
    // We just look for all the discrete dimensions mentioned in the initial condition
    auto discrete_inits = fplus::keep_if(
    [&](auto const& x) -> bool {
      return discrete_dimensions.find(x) != discrete_dimensions.end();
    },
    fwd::apply(problem_sexp->getChildByPath("define/:init")->value.sexp,
               fwd::transform(dimension_of_ground_atom)));

    log->debug("Initial atoms: {}", fplus::join(separator, discrete_inits));

    // TODO(Wil): It might be a better idea to not have to construct an unordered_set from a
    // vector...
    init_result = fplus::convert_container<Set<Str>>(discrete_inits);
    log->debug("Constructed initial state");
    log->debug("Initial atoms: {}", fplus::show_cont(*init_result));
  } catch (std::out_of_range& e) {
    init_result = std::nullopt;
    log->error("Failed to construct initial state!");
  }

  // Construct the goal
  std::optional<Goal> goal_result;
  try {
    auto bodies = fplus::transform(
    empty_helper,
    parse_formula_sexp(problem_sexp->getChildByPath("define/:goal")->value.sexp[1],
                       discrete_predicates));

    log->info("Goal is: {}", bodies);
    // NOTE: We bind all the objects because a goal is a ground formula and could reference any
    // of them
    Vec<Str> all_object_names;
    for (const auto& type_obj : typed_objects) {
      std::copy(type_obj.second.begin(),
                type_obj.second.end(),
                std::back_inserter(all_object_names));
    }

    goal_result = fwd::apply(
    bodies,
    fwd::transform_with_idx([&](std::size_t i, const StrConfiguration& config) -> Configuration {
      return std::make_pair(Formula(fmt::format("goal_{}", i), all_object_names, config.first),
                            fplus::pairs_to_map<Map<Str, bool>>(fplus::transform(
                            fwd::transform_fst(fwd::join(Str("_"))), config.second)));
    }),
    fwd::keep_if([](const auto& branch) {
      // log->info("Branch has {} and {}", branch.first.body, branch.second);
      // log->info("Branch has {} and {}",
      //           branch.first.body != "True()" && branch.first.body != "False()",
      //           !branch.second.empty());
      return !branch.second.empty() ||
             (branch.first.body != "True()" && branch.first.body != "False()");
    }));
    log->info("Constructed goal with {} possible configuration sets", goal_result->size());
  } catch (std::out_of_range& e) {
    goal_result = std::nullopt;
    log->error("Failed to construct goal!");
  }

  return {std::move(domain_result), init_result, goal_result};
}
}  // namespace input::specification
