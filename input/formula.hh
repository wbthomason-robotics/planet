#pragma once
#ifndef FORMULA_HH
#define FORMULA_HH
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "common.hh"
#include "fplus/fplus.hpp"

namespace input::specification {
using LuaRef = int;
struct Formula {
  /// The basic wrapper for a formula. Takes in a n-length vector of the whole state.
  static constexpr auto FORMULA_FN_FMT =
  R"FUNCFORM(function {}_formula(...)
  {}
  return {}
end)FUNCFORM";
  static constexpr auto FORMULA_BODY_FMT = "local {} = ...";

  Formula(const Str& name, const Vec<Str>& bindings, const Str& body)
  : normal_def(make_normal_definition(name, bindings, body))
  , name(name)
  , body(body)
  , normal_fn_name(fmt::format("{}_formula", name))
  , grad_fn_name(fmt::format("{}_gradient", name))
  , bindings(bindings) {}
  const Str normal_def;
  const Str name;
  const Str body;

  // This is because a single formula may be loaded into multiple environments
  Map<Str, LuaRef> normal_fn;
  const Str normal_fn_name;
  const Str grad_fn_name;
  const Vec<Str> bindings;

 private:
  Str make_normal_definition(const Str& name, const Vec<Str>& bindings, const Str& body) {
    auto binding_symbols = fplus::join(Str(", "), bindings);
    const auto lua_initialization =
    (body == "True()" || body == "False()") ? "" : fmt::format(FORMULA_BODY_FMT, binding_symbols);
    return fmt::format(FORMULA_FN_FMT, name, lua_initialization, body);
  }
};
}  // namespace input::specification
#endif
