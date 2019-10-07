#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include "fplus/fplus.hpp"

namespace input {
namespace specification {
  using LuaRef = int;
  struct Formula {
    /// The basic wrapper for a formula. Takes in a n-length vector of the whole state.
    static constexpr auto FORMULA_FN_FMT =
    R"FUNCFORM(function {}_formula(state_vector)
  {}
  return {}
end)FUNCFORM";
    static constexpr auto FORMULA_BODY_FMT = "local {} = generate_objects(state_vector, {})";

    static constexpr auto GRADIENT_FN_FMT =
    R"FUNCFORM({}_gradient = diff.gradientf({}_formula, {}))FUNCFORM";

    Formula(const std::string& name,
            const std::vector<std::string>& bindings,
            const std::string& body)
    : normal_def(make_normal_definition(name, bindings, body))
    , name(name)
    , body(body)
    , normal_fn_name(fmt::format("{}_formula", name))
    , grad_fn_name(fmt::format("{}_gradient", name)) {}
    const std::string normal_def;
    const std::string name;
    const std::string body;

    // This is because a single formula may be loaded into multiple environments
    std::unordered_map<std::string, LuaRef> gradient_fn;
    std::unordered_map<std::string, LuaRef> normal_fn;
    const std::string normal_fn_name;
    const std::string grad_fn_name;

   private:
    std::string make_normal_definition(const std::string& name,
                                       const std::vector<std::string>& bindings,
                                       const std::string& body) {
      auto binding_symbols = fplus::join(std::string(", "), bindings);
      auto lua_body        = (body == "True()" || body == "False()") ?
                      "" :
                      fmt::format(FORMULA_BODY_FMT, binding_symbols, bindings);
      auto def = fmt::format(FORMULA_FN_FMT, name, lua_body, body);
      return def;
    }
  };
}  // namespace specification
}  // namespace input
