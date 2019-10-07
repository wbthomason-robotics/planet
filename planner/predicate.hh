#pragma once
#ifndef PREDICATE_HH
#define PREDICATE_HH
#include "common.hh"

#include <memory>
#include <optional>
#include <utility>

#include <armadillo>

// clang-format off
#include "lua.hpp"
#include "lualib.h"
#include "lauxlib.h"
// clang-format on

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

#include "specification.hh"

namespace symbolic::predicate {
constexpr char GRADIENT_PRELUDE_PATH[] = "lua/autodiff_prelude.lua";
constexpr char UNSAT_PRELUDE_PATH[]    = "lua/unsat_prelude.lua";
constexpr char BOOL_PRELUDE_PATH[]     = "lua/normal_prelude.lua";

namespace ob = ompl::base;
std::vector<double> generate_state_vector(const ob::StateSpace* const space,
                                          const ob::State* const state,
                                          const bool base_movable);

namespace spec = input::specification;
struct LuaEnvData {
  explicit LuaEnvData(const Str& name, const Str& prelude_filename = Str());
  bool load_predicates(const Str& predicates_filename) const;
  bool load_formula(spec::Formula* formula) const;
  bool load_gradient(spec::Formula* formula, const int num_dims) const;
  std::optional<std::pair<arma::vec, double>>
  call_gradient(const spec::Formula* formula, const arma::vec& point) const;
  void set_bindings(const std::unordered_map<Str, Str>& bindings) const;
  void set_universe(void* universe) const;
  void cleanup() const;
  const Str name;

 protected:
  std::shared_ptr<spdlog::logger> log;
  lua_State* L;
  bool call_lua(const Str& fn_name,
                const ob::StateSpace* const space,
                const ob::State* const state,
                const bool base_movable) const;
};

template <typename CallType> struct LuaEnv : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
  CallType operator()(const Str& fn_name, ob::State* state);
};

template <> struct LuaEnv<bool> : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
  bool operator()(const Str& fn_name,
                  const ob::StateSpace* const space,
                  const ob::State* const state,
                  const bool base_movable) const {
    if (call_lua(fn_name, space, state, base_movable)) {
      auto result = lua_toboolean(L, -1);
      lua_pop(L, 1);
      return result;
    }

    return false;
  }
};

template <> struct LuaEnv<double> : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
  double operator()(const Str& fn_name,
                    const ob::StateSpace* const space,
                    const ob::State* const state,
                    const bool base_movable) const {
    if (call_lua(fn_name, space, state, base_movable)) {
      auto result = lua_tonumber(L, -1);
      lua_pop(L, 1);
      return result;
    }

    return std::numeric_limits<double>::infinity();
  }
};
}  // namespace symbolic::predicate
#endif
