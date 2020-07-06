#pragma once
#ifndef PREDICATE_HH
#define PREDICATE_HH
#include <memory>
#include <optional>
#include <utility>

#include "common.hh"

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
#include "scenegraph.hh"
#include "object.hh"

namespace symbolic::predicate {
constexpr char GRADIENT_PRELUDE_PATH[] = "lua/autodiff_prelude.lua";
constexpr char UNSAT_PRELUDE_PATH[]    = "lua/unsat_prelude.lua";
constexpr char BOOL_PRELUDE_PATH[]     = "lua/normal_prelude.lua";

constexpr int OBJECT_DATA_SIZE         = 8;
constexpr int OBJECT_METADATA_SIZE     = 3;
constexpr int GRASP_FRAME_DATA_SIZE    = 7;
constexpr int GRASP_TEMPLATE_DATA_SIZE = 2;
constexpr char const* OBJECT_FIELD_NAMES[]{"px", "py", "pz", "rx", "ry", "rz", "rw"};
constexpr int OBJ_STATE_SIZE = OBJECT_DATA_SIZE + OBJECT_METADATA_SIZE;

extern const structures::object::ObjectSet* objects;
extern const structures::object::ObjectSet* obstacles;

namespace ob = ompl::base;
std::vector<double> generate_state_vector(const ob::StateSpace* const space,
                                          const ob::State* const state,
                                          const bool base_movable);

namespace spec = input::specification;

struct LuaEnvData {
  explicit LuaEnvData(const Str& name, const Str& prelude_filename = Str());
  bool load_predicates(const Str& predicates_filename) const;
  bool load_formula(spec::Formula* formula) const;
  void teardown_world(const spec::Formula& formula) const;
  void cleanup() const;

  template <typename T> T call_formula(const spec::Formula& formula) const;
  template <> addn::DN call_formula(const spec::Formula& formula) const {
    const auto start_top = lua_gettop(L);

    // Run the formula
    const auto err_func_idx = call_helper(formula, start_top);

    // Get the return value
    lua_pushstring(L, "_v");
    lua_gettable(L, -2);
    const double v = lua_tonumber(L, -1);

    lua_pushstring(L, "_a");
    lua_gettable(L, -3);
    const double a = lua_tonumber(L, -1);

    // Clean up
    cleanup_helper(start_top, err_func_idx);

    return {v, a};
  }

  template <> bool call_formula(const spec::Formula& formula) const {
    const auto start_top = lua_gettop(L);

    // Run the formula
    const auto err_func_idx = call_helper(formula, start_top);

    // Get the return value
    const double v = lua_toboolean(L, -1);

    // Clean up
    cleanup_helper(start_top, err_func_idx);

    return v;
  }

  template <typename T> void update_object(const Str& name, const Transform3<T>& pose) const {
    const auto& object_idx = binding_idx.find(name);
    if (object_idx != binding_idx.end()) {
      const auto idx = object_idx->second;
      update_field(idx, OBJECT_FIELD_NAMES[0], pose.translation().x());
      update_field(idx, OBJECT_FIELD_NAMES[1], pose.translation().y());
      update_field(idx, OBJECT_FIELD_NAMES[2], pose.translation().z());
      Eigen::Quaternion<T> rotation(pose.linear());
      update_field(idx, OBJECT_FIELD_NAMES[3], rotation.x());
      update_field(idx, OBJECT_FIELD_NAMES[4], rotation.y());
      update_field(idx, OBJECT_FIELD_NAMES[5], rotation.z());
      update_field(idx, OBJECT_FIELD_NAMES[6], rotation.w());
    }
  }

  template <typename T>
  void setup_world(const spec::Formula& formula,
                   const Map<Str, Str>& bindings,
                   structures::scenegraph::Graph* const sg) {
    binding_idx.clear();
    binding_idx.reserve(bindings.size());
    // Make tables for each bound object in the same order as the formula will request them
    for (int i = formula.bindings.size() - 1; i >= 0; --i) {
      // NOTE(Wil): Are there variables in the formula bindings which may not be in the bindings
      // map? e.g. global names?
      auto name                = formula.bindings[i];
      const auto& binding_iter = bindings.find(name);
      if (binding_iter != bindings.end()) {
        name = binding_iter->second;
      }

      const auto idx = lua_gettop(L) + 1;
      binding_idx.insert({name, idx});
      const auto& node = sg->find(name);
      create_object<T>(node);
    }
  }

  const Str name;

 protected:
  std::shared_ptr<spdlog::logger> log;
  lua_State* L;
  void update_field(const int idx, const char* const field_name, const double value) const {
    lua_pushstring(L, field_name);
    lua_pushnumber(L, value);
    lua_rawset(L, idx);
  }

  void update_field(const int idx, const char* const field_name, const addn::DN value) const {
    lua_pushstring(L, field_name);
    lua_pushstring(L, field_name);
    lua_rawget(L, idx);
    lua_pushstring(L, "_v");
    lua_pushnumber(L, value.v);
    lua_settable(L, -3);
    lua_pushstring(L, "_a");
    lua_pushnumber(L, value.a);
    lua_settable(L, -3);
    lua_rawset(L, idx);
  }

  Map<Str, int> binding_idx;
  template <typename T> void create_object(const structures::scenegraph::Node& node) const {
    if (!node.is_object && !node.is_obstacle) {
      lua_createtable(L, 0, OBJECT_DATA_SIZE);
    } else {
      structures::object::Object* obj_data = nullptr;
      if (node.is_obstacle) {
        obj_data = obstacles->at(node.name).get();
      } else {
        obj_data = objects->at(node.name).get();
      }

      lua_createtable(L, 0, OBJ_STATE_SIZE);
      setup_metadata(obj_data);
    }

    lua_pushstring(L, "name");
    lua_pushstring(L, node.name.c_str());
    lua_rawset(L, -3);
    setup_object<T>();
  }

  template <typename T> void setup_object() const;
  template <> void setup_object<addn::DN>() const {
    for (int i = 0; i < 7; ++i) {
      lua_pushstring(L, OBJECT_FIELD_NAMES[i]);
      lua_getglobal(L, "dn");
      lua_pushnumber(L, 0.0);
      lua_pushnumber(L, 0.0);
      lua_pcall(L, 2, 1, 0);
      lua_rawset(L, -3);
    }
  }

  template <> void setup_object<double>() const {
    for (int i = 0; i < 7; ++i) {
      lua_pushstring(L, OBJECT_FIELD_NAMES[i]);
      lua_pushnumber(L, 0.0);
      lua_rawset(L, -3);
    }
  }

  void setup_metadata(const structures::object::Object* const obj_data) const;

 private:
  int call_helper(const spec::Formula& formula, const int start_top) const;
  void cleanup_helper(const int start_top, const int err_func_idx) const;
};

template <typename T> void make_base_tf(const Vec<T>& state, Transform3<T>& base_tf) {
  base_tf.translation().x() = state[0];
  base_tf.translation().y() = state[1];
  base_tf.translation().z() = state[2];

  const auto base_rotation = state[3];
  base_tf.linear() =
  Eigen::Quaternion<T>(Eigen::AngleAxis<T>(base_rotation, Eigen::Matrix<T, 3, 1>::UnitZ()))
  .normalized()
  .toRotationMatrix();
}

template <typename CallType> struct LuaEnv : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
};

template <> struct LuaEnv<bool> : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
  bool call(const spec::Formula& formula,
            const Map<Str, Str>& bindings,
            structures::scenegraph::Graph* const sg,
            const ob::State* const state,
            const ob::StateSpace* const space,
            const bool base_movable);
};

template <> struct LuaEnv<double> : public LuaEnvData {
  explicit LuaEnv(const Str& name, const Str& prelude_filename = Str())
  : LuaEnvData(name, prelude_filename) {}
};
}  // namespace symbolic::predicate
#endif
