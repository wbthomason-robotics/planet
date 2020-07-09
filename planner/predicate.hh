#pragma once
#ifndef PREDICATE_HH
#define PREDICATE_HH
#include <memory>

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
#include "predicate-types.hh"

namespace symbolic::predicate {
constexpr char GRADIENT_PRELUDE_PATH[] = "lua/autodiff_prelude.lua";
constexpr char UNSAT_PRELUDE_PATH[]    = "lua/unsat_prelude.lua";
constexpr char BOOL_PRELUDE_PATH[]     = "lua/normal_prelude.lua";

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
    const auto start_top    = lua_gettop(L);
    const auto err_func_idx = call_helper(formula, start_top);
    const auto& result      = **static_cast<addn::DN**>(lua_touserdata(L, -1));
    cleanup_helper(start_top, err_func_idx);
    return result;
  }

  template <> bool call_formula(const spec::Formula& formula) const {
    const auto start_top    = lua_gettop(L);
    const auto err_func_idx = call_helper(formula, start_top);
    const auto result       = lua_toboolean(L, -1);
    cleanup_helper(start_top, err_func_idx);
    return result;
  }

  template <typename T> void update_object(const Str& name, const Transform3<T>& pose) const {
    const auto& object_iter = binding_idx.find(name);
    if (object_iter != binding_idx.end()) {
      Eigen::Quaternion<T> rotation(pose.linear());
      auto object = static_cast<LuaLink<T>*>(object_iter->second);
      object->px  = pose.translation().x();
      object->py  = pose.translation().y();
      object->pz  = pose.translation().z();
      object->rx  = rotation.x();
      object->ry  = rotation.y();
      object->rz  = rotation.z();
      object->rw  = rotation.w();
    }
  }

  template <typename T>
  void setup_world(const spec::Formula& formula,
                   const Map<Str, Str>& bindings,
                   structures::scenegraph::Graph* const sg) {
    binding_idx.clear();
    binding_idx.reserve(bindings.size());
    // Make userdata for each bound object in the same order as the formula will request them
    auto binding = formula.bindings.rbegin();
    for (; binding != formula.bindings.rend(); ++binding) {
      // NOTE(Wil): Are there variables in the formula bindings which may not be in the bindings
      // map? e.g. global names?
      auto name                = *binding;
      const auto& binding_iter = bindings.find(name);
      if (binding_iter != bindings.end()) {
        name = binding_iter->second;
      }

      const auto& node = sg->find(name);
      auto object      = create_object<T>(node);
      binding_idx.insert({name, object});
    }
  }

  const Str name;

 protected:
  std::shared_ptr<spdlog::logger> log;
  lua_State* L;
  Map<Str, void*> binding_idx;
  template <typename T> LuaLink<T>* create_object(const structures::scenegraph::Node& node) const {
    LuaLink<T>* link = nullptr;
    if (!node.is_object && !node.is_obstacle) {
      link = static_cast<LuaLink<T>*>(lua_newuserdata(L, sizeof(LuaLink<T>)));
      get_metatable<LuaLink<T>>(L);
      lua_setmetatable(L, -2);
    } else {
      structures::object::Object* obj_data = nullptr;
      if (node.is_obstacle) {
        obj_data = obstacles->at(node.name).get();
      } else {
        obj_data = objects->at(node.name).get();
      }

      LuaObject<T>* obj = static_cast<LuaObject<T>*>(lua_newuserdata(L, sizeof(LuaObject<T>)));
      get_metatable<LuaObject<T>>(L);
      lua_setmetatable(L, -2);
      obj->object_data = obj_data;
      if (obj_data->stable_face) {
        obj->sssp.sssp = &*obj->object_data->stable_face;
      } else {
        obj->sssp.sssp = nullptr;
      }

      link = obj;
    }

    setup_link(link, node.name.c_str());
    return link;
  }

  template <typename T> void setup_link(LuaLink<T>* link, const char* name) const {
    link->name = name;
    link->px   = 0.0;
    link->py   = 0.0;
    link->pz   = 0.0;
    link->rx   = 0.0;
    link->ry   = 0.0;
    link->rz   = 0.0;
    link->rw   = 0.0;
  }

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
