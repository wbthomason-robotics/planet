#pragma once
#ifndef PREDICATE_IMPL_HH
#define PREDICATE_IMPL_HH

#include <iostream>
#include <mpark/patterns.hpp>
#include <optional>
#include <type_traits>

#include "common.hh"

// clang-format off
#include "lua.hpp"
#include "lualib.h"
#include "lauxlib.h"
// clang-format on

#include "specification.hh"
#include "scenegraph.hh"
#include "object.hh"
#include "autodiff.hh"
#include "predicate-types.hh"

namespace symbolic::predicate {
namespace {
  namespace obj = structures::object;

  Vec<addn::DN> dn_block;
  template <typename T> void return_userdata(lua_State* L, const T* data) {
    const T** p = static_cast<const T**>(lua_newuserdata(L, sizeof(T**)));
    *p          = data;
    get_metatable<typename std::remove_const<T>::type>(L);
    lua_setmetatable(L, -2);
  }

  template <typename T> static int obj_iter(lua_State* L) {
    Vec<T>* data       = static_cast<Vec<T>*>(lua_touserdata(L, lua_upvalueindex(2)));
    const int next_idx = lua_tonumber(L, lua_upvalueindex(1));
    if (next_idx >= data->size()) {
      return 0;
    }

    return_userdata(L, &(*data)[next_idx]);
    return 1;
  }

  template <typename T> void return_iter(lua_State* L, const Vec<T>* data) {
    lua_pushlightuserdata(L, static_cast<void*>(const_cast<Vec<T>*>(data)));
    lua_pushinteger(L, 0);
    lua_pushcclosure(L, obj_iter<T>, 2);
  }

  void fresh_dn(lua_State* L, const addn::DN& val) {
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*)));
    dn_block.emplace_back(val);
    *result     = &dn_block.back();
    luaL_getmetatable(L, "DN");
    lua_setmetatable(L, -2);
  }

  // Metatable index functions
  template <typename N>
  bool handle_link_key(lua_State* L, const Str& key, const LuaLink<N>* const link);
  template <>
  bool
  handle_link_key<addn::DN>(lua_State* L, const Str& key, const LuaLink<addn::DN>* const link) {
    if (key == "px") {
      fresh_dn(L, link->px);
    } else if (key == "py") {
      fresh_dn(L, link->py);
    } else if (key == "pz") {
      fresh_dn(L, link->pz);
    } else if (key == "rx") {
      fresh_dn(L, link->rx);
    } else if (key == "ry") {
      fresh_dn(L, link->ry);
    } else if (key == "rz") {
      fresh_dn(L, link->rz);
    } else if (key == "rw") {
      fresh_dn(L, link->rw);
    } else {
      return false;
    }

    return true;
  }

  template <>
  bool handle_link_key<double>(lua_State* L, const Str& key, const LuaLink<double>* const link) {
    if (key == "px") {
      lua_pushnumber(L, link->px);
    } else if (key == "py") {
      lua_pushnumber(L, link->py);
    } else if (key == "pz") {
      lua_pushnumber(L, link->pz);
    } else if (key == "rx") {
      lua_pushnumber(L, link->rx);
    } else if (key == "ry") {
      lua_pushnumber(L, link->ry);
    } else if (key == "rz") {
      lua_pushnumber(L, link->rz);
    } else if (key == "rw") {
      lua_pushnumber(L, link->rw);
    } else if (key == "name") {
      lua_pushstring(L, link->name);
    } else {
      return false;
    }

    return true;
  }

  template <typename N> static int link_index(lua_State* L) {
    auto link = static_cast<LuaLink<N>*>(lua_touserdata(L, 1));
    Str key   = luaL_checkstring(L, 2);
    if (!handle_link_key(L, key, link)) {
      luaL_error(L, "Unknown key for link %s: %s", link->name, key.c_str());
    }

    return 1;
  }

  template <typename N> static int object_index(lua_State* L) {
    auto object = static_cast<LuaObject<N>*>(lua_touserdata(L, 1));
    Str key     = luaL_checkstring(L, 2);
    if (key == "sssp") {
      if (object->object_data->stable_face) {
        return_userdata(L, &object->sssp);
      } else {
        luaL_error(L, "Object %s has no SSSP!", object->name);
      }
    } else if (key == "sop") {
      return_iter(L, &object->object_data->stable_poses);
    } else if (key == "grasps") {
      return_iter(L, &object->object_data->grasps);
    } else if (!handle_link_key(L, key, object)) {
      luaL_error(L, "Unknown key for object %s: %s", object->name, key.c_str());
    }

    return 1;
  }
}  // namespace

void load_metatables(lua_State* L);
void load_math_lib(lua_State* L);
void init_dn_block();
void clear_dn_block();
}  // namespace symbolic::predicate
#endif
