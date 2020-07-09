#pragma once
#ifndef PREDICATE_TYPES_HH
#define PREDICATE_TYPES_HH
// clang-format off
#include "lua.hpp"
#include "lualib.h"
#include "lauxlib.h"
// clang-format on

#include "object.hh"

namespace {
namespace obj = structures::object;
template <typename N> struct LuaLink {
  N px;
  N py;
  N pz;
  N rx;
  N ry;
  N rz;
  N rw;
  const char* name;
};

struct LuaSSSP {
  const obj::StableFace* sssp;
};

template <typename N> struct LuaObject : public LuaLink<N> {
  const structures::object::Object* object_data;
  LuaSSSP sssp;
};

template <typename T> void get_metatable(lua_State* L);
template <> void get_metatable<addn::DN>(lua_State* L) { luaL_getmetatable(L, "DN"); }

template <> void get_metatable<obj::StableRegion>(lua_State* L) {
  luaL_getmetatable(L, "StableRegion");
}

template <> void get_metatable<LuaSSSP>(lua_State* L) {
  luaL_getmetatable(L, "LuaSSSP");
}

template <> void get_metatable<obj::Bounds>(lua_State* L) { luaL_getmetatable(L, "Bounds"); }

template <> void get_metatable<obj::StablePlane>(lua_State* L) {
  luaL_getmetatable(L, "StablePlane");
}

template <> void get_metatable<obj::Grasp>(lua_State* L) { luaL_getmetatable(L, "Grasp"); }

template <> void get_metatable<Vector3r>(lua_State* L) { luaL_getmetatable(L, "Vector3r"); }

template <> void get_metatable<obj::StablePose>(lua_State* L) {
  luaL_getmetatable(L, "StablePose");
}

template <> void get_metatable<Eigen::Quaterniond>(lua_State* L) {
  luaL_getmetatable(L, "Quaterniond");
}

template <> void get_metatable<LuaLink<double>>(lua_State* L) {
  luaL_getmetatable(L, "LuaLinkDouble");
}

template <> void get_metatable<LuaLink<addn::DN>>(lua_State* L) {
  luaL_getmetatable(L, "LuaLinkDN");
}

template <> void get_metatable<LuaObject<double>>(lua_State* L) {
  luaL_getmetatable(L, "LuaObjectDouble");
}

template <> void get_metatable<LuaObject<addn::DN>>(lua_State* L) {
  luaL_getmetatable(L, "LuaObjectDN");
}
}  // namespace
#endif
