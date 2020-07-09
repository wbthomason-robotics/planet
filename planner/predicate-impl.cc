#include <type_traits>
#include <variant>

#include "autodiff.hh"
#include "fmt/format.h"
#include "predicate-impl.hh"

namespace symbolic::predicate {
namespace {
  static int StableRegion_index(lua_State* L) {
    auto region = *static_cast<obj::StableRegion**>(lua_touserdata(L, 1));
    Str key     = luaL_checkstring(L, 2);
    if (key == "x") {
      return_userdata(L, &region->x);
    } else if (key == "y") {
      return_userdata(L, &region->y);
    } else if (key == "z") {
      return_userdata(L, &region->z);
    } else {
      luaL_error(L, "Unknown key for region: %s", key.c_str());
    }

    return 1;
  }

  static int LuaSSSP_index(lua_State* L) {
    auto sssp              = *static_cast<LuaSSSP**>(lua_touserdata(L, 1));
    Str key                = luaL_checkstring(L, 2);
    const bool want_region = key == "region";
    const bool want_plane  = key == "plane";
    if (!want_plane && !want_region) {
      luaL_error(L, "Unknown key for SSSP: %s", key.c_str());
    } else {
      std::visit(
      [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, obj::StableRegion>) {
          if (want_region) {
            return_userdata(L, &arg);
          } else {
            lua_pushnil(L);
          }
        } else if constexpr (std::is_same_v<T, obj::StablePlane>) {
          if (want_plane) {
            return_iter(L, &arg);
          } else {
            lua_pushnil(L);
          }
        }
      },
      *(sssp->sssp));
    }

    return 1;
  }

  static int Bounds_index(lua_State* L) {
    auto bounds = *static_cast<obj::Bounds**>(lua_touserdata(L, 1));
    Str key     = luaL_checkstring(L, 2);
    if (key == "low") {
      lua_pushnumber(L, bounds->low);
    } else if (key == "high") {
      lua_pushnumber(L, bounds->high);
    } else {
      luaL_error(L, "Unknown key for bounds: %s", key.c_str());
    }

    return 1;
  }

  static int Grasp_index(lua_State* L) {
    auto grasp = *static_cast<obj::Grasp**>(lua_touserdata(L, 1));
    Str key    = luaL_checkstring(L, 2);
    if ((std::holds_alternative<obj::ContinuousGrasp>(*grasp) && key == "discrete") ||
        (std::holds_alternative<obj::DiscreteGrasp>(*grasp) && key == "continuous")) {
      lua_pushnil(L);
    } else if (key == "pos") {
      std::visit([=](auto&& arg) { return_userdata(L, &arg.translation); }, *grasp);
    } else if (key == "rot") {
      std::visit([=](auto&& arg) { return_userdata(L, &arg.rotation); }, *grasp);
    } else if (std::holds_alternative<obj::ContinuousGrasp>(*grasp) && key == "axis") {
      return_userdata(L, &std::get<obj::ContinuousGrasp>(*grasp).axis);
    } else {
      luaL_error(L, "Unknown key for grasp: %s", key.c_str());
    }

    return 1;
  }

  static int Vector3r_index(lua_State* L) {
    auto vec = *static_cast<Vector3r**>(lua_touserdata(L, 1));
    Str key  = luaL_checkstring(L, 2);
    if (key == "x") {
      lua_pushnumber(L, vec->x());
    } else if (key == "y") {
      lua_pushnumber(L, vec->y());
    } else if (key == "z") {
      lua_pushnumber(L, vec->z());
    } else {
      luaL_error(L, "Unknown key for vector3r: %s", key.c_str());
    }

    return 1;
  }

  static int Quaterniond_index(lua_State* L) {
    auto quat = *static_cast<Eigen::Quaterniond**>(lua_touserdata(L, 1));
    Str key   = luaL_checkstring(L, 2);
    if (key == "x") {
      lua_pushnumber(L, quat->x());
    } else if (key == "y") {
      lua_pushnumber(L, quat->y());
    } else if (key == "z") {
      lua_pushnumber(L, quat->z());
    } else if (key == "w") {
      lua_pushnumber(L, quat->w());
    } else {
      luaL_error(L, "Unknown key for quaternion: %s", key.c_str());
    }

    return 1;
  }

  // Metatable functions for DNs and math functions
  // static int dn_gc(lua_State* L) {
  //   auto dn = *static_cast<addn::DN**>(lua_touserdata(L, 1));
  //   delete dn;
  //   return 0;
  // }

  static int dn_unm(lua_State* L) {
    auto dn     = static_cast<addn::DN**>(lua_touserdata(L, 1));
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*)));
    dn_block.emplace_back(-**dn);
    *result = &dn_block.back();
    luaL_getmetatable(L, "DN");
    lua_setmetatable(L, -2);
    return 1;
  }

#define DN_DISPATCH(both, only_a, only_b)                                             \
  const bool dn_1 = lua_isuserdata(L, 1);                                             \
  const bool dn_2 = lua_isuserdata(L, 2);                                             \
  if (dn_1 && dn_2) {                                                                 \
    auto a = static_cast<addn::DN**>(lua_touserdata(L, 1));                           \
    auto b = static_cast<addn::DN**>(lua_touserdata(L, 2));                           \
    dn_block.emplace_back(both);                                                      \
  } else if (dn_1 && !dn_2) {                                                         \
    auto a = static_cast<addn::DN**>(lua_touserdata(L, 1));                           \
    auto b = lua_tonumber(L, 2);                                                      \
    dn_block.emplace_back(only_a);                                                    \
  } else if (!dn_1 && dn_2) {                                                         \
    auto a = lua_tonumber(L, 1);                                                      \
    auto b = static_cast<addn::DN**>(lua_touserdata(L, 2));                           \
    dn_block.emplace_back(only_b);                                                    \
  } else {                                                                            \
    luaL_error(L, "DN dispatch called for two doubles, which should be impossible!"); \
  }

#define DN_BINOP(name, op)                                                        \
  static int dn_##name(lua_State* L) {                                            \
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*))); \
    DN_DISPATCH(**a op** b, **a op b, a op** b);                                  \
    *result = &dn_block.back();                                                   \
    luaL_getmetatable(L, "DN");                                                   \
    lua_setmetatable(L, -2);                                                      \
    return 1;                                                                     \
  }

  DN_BINOP(add, +)
  DN_BINOP(sub, -)
  DN_BINOP(mul, *)
  DN_BINOP(div, /)

  static int dn_pow(lua_State* L) {
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*)));
    DN_DISPATCH(addn::pow(**a, **b), addn::pow(**a, b), addn::pow(a, **b));
    *result = &dn_block.back();
    luaL_getmetatable(L, "DN");
    lua_setmetatable(L, -2);
    return 1;
  }

  DN_BINOP(eq, ==)
  DN_BINOP(lt, <)
  DN_BINOP(le, <=)

#define DN_MATH_UN_FN(name)                                                       \
  static int math_##name(lua_State* L) {                                          \
    auto a      = static_cast<addn::DN**>(lua_touserdata(L, 1));                  \
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*))); \
    dn_block.emplace_back(addn::name(**a));                                       \
    *result = &dn_block.back();                                                   \
    luaL_getmetatable(L, "DN");                                                   \
    lua_setmetatable(L, -2);                                                      \
    return 1;                                                                     \
  }

#define DN_MATH_BIN_FN(name)                                                      \
  static int math_##name(lua_State* L) {                                          \
    auto result = static_cast<addn::DN**>(lua_newuserdata(L, sizeof(addn::DN*))); \
    DN_DISPATCH(addn::name(**a, **b), addn::name(**a, b), addn::name(a, **b));    \
    *result = &dn_block.back();                                                   \
    luaL_getmetatable(L, "DN");                                                   \
    lua_setmetatable(L, -2);                                                      \
    return 1;                                                                     \
  }

  DN_MATH_UN_FN(abs)
  DN_MATH_UN_FN(acos)
  DN_MATH_UN_FN(asin)
  DN_MATH_UN_FN(atan)
  DN_MATH_UN_FN(cos)
  DN_MATH_UN_FN(exp)
  DN_MATH_UN_FN(log)
  DN_MATH_UN_FN(sin)
  DN_MATH_UN_FN(sqrt)
  DN_MATH_UN_FN(tan)
  DN_MATH_UN_FN(ceil)
  DN_MATH_UN_FN(floor)
  DN_MATH_UN_FN(cosh)
  DN_MATH_UN_FN(sinh)
  DN_MATH_UN_FN(tanh)

  DN_MATH_BIN_FN(max)
  DN_MATH_BIN_FN(min)
  DN_MATH_BIN_FN(atan2)
  DN_MATH_BIN_FN(pow)

  static int dn_tostring(lua_State* L) {
    auto a     = *static_cast<addn::DN**>(lua_touserdata(L, 1));
    auto a_str = fmt::format("(v = {}, a = {})", a->v, a->a);
    lua_pushstring(L, a_str.c_str());
    return 1;
  }

  static const struct luaL_Reg dn_mt[] = {{"__unm", dn_unm},
                                          {"__add", dn_add},
                                          {"__sub", dn_sub},
                                          {"__mul", dn_mul},
                                          {"__div", dn_div},
                                          {"__pow", dn_pow},
                                          {"__eq", dn_eq},
                                          {"__lt", dn_lt},
                                          {"__le", dn_le},
                                          {"__tostring", dn_tostring},
                                          {NULL, NULL}};

  // static const struct luaL_Reg gc_dn_mt[] = {{"__gc", dn_gc},
  //                                            {"__unm", dn_unm},
  //                                            {"__add", dn_add},
  //                                            {"__sub", dn_sub},
  //                                            {"__mul", dn_mul},
  //                                            {"__div", dn_div},
  //                                            {"__pow", dn_pow},
  //                                            {"__eq", dn_eq},
  //                                            {"__lt", dn_lt},
  //                                            {"__le", dn_le},
  //                                            {"__tostring", dn_tostring},
  //                                            {NULL, NULL}};

  static const struct luaL_Reg mathlib[] = {
  {"abs", math_abs},     {"acos", math_acos},   {"asin", math_asin}, {"atan", math_atan},
  {"atan2", math_atan2}, {"ceil", math_ceil},   {"cos", math_cos},   {"cosh", math_cosh},
  {"exp", math_exp},     {"floor", math_floor}, {"log", math_log},   {"max", math_max},
  {"min", math_min},     {"pow", math_pow},     {"sin", math_sin},   {"sinh", math_sinh},
  {"sqrt", math_sqrt},   {"tan", math_tan},     {"tanh", math_tanh}, {NULL, NULL}};

#define MAKE_MT(name)                 \
  luaL_newmetatable(L, #name);        \
  lua_pushcfunction(L, name##_index); \
  lua_setfield(L, -2, "__index");
}  // namespace

void load_metatables(lua_State* L) {
  // DN
  luaL_newmetatable(L, "DN");
  luaL_register(L, NULL, dn_mt);

  // luaL_newmetatable(L, "GCDN");
  // luaL_register(L, NULL, gc_dn_mt);

  // StableRegion
  MAKE_MT(StableRegion);

  // Bounds
  MAKE_MT(Bounds);

  // LuaSSSP
  MAKE_MT(LuaSSSP);

  // Grasp
  MAKE_MT(Grasp);

  // Vector3r
  MAKE_MT(Vector3r);

  // Quaterniond
  MAKE_MT(Quaterniond);

  // LuaLink<T>
  luaL_newmetatable(L, "LuaLinkDouble");
  lua_pushcfunction(L, link_index<double>);
  lua_setfield(L, -2, "__index");
  luaL_newmetatable(L, "LuaLinkDN");
  lua_pushcfunction(L, link_index<addn::DN>);
  lua_setfield(L, -2, "__index");

  // LuaObject<T>
  luaL_newmetatable(L, "LuaObjectDouble");
  lua_pushcfunction(L, object_index<double>);
  lua_setfield(L, -2, "__index");
  luaL_newmetatable(L, "LuaObjectDN");
  lua_pushcfunction(L, object_index<addn::DN>);
  lua_setfield(L, -2, "__index");

  // Cleanup
  lua_pop(L, 10);
}

void load_math_lib(lua_State* L) { luaL_register(L, "dnmath", mathlib); }
void clear_dn_block() { dn_block.clear(); }
void init_dn_block() { dn_block.reserve(100 * sizeof(addn::DN)); }
}  // namespace symbolic::predicate
