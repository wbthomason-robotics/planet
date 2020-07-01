#include <Eigen/Core>
#include <mpark/patterns.hpp>
#include <stdexcept>

#include "cspace.hh"
#include "fmt/format.h"
#include "predicate.hh"

constexpr char TRACEBACK_NAME[] = "err_func";
static int traceback(lua_State* L) {
  if (!lua_isstring(L, 1))  // 'message' not a string?
    return 1;               // Keep it intact
  lua_getfield(L, LUA_GLOBALSINDEX, "debug");
  if (!lua_istable(L, -1)) {
    lua_pop(L, 1);
    return 1;
  }
  lua_getfield(L, -1, "traceback");
  if (!lua_isfunction(L, -1)) {
    lua_pop(L, 2);
    return 1;
  }
  lua_pushvalue(L, 1);    // Pass error message
  lua_pushinteger(L, 2);  // Skip this function and traceback
  lua_call(L, 2, 1);      // Call debug.traceback
  return 1;
}

static int wrap_exceptions(lua_State* L, lua_CFunction f) {
  try {
    return f(L);             // Call wrapped function and return result.
  } catch (const char* s) {  // Catch and convert exceptions.
    lua_pushstring(L, s);
  } catch (std::exception& e) {
    lua_pushstring(L, e.what());
  } catch (...) {
    lua_pushliteral(L, "caught (...)");
  }

  return lua_error(L);  // Rethrow as a Lua error.
}

namespace symbolic::predicate {
const structures::object::ObjectSet* objects   = nullptr;
const structures::object::ObjectSet* obstacles = nullptr;
namespace cspace                               = planner::cspace;
namespace {
  constexpr char const* dim_names[]{"x", "y", "z"};
  void load_c_fns(lua_State* L) {
    lua_pushcfunction(L, traceback);
    lua_setglobal(L, TRACEBACK_NAME);
#ifdef DEBUG_LUA
    lua_pushlightuserdata(L, reinterpret_cast<void*>(wrap_exceptions));
    luaJIT_setmode(L, -1, LUAJIT_MODE_WRAPCFUNC | LUAJIT_MODE_ON);
    lua_pop(L, 1);
#endif
  }

  void push_pos(lua_State* L, const Vector3r& pos, const char* const name) {
    if (name != nullptr) {
      lua_pushstring(L, name);
    }

    lua_createtable(L, 0, 3);
    for (int i = 0; i < 3; ++i) {
      lua_pushstring(L, dim_names[i]);
      lua_pushnumber(L, pos(i));
      lua_rawset(L, -3);
    }

    if (name != nullptr) {
      // NOTE: This assumes there's a table on the stack ready for the position field just behind
      // the start position
      lua_rawset(L, -3);
    }
  }

  void push_quat(lua_State* L, const Eigen::Quaterniond& quat, const char* const name) {
    if (name != nullptr) {
      lua_pushstring(L, name);
    }

    lua_createtable(L, 0, 4);
    lua_pushstring(L, "x");
    lua_pushnumber(L, quat.x());
    lua_rawset(L, -3);

    lua_pushstring(L, "y");
    lua_pushnumber(L, quat.y());
    lua_rawset(L, -3);

    lua_pushstring(L, "z");
    lua_pushnumber(L, quat.z());
    lua_rawset(L, -3);

    lua_pushstring(L, "w");
    lua_pushnumber(L, quat.w());
    lua_rawset(L, -3);

    if (name != nullptr) {
      // NOTE: This assumes there's a table on the stack ready for the position field just behind
      // the start position
      lua_rawset(L, -3);
    }
  }
}  // namespace

bool LuaEnv<bool>::call(const spec::Formula& formula,
                        const Map<Str, Str>& bindings,
                        structures::scenegraph::Graph* const sg,
                        const ob::State* const state,
                        const ob::StateSpace* const space,
                        const bool base_movable) {
  setup_world<double>(formula, bindings, sg);
  Transform3r base_tf;
  const auto& state_vec          = generate_state_vector(space, state, base_movable);
  const double* const cont_vals  = &(*state_vec.begin()) + (base_movable ? 4 : 0);
  const double* const joint_vals = cont_vals + planner::cspace::cont_joint_idxs.size();
  if (base_movable) {
    make_base_tf(state_vec, base_tf);
  }

  sg->update_transforms(cont_vals,
                        joint_vals,
                        base_tf,
                        [&](const structures::scenegraph::Node* const node,
                            const bool new_value,
                            const Transform3r& pose,
                            const Transform3r& coll_pose) { update_object(node->name, pose); });
  const bool result = call_formula<bool>(formula);
  teardown_world(formula);

  return result;
}

Vec<double> generate_state_vector(const ob::StateSpace* const space,
                                  const ob::State* const state,
                                  const bool base_movable) {
  // The state vector has the form [robot base pose (if it exists), continuous joints, other
  // joints]
  Vec<double> result(cspace::num_dims);

  // Separate out the state spaces for dimension and index information
  auto full_space  = space->as<ob::CompoundStateSpace>();
  auto robot_space = full_space->getSubspace(cspace::ROBOT_SPACE)->as<ob::CompoundStateSpace>();

  const auto cstate = state->as<ob::CompoundState>();

  const auto& robot_state =
  cstate->as<ob::CompoundState>(full_space->getSubspaceIndex(cspace::ROBOT_SPACE));

  int offset = 0;
  if (base_movable) {
    auto robot_base_state = robot_state->as<cspace::RobotBaseSpace::StateType>(
    robot_space->getSubspaceIndex(cspace::BASE_SPACE));

    // NOTE: This is only valid because we're only getting pose. Could be cleaner, too
    result[0] = robot_base_state->getX();
    result[1] = robot_base_state->getY();
    result[2] = robot_base_state->getZ();

    const ob::SO2StateSpace::StateType& base_rotation = robot_base_state->rotation();
    result[3]                                         = base_rotation.value;

    offset += 4;
  }

  const auto& joint_state = robot_state->as<cspace::RobotJointSpace::StateType>(
  robot_space->getSubspaceIndex(cspace::JOINT_SPACE));

  for (size_t i = 0; i < cspace::cont_joint_idxs.size(); ++i) {
    const auto idx     = cspace::cont_joint_idxs[i];
    result[offset + i] = robot_state->as<ob::SO2StateSpace::StateType>(idx)->value;
  }

  offset += cspace::cont_joint_idxs.size();
  for (size_t i = 0; i < cspace::joint_bounds.size(); ++i) {
    result[offset + i] = joint_state->values[i];
  }

  return result;
}

LuaEnvData::LuaEnvData(const Str& name, const Str& prelude_filename) : name(name) {
  log = spdlog::stdout_color_st(fmt::format("lua-{}", name));
  L   = luaL_newstate();
  luaL_openlibs(L);
  load_c_fns(L);
  if (!prelude_filename.empty()) {
    log->debug("Loading prelude from {}", prelude_filename);
    if (luaL_dofile(L, prelude_filename.c_str()) != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Loading prelude from {} failed: {}", prelude_filename, err_msg);
      throw std::runtime_error("Failed to load Lua prelude!");
    }
  }
}

bool LuaEnvData::load_predicates(const Str& predicates_filename) const {
  log->debug("Loading predicates from {}", predicates_filename);
  if (luaL_dofile(L, predicates_filename.c_str()) != 0) {
    auto err_msg = lua_tostring(L, -1);
    log->error("Loading predicates from {} failed: {}", predicates_filename, err_msg);
    throw std::runtime_error(
    fmt::format("Failed to load predicates from {}: {}", predicates_filename, err_msg));
  }

  return true;
}

bool LuaEnvData::load_formula(spec::Formula* formula) const {
  log->debug("Loading formula: {}", formula->name);
  if (luaL_dostring(L, formula->normal_def.c_str()) != 0) {
    auto err_msg = lua_tostring(L, -1);
    log->error("Loading formula {} failed: {}", formula->name, err_msg);
    throw std::runtime_error(fmt::format("Failed to load formula {}: {}", formula->name, err_msg));
  }

  lua_getglobal(L, formula->normal_fn_name.c_str());
  formula->normal_fn[name] = luaL_ref(L, LUA_REGISTRYINDEX);

  return true;
}

void LuaEnvData::teardown_world(const spec::Formula& formula) const {
  // NOTE: If we for some reason stop making one object/binding (e.g. if there's something weird
  // with globals), then this is wrong and will cause segfaults
  lua_pop(L, formula.bindings.size());
}

void LuaEnvData::setup_metadata(const structures::object::Object* const obj_data) const {
  using namespace mpark::patterns;
  namespace obj = structures::object;
  // SSSP
  lua_pushstring(L, "sssp");
  if (obj_data->stable_face) {
    lua_createtable(L, 0, 2);
    match (*obj_data->stable_face)(
    pattern(as<obj::StableRegion>(arg)) =
    [&](const auto& region) {
      // Set plane to nil
      lua_pushstring(L, "plane");
      lua_pushnil(L);
      lua_rawset(L, -3);

      lua_pushstring(L, "region");
      lua_createtable(L, 0, 3);
      for (const auto& name : dim_names) {
        lua_pushstring(L, name);
        lua_createtable(L, 0, 2);

        lua_pushstring(L, "low");
        lua_pushnumber(L, region.bounds.at(name).low);
        lua_rawset(L, -3);

        lua_pushstring(L, "high");
        lua_pushnumber(L, region.bounds.at(name).high);
        lua_rawset(L, -3);

        // Sets the bounds table as the item for the key $name in the region table
        lua_rawset(L, -3);
      }

      // Sets the table of bounds as the item for the key "region" in the object data table
      lua_rawset(L, -3);
    },
    pattern(as<obj::StablePlane>(arg)) =
    [&](const auto& plane_vecs) {
      // Set region to nil
      lua_pushstring(L, "region");
      lua_pushnil(L);
      lua_rawset(L, -3);

      // Setup the plane representation
      lua_pushstring(L, "plane");
      lua_createtable(L, plane_vecs.size(), 0);
      for (size_t i = 0; i < plane_vecs.size(); ++i) {
        const auto& vec = plane_vecs[i];
        push_pos(L, vec, nullptr);
        lua_rawseti(L, -2, i + 1);
      }

      lua_rawset(L, -3);
    });
  } else {
    lua_pushnil(L);
  }

  lua_rawset(L, -3);

  // SOP
  lua_pushstring(L, "sop");
  const auto& sops = obj_data->stable_poses;
  lua_createtable(L, sops.size(), 0);
  for (size_t i = 0; i < sops.size(); ++i) {
    lua_createtable(L, 0, 3);
    const Vector3r& axis          = sops[i].axis;
    const Eigen::Quaterniond& rot = sops[i].template_rotation;
    const double distance         = sops[i].distance;
    push_pos(L, axis, "axis");
    push_quat(L, rot, "rot");

    lua_pushstring(L, "d");
    lua_pushnumber(L, distance);
    lua_rawset(L, -3);

    lua_rawseti(L, -2, i + 1);
  }

  lua_rawset(L, -3);

  // Grasps
  lua_pushstring(L, "grasps");
  const auto& grasps = obj_data->grasps;
  lua_createtable(L, grasps.size(), 0);
  for (size_t i = 0; i < grasps.size(); ++i) {
    lua_createtable(L, 0, 2);
    const auto& grasp = grasps[i];
    lua_pushstring(L, "discrete");
    match(grasp)(
    pattern(as<obj::DiscreteGrasp>(arg)) =
    [&](const auto& grasp_data) {
      lua_createtable(L, 0, 2);
      const auto& grasp_frame = grasp_data.frame;
      const Vector3r& pos     = grasp_frame.translation();
      const Eigen::Quaterniond rot(grasp_frame.linear());
      push_pos(L, pos, "pos");
      push_quat(L, rot, "rot");
      lua_rawset(L, -3);

      lua_pushstring(L, "continuous");
      lua_pushnil(L);
      lua_rawset(L, -3);
    },
    pattern(as<obj::ContinuousGrasp>(arg)) =
    [&](const auto& grasp_data) {
      lua_pushnil(L);
      lua_rawset(L, -3);
      lua_pushstring(L, "continuous");
      lua_createtable(L, 0, 3);
      const auto& [grasp_frame, grasp_axis] = grasp_data;
      const Vector3r& pos                   = grasp_frame.translation();
      const Eigen::Quaterniond rot(grasp_frame.linear());

      push_pos(L, pos, "pos");
      push_quat(L, rot, "rot");
      push_pos(L, grasp_axis, "axis");

      lua_rawset(L, -3);
    });

    lua_rawseti(L, -2, i + 1);
  }

  lua_rawset(L, -3);
}

void LuaEnvData::cleanup() const { lua_gc(L, LUA_GCCOLLECT, 0); }
int LuaEnvData::call_helper(const spec::Formula& formula, const int start_top) const {
  // Push traceback
  lua_getglobal(L, TRACEBACK_NAME);
  const int err_func_idx = lua_gettop(L);

  // Retrieve the formula function ref - use at() to maintain constness
  lua_rawgeti(L, LUA_REGISTRYINDEX, formula.normal_fn.at(name));

  // Copy the object tables as arguments
  for (size_t i = 0; i < binding_idx.size(); ++i) {
    lua_pushvalue(L, start_top - i);
  }

  // Call the function ref
  if (lua_pcall(L, binding_idx.size(), 1, err_func_idx) != 0) {
    auto err_msg = lua_tostring(L, -1);
    log->error("Error calling {} for gradient: {}", formula.name, err_msg);
    int top_idx = lua_gettop(L);
    lua_pop(L, top_idx - err_func_idx + 1);
    throw std::runtime_error(
    fmt::format("Failed to call {} for gradient: {}", formula.name, err_msg));
  }
  return err_func_idx;
}

void LuaEnvData::cleanup_helper(const int start_top, const int err_func_idx) const {
  // Pop cruft from the stack: the result table, a, v, err_func
  int top_idx = lua_gettop(L);
  lua_pop(L, top_idx - err_func_idx + 1);
  const auto end_top = lua_gettop(L);
  if (end_top != start_top) {
    log->error("Top grew from {} to {}", start_top, end_top);
  }
}
}  // namespace symbolic::predicate
