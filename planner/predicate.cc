#include <stdexcept>

#include "cspace.hh"
#include "fmt/format.h"
#include "predicate.hh"
#include "predicate-impl.hh"

constexpr char TRACEBACK_NAME[] = "err_func";
static int traceback(lua_State* L) {
  // 'message' not a string?
  if (!lua_isstring(L, 1)) {
    return 1;                 // Keep it intact
  }

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
  void load_c_fns(lua_State* L) {
    lua_pushcfunction(L, traceback);
    lua_setglobal(L, TRACEBACK_NAME);
#ifdef DEBUG_LUA
    lua_pushlightuserdata(L, reinterpret_cast<void*>(wrap_exceptions));
    luaJIT_setmode(L, -1, LUAJIT_MODE_WRAPCFUNC | LUAJIT_MODE_ON);
    lua_pop(L, 1);
#endif
    load_metatables(L);
    load_math_lib(L);
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

void LuaEnvData::cleanup() const { lua_gc(L, LUA_GCCOLLECT, 0); }
int LuaEnvData::call_helper(const spec::Formula& formula, const int start_top) const {
  init_dn_block();
  clear_dn_block();

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
