#include "predicate.hh"

#include <stdexcept>

#include "fmt/format.h"

#include <Eigen/Core>

#include "cspace.hh"
#include "world_functions.hh"

constexpr char TRACEBACK_NAME[] = "err_func";
static int traceback(lua_State* L) {
  if (!lua_isstring(L, 1)) /* 'message' not a string? */
    return 1;              /* keep it intact */
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
  lua_pushvalue(L, 1);   /* pass error message */
  lua_pushinteger(L, 2); /* skip this function and traceback */
  lua_call(L, 2, 1);     /* call debug.traceback */
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

namespace symbolic {
namespace predicate {
  namespace cspace = planner::cspace;
  namespace {
    inline void load_c_fns(lua_State* L) {
      /// Add the C functions defined in the world functions module to the Lua environment being
      /// constructed
      lua_pushcfunction(L, worldfns::generate_objects);
      lua_setglobal(L, worldfns::OBJECT_FN_NAME);
      lua_pushcfunction(L, traceback);
      lua_setglobal(L, TRACEBACK_NAME);
      // lua_pushlightuserdata(L, (void*)wrap_exceptions);
      // luaJIT_setmode(L, -1, LUAJIT_MODE_WRAPCFUNC | LUAJIT_MODE_ON);
      // lua_pop(L, 1);
    }
  }  // namespace

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

    for (int i = 0; i < cspace::cont_joint_idxs.size(); ++i) {
      const auto idx     = cspace::cont_joint_idxs[i];
      result[offset + i] = robot_state->as<ob::SO2StateSpace::StateType>(idx)->value;
    }

    offset += cspace::cont_joint_idxs.size();
    for (int i = 0; i < cspace::joint_bounds.size(); ++i) {
      result[offset + i] = joint_state->values[i];
    }

    return result;
  }

  LuaEnvData::LuaEnvData(const Str& name, const Str& prelude_filename) : name(name) {
    log = spdlog::stdout_color_mt(fmt::format("lua-{}", name));
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

  bool LuaEnvData::call_lua(const Str& fn_name,
                            const ob::StateSpace* const space,
                            const ob::State* const state,
                            const bool base_movable) const {
    const auto start_top = lua_gettop(L);

    // Push traceback
    lua_getglobal(L, TRACEBACK_NAME);
    int err_func_idx = lua_gettop(L);

    // Get the function ref
    lua_getglobal(L, fn_name.c_str());

    // Load state vector
    const auto state_vec = generate_state_vector(space, state, base_movable);
    lua_createtable(L, state_vec.size(), 0);
    for (int i = 0; i < state_vec.size(); ++i) {
      lua_pushnumber(L, state_vec[i]);
      lua_rawseti(L, -2, i + 1);
    }

    auto call_result = lua_pcall(L, 1, 1, err_func_idx);
    if (call_result != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Error calling {}: {}", fn_name, err_msg);
      lua_remove(L, err_func_idx + 1);
      throw std::runtime_error(fmt::format("Failed to run {}", fn_name, err_msg));
    }

    lua_remove(L, err_func_idx);
    const auto end_top = lua_gettop(L);
    // We have the +1 here because this method is expected to leave its result on the stack by its
    // callers
    if (end_top != start_top + 1) {
      log->error("Top grew from {} to {}", start_top, end_top);
    }

    return true;
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
      throw std::runtime_error(
      fmt::format("Failed to load formula {}: {}", formula->name, err_msg));
    }

    lua_getglobal(L, formula->normal_fn_name.c_str());
    formula->normal_fn[name] = luaL_ref(L, LUA_REGISTRYINDEX);

    return true;
  }

  bool LuaEnvData::load_gradient(spec::Formula* formula, const int num_dims) const {
    // NOTE: This is only designed to work inside the automatic differentiation context; it will
    // fail elsewhere

    if (fplus::map_contains(formula->gradient_fn, name)) {
      return true;
    }

    log->debug("Creating gradient for: {}", formula->name);
    auto grad_str =
    fmt::format(spec::Formula::GRADIENT_FN_FMT, formula->name, formula->name, num_dims);
    if (luaL_dostring(L, grad_str.c_str()) != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Loading gradient for {} failed: {}", formula->name, err_msg);
      throw std::runtime_error(
      fmt::format("Failed to load gradient for {}: {}", formula->name, err_msg));
    }

    lua_getglobal(L, formula->grad_fn_name.c_str());
    formula->gradient_fn[name] = luaL_ref(L, LUA_REGISTRYINDEX);
    log->debug("Succeeded creating gradient for {}", formula->name);
    return true;
  }

  std::optional<std::pair<arma::vec, double>>
  LuaEnvData::call_gradient(const spec::Formula* formula, const arma::vec& point) const {
    if (!fplus::map_contains(formula->gradient_fn, name)) {
      log->error("Called gradient for {} before it was initialized!", formula->name);
      return std::nullopt;
    }

    const auto start_top = lua_gettop(L);

    // Push traceback
    lua_getglobal(L, TRACEBACK_NAME);
    int err_func_idx = lua_gettop(L);

    // Push alg
    lua_getglobal(L, "alg");

    // Retrieve the gradient ref - use at() to maintain constness
    lua_rawgeti(L, LUA_REGISTRYINDEX, formula->gradient_fn.at(name));

    // Push tovec
    lua_getfield(L, -2, "tovec");

    // Make an array
    lua_createtable(L, point.n_elem, 0);

    // Load the array with point's data
    for (int i = 0; i < point.n_elem; ++i) {
      lua_pushnumber(L, point[i]);
      lua_rawseti(L, -2, i + 1);
    }

    // Use alg.tovec
    if (lua_pcall(L, 1, 1, err_func_idx) != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Error calling alg.tovec: {}", err_msg);
      int top_idx = lua_gettop(L);
      lua_pop(L, top_idx - err_func_idx + 1);
      throw std::runtime_error(fmt::format("Failed to call alg.tovec: {}", err_msg));
    }

    // Make a vec for the gradient result
    lua_getfield(L, -3, "vec");
    lua_pushinteger(L, point.n_elem);

    if (lua_pcall(L, 1, 1, err_func_idx) != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Error calling alg.vec: {}", err_msg);
      int top_idx = lua_gettop(L);
      lua_pop(L, top_idx - err_func_idx + 1);
      throw std::runtime_error(fmt::format("Failed to call alg.vec: {}", err_msg));
    }

    // Store the grad vec somewhere we can get to it later
    auto grad_vec_ref = luaL_ref(L, LUA_REGISTRYINDEX);

    // Put the grad vec back on the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, grad_vec_ref);

    // Call the gradient ref with the point vec and the grad vec
    if (lua_pcall(L, 2, 1, err_func_idx) != 0) {
      auto err_msg = lua_tostring(L, -1);
      log->error("Error calling {} gradient: {}", formula->name, err_msg);
      int top_idx = lua_gettop(L);
      lua_pop(L, top_idx - err_func_idx + 1);
      throw std::runtime_error(
      fmt::format("Failed to call gradient for {}: {}", formula->name, err_msg));
    }

    // Get the value out
    const double value = lua_tonumber(L, -1);
    lua_pop(L, 1);

    // Copy the values out of the grad vec into the result
    arma::vec grad(point.n_elem);
    lua_rawgeti(L, LUA_REGISTRYINDEX, grad_vec_ref);
    const auto grad_vec_idx = lua_gettop(L);
    for (arma::uword i = 0; i < point.n_elem; ++i) {
      // We have to use metamethods to access the elements, else we get a segfault
      lua_pushinteger(L, i + 1);
      lua_gettable(L, grad_vec_idx);
      grad[i] = lua_tonumber(L, -1);
      // TODO(Wil): If this is slow, we could batch pops better
      lua_pop(L, 1);
    }

    // Pop cruft from the stack: alg, value, and the grad vec
    int top_idx = lua_gettop(L);
    lua_pop(L, top_idx - err_func_idx + 1);
    const auto end_top = lua_gettop(L);
    if (end_top != start_top) {
      log->error("Top grew from {} to {}", start_top, end_top);
    }

    return std::optional{std::pair{std::move(grad), value}};
  }

  void LuaEnvData::set_bindings(const Map<Str, Str>& bindings) const {
    for (const auto& [bind_name, real_name] : bindings) {
      lua_pushstring(L, real_name.c_str());
      lua_setglobal(L, (Str("binding") + bind_name).c_str());
    }
  }

  void LuaEnvData::set_universe(void* universe) const {
    /// Set a light userdata with the universe for world_functions
    lua_pushstring(L, "universe");
    lua_pushlightuserdata(L, universe);
    lua_settable(L, LUA_REGISTRYINDEX);
  }

  void LuaEnvData::cleanup() const { lua_gc(L, LUA_GCCOLLECT, 0); }
}  // namespace predicate
}  // namespace symbolic
