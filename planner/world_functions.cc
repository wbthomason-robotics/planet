#include "world_functions.hh"
#include "common.hh"

#include <stdexcept>
#include <variant>
#include <vector>

#include "world_fn_utils.hh"

namespace symbolic {
namespace worldfns {
  namespace {
    auto log = spdlog::stdout_color_mt("world_fns");
    constexpr char const* dim_names[]{"x", "y", "z"};
  }  // namespace

  ompl::base::CompoundStateSpace* robot_space = nullptr;
  ObjectSet* objects                          = nullptr;
  ObjectSet* obstacles                        = nullptr;
  Robot* robot                                = nullptr;
  int state_size                              = 0;

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

  void setup_metadata(lua_State* L, const object::Object* const obj_data) {
    // SSSP
    lua_pushstring(L, "sssp");
    if (obj_data->stable_face) {
      lua_createtable(L, 0, 2);
      if (std::holds_alternative<object::StableRegion>(*(obj_data->stable_face))) {
        // Set plane to nil
        lua_pushstring(L, "plane");
        lua_pushnil(L);
        lua_rawset(L, -3);

        // Setup the region bounds
        const auto& region = std::get<object::StableRegion>(*(obj_data->stable_face));
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

      } else {
        // Must be a StablePlane
        // Set region to nil
        lua_pushstring(L, "region");
        lua_pushnil(L);
        lua_rawset(L, -3);
        // Setup the plane representation
        const auto& plane_vecs = std::get<object::StablePlane>(*(obj_data->stable_face));
        lua_pushstring(L, "plane");
        lua_createtable(L, plane_vecs.size(), 0);
        for (int i = 0; i < plane_vecs.size(); ++i) {
          const auto& vec = plane_vecs[i];
          push_pos(L, vec, nullptr);
          lua_rawseti(L, -2, i + 1);
        }

        lua_rawset(L, -3);
      }
    } else {
      lua_pushnil(L);
    }

    lua_rawset(L, -3);

    // SOP
    lua_pushstring(L, "sop");
    const auto& sops = obj_data->stable_poses;
    lua_createtable(L, sops.size(), 0);
    for (int i = 0; i < sops.size(); ++i) {
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
    for (int i = 0; i < grasps.size(); ++i) {
      lua_createtable(L, 0, 2);
      const auto& grasp = grasps[i];
      lua_pushstring(L, "discrete");
      if (std::holds_alternative<object::DiscreteGrasp>(grasp)) {
        lua_createtable(L, 0, 2);
        const auto& grasp_frame = std::get<object::DiscreteGrasp>(grasp).frame;
        const Vector3r& pos     = grasp_frame.translation();
        const Eigen::Quaterniond rot(grasp_frame.linear());
        push_pos(L, pos, "pos");
        push_quat(L, rot, "rot");
        lua_rawset(L, -3);

        lua_pushstring(L, "continuous");
        lua_pushnil(L);
        lua_rawset(L, -3);
      } else {
        // Must be a ContinuousGrasp
        lua_pushnil(L);
        lua_rawset(L, -3);
        lua_pushstring(L, "continuous");
        lua_createtable(L, 0, 3);
        const auto& [grasp_frame, grasp_axis] = std::get<object::ContinuousGrasp>(grasp);
        const Vector3r& pos                   = grasp_frame.translation();
        const Eigen::Quaterniond rot(grasp_frame.linear());

        push_pos(L, pos, "pos");
        push_quat(L, rot, "rot");
        push_pos(L, grasp_axis, "axis");

        lua_rawset(L, -3);
      }

      lua_rawseti(L, -2, i + 1);
    }

    lua_rawset(L, -3);
  }

  /// NOTE: This is only used from Lua
  int generate_objects(lua_State* L) {
    // Check for initialization of globals
    if (robot_space == nullptr) {
      log->error("Somehow generate_objects was called before state space was initialized!");
      lua_pushnil(L);
      lua_pushstring(L, "State space not initialized!");
      return 2;
    }

    if (robot == nullptr) {
      log->error("Somehow generate_objects was called before robot info was initialized!");
      lua_pushnil(L);
      lua_pushstring(L, "Robot not initialized!");
      return 2;
    }

    if (obstacles == nullptr) {
      log->error("Somehow generate_objects was called before obstacle info was initialized!");
      lua_pushnil(L);
      lua_pushstring(L, "Obstacles not initialized!");
      return 2;
    }

    if (objects == nullptr) {
      log->error("Somehow generate_objects was called before object info was initialized!");
      lua_pushnil(L);
      lua_pushstring(L, "Object info not initialized!");
      return 2;
    }

    if (lua_gettop(L) != 2) {
      log->critical("Stack top is {} instead of expected 2 in generate_objects!", lua_gettop(L));
      lua_pushnil(L);
      lua_pushstring(L, "Dirty stack; unexpected objects!");
      return 2;
    }

    // Get the universe
    lua_pushstring(L, "universe");
    lua_gettable(L, LUA_REGISTRYINDEX);
    auto uni_data = static_cast<const sampler::Universe* const>(lua_topointer(L, -1));
    lua_pop(L, 1);

    // If we get here, we have all the global state we need
    // State vector is: [Robot_parts... Object1... Object2... ... ObjectN...]
    int num_objects         = lua_objlen(L, -1);
    constexpr int state_idx = 1;
    // NOTE: The autograd library uses a custom cdata datatype, so we have these variants
    const auto is_gradient = lua_type(L, state_idx) != LUA_TTABLE;
    if (is_gradient) {
      return generate_gradient_objects(L, num_objects, state_idx, uni_data);
    }

    return generate_correctness_objects(L, num_objects, state_idx, uni_data);
  }

  int generate_correctness_objects(lua_State* L,
                                   const int num_objects,
                                   const int state_idx,
                                   const sampler::Universe* const uni_data) {
    Map<Str, Transform3r> obj_poses;
    Vec<Str> obj_order;
    obj_order.reserve(num_objects);
    obj_poses.reserve(num_objects);
    collect_obj_names(L, &obj_order, num_objects);
    // for (const auto& name : obj_order) {
    //   obj_poses.emplace(name, nullptr);
    // }

    // Read off state vec
    Vec<double> state(state_size);
    for (int i = 1; i <= state_size; ++i) {
      lua_rawgeti(L, state_idx, i);
      state[i - 1] = lua_tonumber(L, -1);
    }

    lua_pop(L, state_size);

    // log->warn("State vec for {} is: {}", is_gradient ? "gradient" : "correctness", state);

    // Pose the objects
    double cont_vals[cspace::cont_joint_idxs.size()];
    double joint_vals[cspace::joint_bounds.size()];
    Transform3r base_tf = make_base_pose(state);
    fill_joint_arrays(state, cont_vals, joint_vals, robot->base_movable ? 4 : 0);
    pose_objects(cont_vals, joint_vals, obj_poses, base_tf, uni_data);
    make_objects(L, obj_order, obj_poses, uni_data->sg);
    return num_objects;
  }

  int generate_gradient_objects(lua_State* L,
                                const int num_objects,
                                const int state_idx,
                                const sampler::Universe* const uni_data) {
    Map<Str, Transform3<addn::DN>> obj_poses;
    Vec<Str> obj_order;
    obj_order.reserve(num_objects);
    obj_poses.reserve(num_objects);
    collect_obj_names(L, &obj_order, num_objects);

    // Read off state vec
    Vec<addn::DN> state(state_size);
    for (int i = 1; i <= state_size; ++i) {
      lua_pushinteger(L, i);
      lua_gettable(L, state_idx);
      const auto table_idx = lua_gettop(L);
      lua_pushstring(L, "_v");
      lua_gettable(L, table_idx);
      const auto v = lua_tonumber(L, -1);
      lua_pushstring(L, "_a");
      lua_gettable(L, table_idx);
      const auto a = lua_tonumber(L, -1);
      state[i - 1] = addn::DN{v, a};
    }

    lua_pop(L, 3 * state_size);

    // Pose the objects
    addn::DN cont_vals[cspace::cont_joint_idxs.size()];
    addn::DN joint_vals[cspace::joint_bounds.size()];
    Transform3<addn::DN> base_tf = make_base_pose(state);
    fill_joint_arrays(state, cont_vals, joint_vals, robot->base_movable ? 4 : 0);
    pose_objects(cont_vals, joint_vals, obj_poses, base_tf, uni_data);
    make_objects(L, obj_order, obj_poses, uni_data->sg);

    return num_objects;
  }

  void collect_obj_names(lua_State* L, Vec<Str>* obj_order, const int num_objects) {
    int bindings = 0;
    // Collect the object names
    for (int i = 1; i <= num_objects; ++i) {
      // Get the name component of the object array
      lua_rawgeti(L, 2, i);
      auto obj_name = luaL_checkstring(L, -1);
      // HACK: This is a way to see if the name is unbound and we need to get the bound name from
      // the env
      if (obj_name[0] == '_') {
        lua_getglobal(L, (Str("binding") + obj_name).c_str());
        obj_name = luaL_checkstring(L, -1);
        ++bindings;
      }

      obj_order->emplace_back(obj_name);
    }

    lua_pop(L, num_objects + bindings);
  }
}  // namespace worldfns
}  // namespace symbolic
