#pragma once
#ifndef WORLD_FN_UTILS_HH
#define WORLD_FN_UTILS_HH
#include "world_functions.hh"

#include "common.hh"

#include <memory>

#include <bullet/btBulletCollisionCommon.h>

namespace symbolic::worldfns {
void push_pos(lua_State* L, const Eigen::Vector3d& pos, const char* name);
void push_quat(lua_State* L, const Eigen::Quaterniond& quat, const char* name);
void setup_metadata(lua_State* L, const object::Object* obj_data);

int generate_gradient_objects(lua_State* L,
                              int num_objects,
                              int state_idx,
                              const sampler::Universe* uni_data);

int generate_correctness_objects(lua_State* L,
                                 int num_objects,
                                 int state_idx,
                                 const sampler::Universe* uni_data);

void collect_obj_names(lua_State* L, Vec<Str>* obj_order, int num_objects);
template <typename T> Transform3<T> make_base_pose(const Vec<T>& state) {
  Transform3<T> base_tf(*(robot->base_pose));
  if (robot->base_movable) {
    base_tf.translation().x() = state[0];
    base_tf.translation().y() = state[1];
    base_tf.translation().z() = state[2];

    const auto base_rotation = state[3];
    base_tf.linear() =
    Eigen::Quaternion<T>(Eigen::AngleAxis<T>(base_rotation, Eigen::Matrix<T, 3, 1>::UnitZ()))
    .normalized()
    .toRotationMatrix();
  }

  return base_tf;
}

template <typename T>
void fill_joint_arrays(const Vec<T>& state, T* cont_vals, T* joint_vals, int offset) {
  for (size_t i = 0; i < cspace::cont_joint_idxs.size(); ++i) {
    cont_vals[i] = state[offset + i];
  }

  offset += cspace::cont_joint_idxs.size();

  for (size_t i = 0; i < cspace::joint_bounds.size(); ++i) {
    joint_vals[i] = state[offset + i];
  }
}

template <typename T>
void pose_objects(const T* const cont_vals,
                  const T* const joint_vals,
                  Map<Str, Transform3<T>>& obj_poses,
                  const Transform3<T>& base_tf,
                  const sampler::Universe* const uni_data) {
  const auto poser =
  [&](const auto* const node, const auto robot_ancestor, const auto& tf, const auto& coll_tf) {
    obj_poses.emplace(node->name, tf);
  };

  uni_data->sg->update_transforms<T>(cont_vals, joint_vals, base_tf, poser);
}

template <typename T>
void setup_object_helper(lua_State* L,
                         const T& px,
                         const T& py,
                         const T& pz,
                         const T& rx,
                         const T& ry,
                         const T& rz,
                         const T& rw) {
  lua_pushstring(L, OBJECT_FIELD_NAMES[0]);
  lua_pushnumber(L, px);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[1]);
  lua_pushnumber(L, py);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[2]);
  lua_pushnumber(L, pz);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[3]);
  lua_pushnumber(L, rx);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[4]);
  lua_pushnumber(L, ry);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[5]);
  lua_pushnumber(L, rz);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[6]);
  lua_pushnumber(L, rw);
  lua_rawset(L, -3);
}

template <>
void setup_object_helper<addn::DN>(lua_State* L,
                                   const addn::DN& px,
                                   const addn::DN& py,
                                   const addn::DN& pz,
                                   const addn::DN& rx,
                                   const addn::DN& ry,
                                   const addn::DN& rz,
                                   const addn::DN& rw) {
  lua_pushstring(L, OBJECT_FIELD_NAMES[0]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, px.v);
  lua_pushnumber(L, px.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[1]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, py.v);
  lua_pushnumber(L, py.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[2]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, pz.v);
  lua_pushnumber(L, pz.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[3]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, rx.v);
  lua_pushnumber(L, rx.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[4]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, ry.v);
  lua_pushnumber(L, ry.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[5]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, rz.v);
  lua_pushnumber(L, rz.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);

  lua_pushstring(L, OBJECT_FIELD_NAMES[6]);
  lua_getglobal(L, "dn");
  lua_pushnumber(L, rw.v);
  lua_pushnumber(L, rw.a);
  lua_pcall(L, 2, 1, 0);
  lua_rawset(L, -3);
}

template <typename T> void setup_object(lua_State* L, const Transform3<T>& tf) {
  Eigen::Matrix<T, 3, 1> translation(tf.translation());
  Eigen::Quaternion<T> rotation(tf.linear());
  setup_object_helper(L,
                      translation.x(),
                      translation.y(),
                      translation.z(),
                      rotation.x(),
                      rotation.y(),
                      rotation.z(),
                      rotation.w());
}

void setup_object(lua_State* L, const btTransform& tf) {
  const auto& translation = tf.getOrigin();
  const auto& rotation    = tf.getRotation();
  setup_object_helper(L,
                      translation.x(),
                      translation.y(),
                      translation.z(),
                      rotation.x(),
                      rotation.y(),
                      rotation.z(),
                      rotation.w());
}

template <typename T>
void make_objects(lua_State* L,
                  const Vec<Str>& obj_order,
                  Map<Str, Transform3<T>>& obj_poses,
                  const std::shared_ptr<structures::scenegraph::Graph>& sg) {
  // Create object and obstacle tables
  for (const auto& obj_name : obj_order) {
    const auto& node = sg->find(obj_name);
    if (!node.is_object && !node.is_obstacle) {
      lua_createtable(L, 0, OBJECT_DATA_SIZE);
      setup_object(L, obj_poses[obj_name]);
      continue;
    }

    object::Object* obj_data = nullptr;
    const auto& obj_it       = objects->find(obj_name);
    if (obj_it != objects->end()) {
      obj_data = obj_it->second.get();
      lua_createtable(L, 0, OBJ_STATE_SIZE);
      setup_metadata(L, obj_data);
      setup_object(L, obj_poses[obj_name]);
      continue;
    }


    obj_data = obstacles->at(obj_name).get();
    lua_createtable(L, 0, OBJ_STATE_SIZE);
    setup_metadata(L, obj_data);
    setup_object(L, obj_poses[obj_name]);
    //  else {
    //   log->critical("Weird thing: {} isn't an object, obstacle, or robot link...", obj_name);
    //   throw std::runtime_error("Object of unknown type!");
    // }
  }
}
}  // namespace symbolic::worldfns
#endif
