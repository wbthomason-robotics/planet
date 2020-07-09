#pragma once
#ifndef SCENEGRAPH_HH
#define SCENEGRAPH_HH

#include <fmt/ostream.h>
#include <tsl/robin_map.h>
#include <urdf_model/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "autodiff.hh"
#include "common.hh"
// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace structures::scenegraph {
namespace {
  template <typename T> struct FKCache {
    T last_joint_val;
    Transform3<T> last_result;
    Transform3r last_coll_result;
    bool first_time = true;
  };
}  // namespace

struct Node {
  // We only support these joint types right now
  enum Type { REVOLUTE, PRISMATIC, FIXED, CONTINUOUS };

  Node(const Str& name,
       const Node::Type type,
       const Transform3r& transform,
       const Transform3r& collision_transform,
       const Vector3r& axis,
       std::shared_ptr<CollisionShape> geom)
  : name(name)
  , transform(transform)
  , geom(std::move(geom))
  , type(type)
  , collision_transform(collision_transform)
  , axis(axis)
  , dn_cache({0.0, Transform3<addn::DN>::Identity(), collision_transform})
  , real_cache({0.0, Transform3r::Identity(), collision_transform}) {}

  Node(const Node& other) = default;

  bool has_child(const Node& child_candidate) const;

  const Str name;

  // Transform from parent's frame to frame at joint of this object, along the axis specified by
  // the URDF joint
  Transform3r transform;

  // Mark the robot base node
  bool is_base = false;

  // Mark obstacle nodes that never move
  bool is_obstacle = false;

  // Mark object nodes
  bool is_object = false;

  // Index into the robot state vector for this joint, if it is a controllable joint
  std::optional<int> idx;

  // Index in the node vector for this joints
  int self_idx;

  const std::shared_ptr<CollisionShape> geom;

  // Describes the joint type of the node for use in FK
  const Node::Type type;

  // Transform from this object's frame to the frame of its collision, as these may be different
  const Transform3r collision_transform;

  // NOTE: If linear search through this list to remove children is slow, use an ordered map on
  // the node names (or an unordered map)
  Vec<int> children;

  // Axis of rotation for the joint associated with this node
  const Vector3r axis;

  // Handle to parent to make removal operations easier
  int parent = -1;

 private:
  void add_child(Node& child);
  void remove_child(const Node& child);
  // Store the last values used in pose_tree, to enable avoiding redundant computation
  FKCache<addn::DN> dn_cache;
  FKCache<double> real_cache;

  template <typename T> FKCache<T>& select_cache();
  template <> FKCache<double>& select_cache<double>() { return real_cache; }

  template <> FKCache<addn::DN>& select_cache<addn::DN>() { return dn_cache; }

  template <typename T>
  std::pair<Transform3<T>, Transform3r> fk(const Transform3<T>& tf, const T& joint_val) {
    Transform3<T> result;
    Transform3r coll_result;
    fk(tf, joint_val, result, coll_result);
    return {result, coll_result};
  }

  template <typename T>
  void fk(const Transform3<T>& tf,
          const T joint_val,
          Transform3<T>& result,
          Transform3r& coll_result) const {
    switch (type) {
      case Type::REVOLUTE:
      case Type::CONTINUOUS:
        // This case is the same as Revolute for us because joint limits are handled in the state
        // space construction
        rev_fk(tf, joint_val, result, coll_result);
        break;

      case Type::FIXED:
        // For a fixed joint, the joint_val will always be 0 anyway, so we ignore it
        fix_fk(tf, result, coll_result);
        break;

      case Type::PRISMATIC:
        pris_fk(tf, joint_val, result, coll_result);
        break;

      default:
        throw std::runtime_error(
        "Invalid joint type in FK! How did this get through construction??");
    }
  }

  template <typename T>
  void rev_fk(const Transform3<T>& tf,
              const T joint_val,
              Transform3<T>& result,
              Transform3r& coll_result) const {
    // axis is guaranteed to be meaningful here
    Eigen::Quaternion<T> pose(Eigen::AngleAxis<T>(joint_val, axis.template cast<T>()));
    result      = tf * transform.template cast<T>() * pose;
    coll_result = result.template cast<double>() * collision_transform;
  }

  template <typename T>
  void pris_fk(const Transform3<T>& tf,
               const T joint_val,
               Transform3<T>& result,
               Transform3r& coll_result) const {
    // axis is guaranteed to be meaningful here
    auto pose   = Eigen::Translation<T, 3>(joint_val * axis.template cast<T>());
    result      = tf * transform.template cast<T>() * pose;
    coll_result = result.template cast<double>() * collision_transform;
  }

  template <typename T>
  void fix_fk(const Transform3<T>& tf, Transform3<T>& result, Transform3r& coll_result) const {
    result      = tf * transform.template cast<T>();
    coll_result = result.template cast<double>() * collision_transform;
  }

  // F = void( const Node* const, const bool, const Transform3<T>&, const Transform3r&), but
  // many arguments to this are highly-capturing lambdas, so we do this rather than a function
  // pointer. We don't use std::function here to avoid the dynamic allocation and virtual call
  // overhead
  template <typename T, typename F>
  void pose_tree(const T* const cont_vals,
                 const T* const joint_vals,
                 const Transform3<T>& parent_tf,
                 const bool new_parent,
                 const bool robot_ancestor,
                 const F& updater,
                 Vec<Node>& nodes,
                 const bool update_cache = true) {
    T joint_val = 0.0;
    if (idx) {
      if (type == CONTINUOUS) {
        joint_val = cont_vals[*idx];
      } else {
        joint_val = joint_vals[*idx];
      }
    }

    FKCache<T>& cache       = select_cache<T>();
    const auto new_pose     = cache.last_joint_val != joint_val;
    const auto needs_update = cache.first_time || new_parent || new_pose;
    auto& last_result       = cache.last_result;
    auto& last_coll_result  = cache.last_coll_result;
    if (needs_update) {
      if (update_cache) {
        cache.last_joint_val = joint_val;
        cache.first_time     = false;
        fk(parent_tf, joint_val, cache.last_result, cache.last_coll_result);
      } else {
        std::tie(last_result, last_coll_result) = fk(parent_tf, joint_val);
      }
    }

    updater(this, robot_ancestor || is_base, last_result, last_coll_result);
    for (const auto child_idx : children) {
      auto& child = nodes[child_idx];
      child.pose_tree(cont_vals,
                      joint_vals,
                      last_result,
                      needs_update,
                      robot_ancestor || is_base,
                      updater,
                      nodes,
                      update_cache);
    }
  }

  friend struct Graph;
};

struct Graph {
  Node& extract(const Str& name);
  Node& add_node(int parent_idx, Node node);
  void add_tree(int idx) { trees.push_back(idx); }
  void reparent_child(Node& parent, Node& child) { parent.add_child(child); }
  Map<Str, Node*> make_robot_nodes_map(const Map<Str, Str>& name_puns);

  Node& find(const Str& name);
  Node& get_node_at_idx(int idx);
  template <typename T, typename F>
  void update_transforms(const T* const cont_vals,
                         const T* const joint_vals,
                         const Transform3<T>& base_tf,
                         const F& updater,
                         const bool update_cache = true) {
    auto& old_base_tf = get_last_base_tf<T>();
    // NOTE: For many floating point values, this will be wrong! But that's ok, because it will
    // always be wrong in the direction of assuming identical parents are not identical, and
    // that's a conservative approximation for a cache
    const auto base_tf_is_new =
    old_base_tf.matrix().cwiseEqual(base_tf.matrix()).count() < old_base_tf.matrix().size();
    old_base_tf = base_tf;
    for (const auto tree_idx : trees) {
      auto& tree = nodes[tree_idx];
      if (tree.is_base) {
        tree.pose_tree<T>(
        cont_vals, joint_vals, base_tf, base_tf_is_new, false, updater, nodes, update_cache);
      } else {
        // NOTE: This is a hack and is wrong if we have objects on other objects, e.g. a tray
        updater(&tree, false, tree.transform.template cast<T>(), tree.collision_transform);
      }
    }
  }

  void pose_objects(const Map<Str, Transform3r>& poses);
  size_t get_num_nodes() { return nodes.size(); }

 private:
  Vec<int> trees;
  Vec<Node> nodes;
  tsl::robin_map<Str, int> idx_index;
  template <typename T> Transform3<T>& get_last_base_tf();
  template <> Transform3r& get_last_base_tf() { return real_last_base_tf; }
  template <> Transform3<addn::DN>& get_last_base_tf() { return dn_last_base_tf; }
  Transform3r real_last_base_tf        = Transform3r::Identity();
  Transform3<addn::DN> dn_last_base_tf = Transform3<addn::DN>::Identity();
};

}  // namespace structures::scenegraph
#endif
