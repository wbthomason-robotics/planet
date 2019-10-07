#pragma once

#include "common.hh"

#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "scenegraph.hh"

namespace structures {
namespace robot {
  struct JointData {
    explicit JointData(const Str& name,
                       const double lower_bound,
                       const double upper_bound,
                       const double init_value)
    : name(std::move(name))
    , lower_bound(lower_bound)
    , upper_bound(upper_bound)
    , init_value(init_value) {}

    const Str name;
    const double lower_bound;
    const double upper_bound;
    const double init_value;
  };

  struct Robot {
    explicit Robot(const Map<Str, scenegraph::Node*>& tree_nodes,
                   const std::map<Str, JointData>& controllable_joints,
                   const bool base_movable,
                   std::unique_ptr<Transform3r> base_pose,
                   const std::optional<double>& torso_cfg)
    : tree_nodes(std::move(tree_nodes))
    , controllable_joints(std::move(controllable_joints))
    , base_movable(base_movable)
    , base_pose(std::move(base_pose))
    , torso_cfg(torso_cfg) {}

    // This is used in some initialization functions but really should not be. Pointers into a Vec
    // == icky
    const Map<Str, scenegraph::Node*> tree_nodes;
    const std::map<Str, JointData> controllable_joints;
    const bool base_movable;
    std::unique_ptr<Transform3r> base_pose;
    std::optional<double> torso_cfg;
  };
}  // namespace robot
}  // namespace structures
