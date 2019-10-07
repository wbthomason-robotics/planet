#include "cspace.hh"

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <bullet/btBulletCollisionCommon.h>

#include <fmt/ostream.h>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include <boost/container_hash/hash.hpp>

#include "fplus/fplus.hpp"

namespace planner::cspace {
namespace {
  auto log                          = spdlog::stdout_color_mt("cspace");
  inline constexpr double neg_infty = -std::numeric_limits<double>::infinity();
  inline constexpr double pos_infty = std::numeric_limits<double>::infinity();
  inline constexpr void update_bounds(std::array<double, 3>& min_bounds,
                                      std::array<double, 3>& max_bounds,
                                      double x,
                                      const double y,
                                      const double z,
                                      const double radius) {
    max_bounds[0] = std::max({x + radius, x - radius, max_bounds[0]});
    min_bounds[0] = std::min({x + radius, x - radius, min_bounds[0]});
    max_bounds[1] = std::max({y + radius, y - radius, max_bounds[1]});
    min_bounds[1] = std::min({y + radius, y - radius, min_bounds[1]});
    max_bounds[2] = std::max({z + radius, z - radius, max_bounds[2]});
    // No need to search for z min as we assume it to be 0
  }
}  // namespace

namespace fwd = fplus::fwd;

std::unique_ptr<ob::RealVectorBounds> workspace_bounds = nullptr;

util::UniverseMap* CompositeSpace::universe_map = nullptr;

// INSANE PLEASE UNDO
void CompositeSpace::sanityChecks(double zero, double eps, unsigned int flags) const {}
void CompositeSpace::sanityChecks() const {}

bool CompositeSpace::isDiscrete() const {
  return std::all_of(components_.begin(), components_.end(), [](const auto& c) {
    return c->isDiscrete();
  });
}

bool CompositeSpace::satisfiesBounds(const ob::State* state) const {
  const auto* cstate = dynamic_cast<const StateType*>(state);
  for (unsigned int i = 0; i < componentCount_; ++i) {
    const auto& component = getSubspace(i);
    if (!component->satisfiesBounds(cstate->components[i])) {
      log->debug("State failed to satisfy bounds for subspace {}.", component->getName());
      return false;
    }
  }

  return true;
}

void CompositeSpace::interpolate(const ob::State* from,
                                 const ob::State* to,
                                 const double t,
                                 ob::State* state) const {
  // We only interpolate in robot state, then update object poses accordingly
  const auto* cfrom = static_cast<const StateType*>(from);
  const auto* cto   = static_cast<const StateType*>(to);
  auto* cstate      = static_cast<StateType*>(state);

  // Set object_poses pointer to be the pose set from the origin
  cstate->object_poses = cfrom->object_poses;
  cstate->sg           = cfrom->sg;

  // Null the action pointer since by interpolating we effectively didn't take an action to get
  // here
  cstate->action = nullptr;

  // Interpolate the robot state
  getSubspace(robot_space_idx)
  ->interpolate(cfrom->components[robot_space_idx],
                cto->components[robot_space_idx],
                t,
                cstate->components[robot_space_idx]);

  // Copy the object state
  getSubspace(objects_space_idx)
  ->copyState(cstate->components[objects_space_idx], cfrom->components[objects_space_idx]);

  // Copy the discrete state
  getSubspace(eqclass_space_idx)
  ->copyState(cstate->components[eqclass_space_idx], cfrom->components[eqclass_space_idx]);

  getSubspace(discrete_space_idx)
  ->copyState(cstate->components[discrete_space_idx], cfrom->components[discrete_space_idx]);

  // Update object poses according to the interpolated robot state
  auto* objects_state       = cstate->as<ob::CompoundState>(objects_space_idx);
  const auto* objects_space = components_[objects_space_idx]->as<ob::CompoundStateSpace>();
  const auto poser          = [&](const structures::scenegraph::Node* const node,
                         const bool robot_ancestor,
                         const Transform3r& tf,
                         const Transform3r& _) {
    if (robot_ancestor && node->is_object) {
      auto object_state =
      objects_state->as<ObjectSpace::StateType>(objects_space->getSubspaceIndex(node->name));
      const auto& translation = tf.translation();
      Eigen::Quaterniond rotation(tf.linear());
      // rotation.normalize();
      object_state->setXYZ(translation.x(), translation.y(), translation.z());
      auto& object_rotation = object_state->rotation();
      object_rotation.x     = rotation.x();
      object_rotation.y     = rotation.y();
      object_rotation.z     = rotation.z();
      object_rotation.w     = rotation.w();
    }
  };

  const auto* robot_state = cstate->as<ob::CompoundState>(robot_space_idx);
  double cont_vals[cont_joint_idxs.size()];
  double* joint_vals      = nullptr;
  const auto& joint_state = robot_state->as<cspace::RobotJointSpace::StateType>(joint_space_idx);
  Transform3r base_tf;
  util::state_to_pose_data(robot_state,
                           joint_state,
                           cspace::cont_joint_idxs,
                           base_space_idx,
                           cont_vals,
                           &joint_vals,
                           &base_tf);

  cstate->sg->update_transforms<double>(cont_vals, joint_vals, base_tf, poser);
}

double CompositeSpace::distance(const ob::State* state1, const ob::State* state2) const {
  const auto* cstate1         = static_cast<const StateType*>(state1);
  const auto* cstate2         = static_cast<const StateType*>(state2);
  const bool same_object_pose = cstate1->object_poses == cstate2->object_poses;
  // const auto* eqclass_space   = components_[eqclass_space_idx].get();
  // const auto* discrete_space  = components_[discrete_space_idx].get();
  // const bool same_universe =
  // eqclass_space->equalStates(cstate1->as<DiscreteSpace::StateType>(eqclass_space_idx),
  //                            cstate2->as<DiscreteSpace::StateType>(eqclass_space_idx));
  // const bool same_config =
  // discrete_space->equalStates(cstate1->as<DiscreteSpace::StateType>(discrete_space_idx),
  //                             cstate2->as<DiscreteSpace::StateType>(discrete_space_idx));
  // const bool same_discrete = same_universe && same_config;
  if (same_object_pose && universe_map->check_valid_transition(cstate1, cstate2)) {
    return contDistance(state1, state2);
  }

  return pos_infty;
}

double CompositeSpace::contDistance(const ob::State* state1, const ob::State* state2) const {
  const auto* cstate1 = static_cast<const StateType*>(state1);
  const auto* cstate2 = static_cast<const StateType*>(state2);
  double dist         = 0.0;
  for (unsigned int i = 0; i < componentCount_; ++i) {
    if (i != eqclass_space_idx && i != discrete_space_idx) {
      dist +=
      weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    }
  }

  return dist;
}

size_t CompositeSpace::computeHash(const StateType& x) const {
  size_t hash_val = 0;
  // We have to do this component-by-component because not all subspaces (i.e. CompoundStateSpace
  // and RealVectorStateSpace) have hash implementations b/c of the stupid OMPL lack of length
  // information

  // Robot subspace
  const auto& robot_state = x.as<ob::CompoundState>(robot_space_idx);
  const auto& robot_space = components_[robot_space_idx]->as<ob::CompoundStateSpace>();
  // Robot: Base subspace (if exists)
  if (robot_space->hasSubspace(BASE_SPACE)) {
    const auto& base_state =
    robot_state->as<RobotBaseSpace::StateType>(robot_space->getSubspaceIndex(BASE_SPACE));
    boost::hash_combine(hash_val, *base_state);
  }

  // Robot: Continuous joints subspaces
  for (const auto idx : cont_joint_idxs) {
    const auto& joint_state = robot_state->as<ob::SO2StateSpace::StateType>(idx);
    boost::hash_combine(hash_val, *joint_state);
  }

  // Robot: Other joints subspaces
  const auto joint_space_idx = robot_space->getSubspaceIndex(JOINT_SPACE);
  const auto& joints_state   = robot_state->as<RobotJointSpace::StateType>(joint_space_idx);
  for (unsigned int i = 0; i < joint_bounds.size(); ++i) {
    boost::hash_combine(hash_val, joints_state->values[i]);
  }

  // Objects subspace
  const auto& objects_state = x.as<ob::CompoundState>(objects_space_idx);
  for (unsigned int i = 0; i < num_objects; ++i) {
    boost::hash_combine(hash_val, *(objects_state->as<ObjectSpace::StateType>(i)));
  }

  // Universe subspace
  const auto& universe_state = x.as<ob::CompoundState>(eqclass_space_idx);
  for (unsigned int i = 0; i < eqclass_subspace_count; ++i) {
    boost::hash_combine(hash_val, *(universe_state->as<DiscreteSpace::StateType>(i)));
  }

  // Config subspace
  const auto& config_state = x.as<ob::CompoundState>(discrete_space_idx);
  for (unsigned int i = 0; i < discrete_subspace_count; ++i) {
    boost::hash_combine(hash_val, *(config_state->as<DiscreteSpace::StateType>(i)));
  }

  return hash_val;
}

// NOTE: Spooky scary global state
int num_dims = 0;
Vec<int> cont_joint_idxs;
Vec<std::pair<double, double>> joint_bounds;

std::tuple<std::array<double, 3>, std::array<double, 3>, double>
make_workspace_bounds(const structures::robot::Robot* const robot,
                      structures::scenegraph::Graph* const sg,
                      const structures::object::ObjectSet* const objects,
                      const structures::object::ObjectSet* const obstacles) {
  // Compute a bounding box for the workspace by (a) finding the min and max (x, y, z) of the
  // robot and all objects (including bounding boxes) and (b) adding 2 * robot diameter to each
  // to allow room to move around the workspace
  std::array<double, 3> max_bounds{neg_infty, neg_infty, neg_infty};
  std::array<double, 3> min_bounds{pos_infty, pos_infty, 0.0};

  // Handle the robot
  double robot_radius              = 0.0;
  const Vector3r& base_translation = robot->base_pose->translation();
  double radius, x, y, z;
  auto robot_walker =
  [&](const auto* const node, const bool robot_ancestor, const auto& tf, const auto& coll_tf) {
    // Ignore non-robot nodes
    if (node->is_object || node->is_obstacle) {
      return;
    }

    // We overapproximate each collision geometry as a sphere
    if (node->geom != nullptr) {
      btScalar bRadius;
      btVector3 bCenter;
      node->geom->getBoundingSphere(bCenter, bRadius);
      radius = bRadius;
    }

    const Eigen::Vector3d& translation = coll_tf.translation();

    // Update current limits
    update_bounds(min_bounds, max_bounds, translation[0], translation[1], translation[2], radius);

    // Update robot diameter
    const auto position_vector = translation - base_translation;
    const auto furthest_point  = position_vector.norm() + radius;
    if (furthest_point > robot_radius) {
      robot_radius = furthest_point;
    }
  };

  const Transform3r base_tf(*robot->base_pose);
  sg->update_transforms<double>(nullptr, nullptr, base_tf, robot_walker);

  // Now that we have the "correct" robot radius, check for bounds updates
  update_bounds(min_bounds,
                max_bounds,
                base_translation[0],
                base_translation[1],
                base_translation[2],
                robot_radius);
  log->info("Robot radius: {}", robot_radius);

  // Handle the objects
  for (const auto& object : fplus::get_map_values(*objects)) {
    object->get_bounding_sphere(x, y, z, radius);
    log->info("Bounds for {} are ({}, {}, {}) with radius {}", object->name, x, y, z, radius);
    update_bounds(min_bounds, max_bounds, x, y, z, radius);
  }

  // Handle the obstacles
  for (const auto& obstacle : fplus::get_map_values(*obstacles)) {
    obstacle->get_bounding_sphere(x, y, z, radius);
    log->info("Bounds for {} are ({}, {}, {}) with radius {}", obstacle->name, x, y, z, radius);
    update_bounds(min_bounds, max_bounds, x, y, z, radius);
  }

  return std::make_tuple(std::move(min_bounds), std::move(max_bounds), robot_radius);
}

std::shared_ptr<ob::CompoundStateSpace>
make_robot_cspace(const structures::robot::Robot* const robot) {
  /// Make the robot's configuration space based on its joints and the mobility of its base
  auto robot_space = std::make_shared<ob::CompoundStateSpace>();
  robot_space->setName(ROBOT_SPACE);

  // Make the joint subspace(s)
  auto joint_space = std::make_shared<RobotJointSpace>();
  joint_space->setName(JOINT_SPACE);
  int idx = 0;
  for (const auto& joint_data : robot->controllable_joints) {
    const auto& [joint_name, lower, upper, _] = joint_data.second;
    if (lower == upper) {
      log->debug("Joint {} has zero-measure bounds; assuming fixed and not adding", joint_name);
      continue;
    }

    // NOTE: The below comparison is OK because f == f for all floating point f, and we're
    // getting the same representation here both times
    if (robot->tree_nodes.at(joint_name)->type == structures::scenegraph::Node::Type::CONTINUOUS) {
      log->debug("Joint {} is continuous. Adding SO(2) subspace!", joint_name);
      auto cont_joint_subspace = std::make_shared<ob::SO2StateSpace>();
      cont_joint_subspace->setName(joint_name);
      robot_space->addSubspace(cont_joint_subspace, 1.0);
      const auto joint_idx                  = robot_space->getSubspaceIndex(joint_name);
      robot->tree_nodes.at(joint_name)->idx = joint_idx;
      cont_joint_idxs.push_back(joint_idx);
      num_dims += 1;
    } else {
      log->debug("Added dimension for joint {} bounded by ({}, {})", joint_name, lower, upper);
      joint_space->addDimension(joint_name, lower, upper);
      // This tracks the index of this controllable joint in the robot state vector
      robot->tree_nodes.at(joint_name)->idx = idx;
      ++idx;
      joint_bounds.push_back({lower, upper});
    }
  }

  // TODO(Wil): Should probably add a dimension for the torso if one exists

  // Add SE(3) for the robot base if it's movable
  if (robot->base_movable) {
    auto base_space = std::make_shared<RobotBaseSpace>();
    base_space->setName(BASE_SPACE);
    // Note: We restrict the Z coordinate of the base pose to its initial value
    auto base_bounds = *workspace_bounds;
    base_bounds.setLow(2, robot->base_pose->translation().z());
    base_bounds.setHigh(2, robot->base_pose->translation().z());
    base_space->setBounds(base_bounds);
    robot_space->addSubspace(base_space, 1.0);
    log->debug("Added SE(3) for base mobility");
    // We represent the SE(3) copy with (x, y, z, qx, qy, qz, qw)
    num_dims += 4;
  }


  // Add in the joint subspace
  robot_space->addSubspace(joint_space, 1.0);
  num_dims += joint_space->getDimension();
  const auto& bounds = joint_space->getBounds();
  for (unsigned int i = 0; i < joint_space->getDimension(); ++i) {
    const auto& name = joint_space->getDimensionName(i);
    log->debug("Robot dimension '{}' at index {} has bounds [{}, {}]",
               name,
               i,
               bounds.low[i],
               bounds.high[i]);
  }

  return robot_space;
}

std::shared_ptr<ob::CompoundStateSpace>
make_object_cspace(const structures::object::ObjectSet* const objects) {
  /// Make a configuration space with a copy of SE(3) for each movable object
  // NOTE: I hack on a pad on the workspace bounds for the object subspaces to account for posing
  ob::RealVectorBounds object_bounds(*workspace_bounds);
  object_bounds.low[0] -= 0.3;
  object_bounds.low[1] -= 0.3;
  object_bounds.low[2] -= 0.3;
  object_bounds.high[0] += 0.3;
  object_bounds.high[1] += 0.3;
  object_bounds.high[2] += 0.3;
  auto object_space = std::make_shared<ob::CompoundStateSpace>();
  object_space->setName(OBJECT_SPACE);
  log->debug("Adding {} copies of SE(3) for movable objects", objects->size());
  for (const auto& [obj_name, _] : *objects) {
    log->debug("Making SE(3) copy for {}", obj_name);
    auto obj = std::make_shared<ObjectSpace>();
    obj->setName(obj_name);
    obj->setBounds(object_bounds);
    object_space->addSubspace(obj, 1.0);
  }

  return object_space;
}


std::shared_ptr<ob::CompoundStateSpace>
make_eqclass_cspace(const input::specification::Domain* const domain) {
  /// Make discrete dimensions for each discrete dimension that can modify the "valid sample"
  /// space for a universe; i.e. the set of equivalence class identifying dimensions
  auto eqclass_space = std::make_shared<ob::CompoundStateSpace>();
  eqclass_space->setName(EQCLASS_SPACE);
  if (domain->eqclass_dimension_ids.empty()) {
    log->critical("No kinematic predicates found! This will probably cause a crash, and you're "
                  "probably using the wrong tool...");
  } else {
    for (const auto& [dim_name, dim_idx] : fwd::apply(domain->eqclass_dimension_ids,
                                                      fwd::map_to_pairs(),
                                                      fwd::sort_by([](const auto&a, const auto&b) {
                                                        return a.second < b.second;
                                                      }))) {
      log->debug("Making eqclass dimension for {}", dim_name);
      auto disc_space = std::make_shared<DiscreteSpace>(0, 1);
      disc_space->setName(dim_name);
      eqclass_space->addSubspace(disc_space, 1.0);
      if (eqclass_space->getSubspaceIndex(dim_name) != dim_idx) {
        log->warn("Subspace index for {} isn't what was expected: {} vs {}",
                  dim_name,
                  eqclass_space->getSubspaceIndex(dim_name),
                  dim_idx);
      }
    }
  }

  return eqclass_space;
}

std::shared_ptr<ob::CompoundStateSpace>
make_discrete_cspace(const input::specification::Domain* const domain) {
  auto discrete_space = std::make_shared<ob::CompoundStateSpace>();
  discrete_space->setName(DISCRETE_SPACE);
  if (domain->discrete_dimension_ids.empty()) {
    log->error("No discrete predicates found! This is weird, and will probably cause a crash");
  } else {
    for (const auto& [dim_name, dim_idx] : fwd::apply(domain->discrete_dimension_ids,
                                                      fwd::map_to_pairs(),
                                                      fwd::sort_by([](const auto&a, const auto&b) {
                                                        return a.second < b.second;
                                                      }))) {
      log->debug("Making discrete dimension for {}", dim_name);
      auto disc_space = std::make_shared<DiscreteSpace>(0, 1);
      disc_space->setName(dim_name);
      discrete_space->addSubspace(disc_space, 1.0);
      if (discrete_space->getSubspaceIndex(dim_name) != dim_idx) {
        log->warn("Subspace index for {} isn't what was expected: {} vs {}",
                  dim_name,
                  discrete_space->getSubspaceIndex(dim_name),
                  dim_idx);
      }
    }
  }

  return discrete_space;
}

std::tuple<std::shared_ptr<CompositeSpace>,
           std::shared_ptr<ob::CompoundStateSpace>,
           std::shared_ptr<ob::CompoundStateSpace>,
           std::shared_ptr<ob::CompoundStateSpace>,
           std::shared_ptr<ob::CompoundStateSpace>>
make_cspace(const structures::robot::Robot* const robot,
            structures::scenegraph::Graph* const sg,
            const input::specification::Domain* const domain,
            const structures::object::ObjectSet* const objects,
            const structures::object::ObjectSet* const obstacles,
            const std::optional<std::array<structures::object::Bounds, 3>>& workspace_bounds_) {
  /// Call the various configuration space generating functions to make the overall compound
  /// space
  auto cspace = std::make_shared<CompositeSpace>();

  workspace_bounds = std::make_unique<ob::RealVectorBounds>(3);
  if (workspace_bounds_) {
    const auto& x_bounds = workspace_bounds_->at(0);
    const auto& y_bounds = workspace_bounds_->at(1);
    const auto& z_bounds = workspace_bounds_->at(2);

    workspace_bounds->setLow(0, x_bounds.low);
    workspace_bounds->setHigh(0, x_bounds.high);

    workspace_bounds->setLow(1, y_bounds.low);
    workspace_bounds->setHigh(1, y_bounds.high);

    workspace_bounds->setLow(2, z_bounds.low);
    workspace_bounds->setHigh(2, z_bounds.high);
  } else {
    const auto& [min_bounds, max_bounds, robot_radius] =
    make_workspace_bounds(robot, sg, objects, obstacles);
    for (int i = 0; i < 2; ++i) {
      workspace_bounds->setLow(i, min_bounds[i] - robot_radius);
      workspace_bounds->setHigh(i, max_bounds[i] + robot_radius);
    }
    workspace_bounds->setHigh(2, max_bounds[2] + robot_radius);
    workspace_bounds->setLow(2, 0.0);
  }


  auto robot_space = make_robot_cspace(robot);
  cspace->addSubspace(robot_space, 1.0);
  cspace->robot_space_idx = cspace->getSubspaceIndex(ROBOT_SPACE);
  if (robot->base_movable) {
    cspace->base_space_idx = robot_space->getSubspaceIndex(BASE_SPACE);
  } else {
    cspace->base_space_idx = -1;
  }

  cspace->joint_space_idx = robot_space->getSubspaceIndex(JOINT_SPACE);
  cspace->base_movable    = robot->base_movable;

  auto objects_space = make_object_cspace(objects);
  cspace->addSubspace(objects_space, 1.0);
  cspace->objects_space_idx = cspace->getSubspaceIndex(OBJECT_SPACE);
  cspace->num_objects       = objects_space->getSubspaceCount();

  log->info("Workspace bounds are:\n\tlow: {}\n\thigh: {}",
            workspace_bounds->low,
            workspace_bounds->high);

  auto universe_space = make_eqclass_cspace(domain);
  cspace->addSubspace(universe_space, 1.0);
  cspace->eqclass_space_idx      = cspace->getSubspaceIndex(EQCLASS_SPACE);
  cspace->eqclass_subspace_count = universe_space->getSubspaceCount();

  auto discrete_space = make_discrete_cspace(domain);
  cspace->addSubspace(discrete_space, 1.0);
  cspace->discrete_space_idx      = cspace->getSubspaceIndex(DISCRETE_SPACE);
  cspace->discrete_subspace_count = discrete_space->getSubspaceCount();

  return std::make_tuple(std::move(cspace),
                         std::move(robot_space),
                         std::move(objects_space),
                         std::move(universe_space),
                         std::move(discrete_space));
}
}  // namespace planner::cspace
