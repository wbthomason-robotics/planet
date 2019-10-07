#pragma once
#ifndef COLLISION_HH
#define COLLISION_HH
#include "common.hh"

#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <utility>

#include <boost/container_hash/hash.hpp>
#include <boost/dynamic_bitset.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <bullet/btBulletCollisionCommon.h>

#include "tsl/robin_set.h"

#include "cspace.hh"
#include "planner_utils.hh"
#include "scene.hh"
#include "scenegraph.hh"

namespace planner::collisions {
namespace ob    = ompl::base;
namespace scene = input::scene;
using Graph     = structures::scenegraph::Graph;
using Robot     = structures::robot::Robot;

extern unsigned int oob_count;
extern unsigned int self_coll_count;
extern unsigned int world_coll_count;
// extern std::unordered_map<std::pair<Str, Str>, unsigned int, boost::hash<std::pair<Str, Str>>>
// collision_counters;
// extern std::ofstream* collisions_file;
// extern std::ofstream* sample_data_file;
class NeighborLinksFilter : public btOverlapFilterCallback {
 public:
  NeighborLinksFilter(const std::optional<Str>& blacklist_path, std::size_t num_items, Graph* sg);
  bool
  needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;
  Graph* sg = nullptr;
  Map<Str, std::size_t> index_map;

 private:
  Vec<boost::dynamic_bitset<>> blacklist;
};

class CollisionChecker : public ob::StateValidityChecker {
 public:
  CollisionChecker(const ob::SpaceInformationPtr& si, const Robot* const robot)
  : ob::StateValidityChecker(si)
  , si(si)
  , space(si->getStateSpace()->as<cspace::CompositeSpace>())
  , robot(robot)
  , robot_index(space->getSubspaceIndex(cspace::ROBOT_SPACE))
  , eqclass_index(space->getSubspaceIndex(cspace::EQCLASS_SPACE))
  , objects_index(space->getSubspaceIndex(cspace::OBJECT_SPACE))
  , objects_space(space->getSubspace(objects_index)->as<ob::CompoundStateSpace>())
  , robot_space(space->getSubspace(robot_index)->as<ob::CompoundStateSpace>())
  , joints_index(robot_space->getSubspaceIndex(cspace::JOINT_SPACE)) {}


  virtual bool isValid(const ob::State* state) const = 0;

 protected:
  const ob::SpaceInformationPtr si;
  const cspace::CompositeSpace* const space;
  const Robot* const robot;
  const unsigned int robot_index;
  const unsigned int eqclass_index;
  const unsigned int objects_index;
  const ob::CompoundStateSpace* const objects_space;
  const ob::CompoundStateSpace* const robot_space;
  const unsigned int joints_index;

  virtual void output_json() const = 0;
};

class BulletCollisionChecker : public CollisionChecker {
 public:
  BulletCollisionChecker(const ob::SpaceInformationPtr& si,
                         const scene::ObjectSet& objects,
                         const scene::ObjectSet& obstacles,
                         const Robot* robot,
                         const std::optional<Str>& blacklist_path,
                         Graph* sg);

  bool isValid(const ob::State* state) const override;

 private:
  // Bullet machinery
  Vec<std::unique_ptr<btCollisionObject>> obstacle_collisions;
  Map<Str, std::unique_ptr<btCollisionObject>> object_collisions;
  Map<Str, std::unique_ptr<btCollisionObject>> robot_collisions;
  std::unique_ptr<btCollisionWorld> collision_world;
  std::unique_ptr<btCollisionConfiguration> collision_config;
  std::unique_ptr<btCollisionDispatcher> collision_dispatch;
  std::unique_ptr<btBroadphaseInterface> broadphase_interface;
  std::unique_ptr<NeighborLinksFilter> broadphase_filter;

  void output_json() const override;
};

constexpr double PENETRATION_EPSILON  = 0.021;
constexpr int OBJECTS_COLLISION_GROUP = 1;
constexpr int OBJECTS_COLLISION_MASK  = 2;
constexpr int ROBOT_COLLISION_GROUP   = 2;
constexpr int ROBOT_COLLISION_MASK    = 3;
}  // namespace planner::collisions
#endif
