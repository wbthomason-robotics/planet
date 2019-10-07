#pragma once
#ifndef CSPACE_HH
#define CSPACE_HH

#include "common.hh"

#include <array>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include "object.hh"
#include "planner_utils.hh"
#include "robot.hh"
#include "scenegraph.hh"
#include "specification.hh"

namespace planner::cspace {
constexpr char ROBOT_SPACE[]    = "robot";
constexpr char JOINT_SPACE[]    = "joints";
constexpr char BASE_SPACE[]     = "base";
constexpr char TORSO_SPACE[]    = "torso";
constexpr char OBJECT_SPACE[]   = "objects";
constexpr char EQCLASS_SPACE[]  = "eqclassdims";
constexpr char DISCRETE_SPACE[] = "discretedims";

namespace ob = ompl::base;
class CompositeSpace : public util::HashableStateSpace {
 public:
  void interpolate(const ob::State* from,
                   const ob::State* to,
                   const double t,
                   ob::State* state) const override;
  bool isDiscrete() const override;
  bool satisfiesBounds(const ob::State* state) const override;
  double distance(const ob::State* state1, const ob::State* state2) const override;
  void sanityChecks(double zero, double eps, unsigned int flags) const override;
  void sanityChecks() const override;
  double contDistance(const ob::State* state1, const ob::State* state2) const;
  size_t computeHash(const StateType& x) const override;

  bool base_movable;
  unsigned int robot_space_idx;
  // NOTE: Signed because if the base is not movable this is -1
  int base_space_idx;
  unsigned int joint_space_idx;
  unsigned int objects_space_idx;
  unsigned int num_objects;
  unsigned int discrete_space_idx;
  unsigned int discrete_subspace_count;
  unsigned int eqclass_space_idx;
  unsigned int eqclass_subspace_count;

  static util::UniverseMap* universe_map;
};

using ObjectSpace     = ob::SE3StateSpace;
using DiscreteSpace   = ob::DiscreteStateSpace;
using RobotJointSpace = ob::RealVectorStateSpace;

using RobotBaseSpace = util::RobotBaseSpace;

// NOTE: Spooky scary global state
// NOTE: IMPORTANT: This is the number of *gradient* dimensions, and *not* the dimensionality
// of the state space (e.g. a pose contributes 7, not 6, even though SE(3) has dimension 6
extern int num_dims;
extern std::unique_ptr<ob::RealVectorBounds> workspace_bounds;
extern Vec<int> cont_joint_idxs;
extern Vec<std::pair<double, double>> joint_bounds;

std::shared_ptr<ob::CompoundStateSpace>
make_robot_cspace(const structures::robot::Robot* const robot);

std::shared_ptr<ob::CompoundStateSpace>
make_object_cspace(const structures::object::ObjectSet* const objects);

std::shared_ptr<ob::CompoundStateSpace>
make_eqclass_cspace(const input::specification::Domain* const domain);

std::shared_ptr<ob::CompoundStateSpace>
make_discrete_cspace(const input::specification::Domain* const domain);

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
            const std::optional<std::array<structures::object::Bounds, 3>>& workspace_bounds_);
}  // namespace planner::cspace
#endif
