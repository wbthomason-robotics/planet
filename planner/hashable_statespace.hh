#pragma once
#ifndef HASHABLE_STATESPACE_HH
#define HASHABLE_STATESPACE_HH

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include "discrete_distribution.hh"
#include "heuristic.hh"
#include "scenegraph.hh"

namespace planner::util {
using Action = symbolic::heuristic::PrioritizedActionPtr;
// Forward declaration to allow definition of ActionDistribution here
struct Universe;
struct Config;
using ActionDistribution =
DiscreteDistribution<std::tuple<Universe* const, Config* const, Action>>;

namespace ob = ompl::base;
struct HashableStateSpace : public ob::CompoundStateSpace {
  struct StateType : public ob::CompoundState {
    StateType() = default;
    StateType(const StateType& x);
    const HashableStateSpace* space_ = nullptr;
    bool operator==(const StateType& other) const { return space_->equalStates(this, &other); }
    util::ActionDistribution::ValueData* action = nullptr;
    const ob::CompoundState* object_poses       = nullptr;
    structures::scenegraph::Graph* sg           = nullptr;
  };

  [[nodiscard]] virtual size_t computeHash(const StateType& state) const { return 0; }
  [[nodiscard]] ob::State* allocState() const override;
  void copyState(ob::State* destination, const ob::State* source) const override;
};

std::size_t hash_value(const HashableStateSpace::StateType& x);

// Robot base space is a 3D pose and a 2D rotation
struct RobotBaseSpace : public ob::CompoundStateSpace {
  RobotBaseSpace();
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() = default;
    double getX() const { return as<ob::RealVectorStateSpace::StateType>(0)->values[0]; }
    double getY() const { return as<ob::RealVectorStateSpace::StateType>(0)->values[1]; }
    double getZ() const { return as<ob::RealVectorStateSpace::StateType>(0)->values[2]; }
    const ob::SO2StateSpace::StateType& rotation() const {
      return *as<ob::SO2StateSpace::StateType>(1);
    }

    ob::SO2StateSpace::StateType& rotation() { return *as<ob::SO2StateSpace::StateType>(1); }
    void setX(double x) { as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x; }
    void setY(double y) { as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y; }
    void setZ(double z) { as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z; }
    void setXYZ(double x, double y, double z) {
      setX(x);
      setY(y);
      setZ(z);
    }
  };

  ~RobotBaseSpace() override = default;
  void setBounds(const ob::RealVectorBounds& bounds) {
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
  }

  const ob::RealVectorBounds& getBounds() const {
    return as<ob::RealVectorStateSpace>(0)->getBounds();
  }

  ob::State* allocState() const override {
    auto* state = new StateType();
    allocStateComponents(state);
    return state;
  }

  void freeState(ob::State* state) const override { ob::CompoundStateSpace::freeState(state); }
};

std::size_t hash_value(const RobotBaseSpace::StateType& x);
void swap(HashableStateSpace::StateType& a, HashableStateSpace::StateType& b);
}  // namespace planner::util

namespace ompl::base {
std::size_t hash_value(const SO3StateSpace::StateType& x);
std::size_t hash_value(const SO2StateSpace::StateType& x);
std::size_t hash_value(const DiscreteStateSpace::StateType& x);
std::size_t hash_value(const SE3StateSpace::StateType& x);
}  // namespace ompl::base


namespace std {
template <> struct hash<planner::util::HashableStateSpace::StateType> {
  size_t operator()(const planner::util::HashableStateSpace::StateType& x) const {
    return hash_value(x);
  }
};
}  // namespace std
#endif /* end of include guard */
