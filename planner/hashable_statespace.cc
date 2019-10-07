#include "hashable_statespace.hh"

#include <boost/container_hash/hash.hpp>

namespace planner::util {
HashableStateSpace::StateType::StateType(const StateType& x) {
  space_ = x.space_;
  space_->allocStateComponents(this);
  space_->copyState(this, &x);
}

ob::State* HashableStateSpace::allocState() const {
  auto* state = new StateType();
  allocStateComponents(state);
  state->space_ = this;
  return static_cast<ob::State*>(state);
}

std::size_t hash_value(const HashableStateSpace::StateType& x) { return x.space_->computeHash(x); }

RobotBaseSpace::RobotBaseSpace() {
  setName("RobotBase" + getName());
  addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 1.0);
  addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
  lock();
}

std::size_t hash_value(const RobotBaseSpace::StateType& x) {
  size_t hash_val = 0;
  boost::hash_combine(hash_val, x.rotation());
  return hash_val;
}

void swap(HashableStateSpace::StateType& a, HashableStateSpace::StateType& b) {
  std::swap(a.components, b.components);
}

void HashableStateSpace::copyState(ob::State* destination, const ob::State* source) const {
  ob::CompoundStateSpace::copyState(destination, source);
  auto* cdest         = destination->as<StateType>();
  const auto* csource = source->as<StateType>();
  cdest->action       = csource->action;
  cdest->object_poses = csource->object_poses;
  cdest->space_       = csource->space_;
  cdest->sg           = csource->sg;
}
}  // namespace planner::util


namespace ompl::base {
std::size_t hash_value(const SO3StateSpace::StateType& x) {
  size_t hash_val = 0;
  boost::hash_combine(hash_val, x.x);
  boost::hash_combine(hash_val, x.y);
  boost::hash_combine(hash_val, x.z);
  boost::hash_combine(hash_val, x.w);
  return hash_val;
}

std::size_t hash_value(const SO2StateSpace::StateType& x) { return boost::hash_value(x.value); }

std::size_t hash_value(const DiscreteStateSpace::StateType& x) {
  return boost::hash_value(x.value);
}

std::size_t hash_value(const SE3StateSpace::StateType& x) {
  size_t hash_val = 0;
  boost::hash_combine(hash_val, x.getX());
  boost::hash_combine(hash_val, x.getY());
  boost::hash_combine(hash_val, x.getZ());
  boost::hash_combine(hash_val, x.rotation());
  return hash_val;
}
}  // namespace ompl::base
