#pragma once
#ifndef MOTION_HH
#define MOTION_HH
/// Motion validation (to prevent motions between universes except where they connect)

#include <memory>
#include <utility>

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>

#include "cspace.hh"
#include "planner_utils.hh"
#include "sampler.hh"

namespace planner::motion {
namespace ob = ompl::base;
class UniverseMotionValidator : public ob::DiscreteMotionValidator {
 public:
  UniverseMotionValidator(ob::SpaceInformation* si)
  : ob::DiscreteMotionValidator(si)
  , space(si->getStateSpace()->as<ob::CompoundStateSpace>())
  , universe_idx(space->getSubspaceIndex(cspace::EQCLASS_SPACE))
  , discrete_idx(space->getSubspaceIndex(cspace::DISCRETE_SPACE)) {}
  UniverseMotionValidator(const ob::SpaceInformationPtr& si)
  : ob::DiscreteMotionValidator(si)
  , space(si->getStateSpace()->as<ob::CompoundStateSpace>())
  , universe_idx(space->getSubspaceIndex(cspace::EQCLASS_SPACE))
  , discrete_idx(space->getSubspaceIndex(cspace::DISCRETE_SPACE)) {}
  bool checkMotion(const ob::State* s1, const ob::State* s2) const override;
  bool checkMotion(const ob::State* s1,
                   const ob::State* s2,
                   std::pair<ob::State*, double>& lastValid) const override;

  static util::UniverseMap* universe_map;

 private:
  std::shared_ptr<ob::CompoundStateSpace> space;
  const unsigned int universe_idx;
  const unsigned int discrete_idx;
};

extern unsigned int invalid_end;
extern unsigned int invalid_interp;
}  // namespace planner::motion
#endif
