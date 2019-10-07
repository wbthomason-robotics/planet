#pragma once
#ifndef RRT_HH
#define RRT_HH

#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include "compositenn.hh"
#include "planner_utils.hh"

namespace planner::rrt {
namespace ob = ompl::base;
namespace og = ompl::geometric;
class CompositeRRT : public og::RRT {
 public:
  CompositeRRT(const ob::SpaceInformationPtr& si, bool addIntermediateStates = false)
  : og::RRT(si, addIntermediateStates) {}
  ob::PlannerStatus solve(const ob::PlannerTerminationCondition& ptc) override;

  template <template <typename T> class NN> void setNearestNeighbors() {
    if (nn_ && nn_->size() != 0) OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nn_ = std::make_shared<NN<Motion*>>();
    setup();
  }

  template <> void setNearestNeighbors<nn::CompositeNearestNeighbors>() {
    if (nn_ && nn_->size() != 0) OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nn_ = std::make_shared<nn::CompositeNearestNeighbors<Motion*>>(universe_map);
    setup();
  }

  ob::StateSamplerPtr sampler_;

  static util::UniverseMap* universe_map;
};
}  // namespace planner::rrt
#endif
