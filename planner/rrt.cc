#include "rrt.hh"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace planner::rrt {
namespace {
  auto log = spdlog::stdout_color_st("RRT");
}

util::UniverseMap* CompositeRRT::universe_map = nullptr;
unsigned long num_too_far                     = 0;

ob::PlannerStatus CompositeRRT::solve(const ob::PlannerTerminationCondition& ptc) {
  checkValidity();
  ob::Goal* goal = pdef_->getGoal().get();
  auto* goal_s   = dynamic_cast<ob::GoalSampleableRegion*>(goal);

  while (const ob::State* st = pis_.nextStart()) {
    auto* motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
  }

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
              getName().c_str(),
              nn_->size());

  Motion* solution  = nullptr;
  Motion* approxsol = nullptr;
  double approxdif  = std::numeric_limits<double>::infinity();
  auto* rmotion     = new Motion(si_);
  ob::State* rstate = rmotion->state;
  ob::State* xstate = si_->allocState();

  while (!ptc) {
    // log->critical("Start of sample loop; {} states in NN", nn_->size());
    /* sample random state (with goal biasing) */
    if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else
      sampler_->sampleUniform(rstate);

    /* find closest state in the tree */
    // log->critical("Starting NN");
    Motion* nmotion   = nn_->nearest(rmotion);
    ob::State* dstate = rstate;
    // log->critical("Ending NN");

    /* find state to add */
    // log->critical("Starting distance");
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_) {
      // We set the action to nullptr because we won't actually reach the state where the action
      // was used
      rstate->as<util::HashableStateSpace::StateType>()->action = nullptr;
      // log->warn("{} is greater than {}. Interpolating!", d, maxDistance_);
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
      dstate = xstate;
      ++num_too_far;
    }

    auto* action_data = dstate->as<util::HashableStateSpace::StateType>()->action;

    // log->critical("Checking motion");
    if (si_->checkMotion(nmotion->state, dstate)) {
      if (action_data != nullptr) {
        action_data->update(std::get<2>(action_data->data)->update_success());
        IF_ACTION_LOG(graph_log->update_success(action_data);)
      }

      if (addIntermediateStates_) {
        std::vector<ob::State*> states;
        const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

        if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
          si_->freeState(states[0]);

        for (std::size_t i = 1; i < states.size(); ++i) {
          universe_map->added_state(states[i]->as<util::HashableStateSpace::StateType>());
          Motion* motion = new Motion;
          motion->state  = states[i];
          motion->parent = nmotion;
          nn_->add(motion);
          nmotion = motion;
        }
      } else {
        universe_map->added_state(dstate->as<util::HashableStateSpace::StateType>());
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        nn_->add(motion);
        nmotion = motion;
      }

      double dist = 0.0;
      bool sat    = goal->isSatisfied(nmotion->state, &dist);
      if (sat) {
        approxdif = dist;
        solution  = nmotion;
        break;
      }
      if (dist < approxdif) {
        approxdif = dist;
        approxsol = nmotion;
      }
    } else {
      if (action_data != nullptr) {
        action_data->update(std::get<2>(action_data->data)->update_failure());
        IF_ACTION_LOG(graph_log->update_failure(action_data);)
      }
      // log->warn("Invalid motion");
    }
  }

  bool solved      = false;
  bool approximate = false;
  if (solution == nullptr) {
    solution    = approxsol;
    approximate = true;
  }

  if (solution != nullptr) {
    lastGoalMotion_ = solution;

    /* construct the solution path */
    std::vector<Motion*> mpath;
    while (solution != nullptr) {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    auto path(std::make_shared<og::PathGeometric>(si_));
    for (int i = mpath.size() - 1; i >= 0; --i) path->append(mpath[i]->state);
    pdef_->addSolutionPath(path, approximate, approxdif, getName());
    solved = true;
  }

  si_->freeState(xstate);
  if (rmotion->state != nullptr) si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

  return ob::PlannerStatus(solved, approximate);
}
}  // namespace planner::rrt
