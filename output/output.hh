#pragma once
#ifndef OUTPUT_HH
#define OUTPUT_HH

#include "common.hh"

#include <array>
#include <optional>

#include <ompl/geometric/PathGeometric.h>

#include "planner_utils.hh"
#include "robot.hh"
#include "sampler.hh"

#include "tsl/hopscotch_map.h"

namespace output {
namespace ob = ompl::base;

struct Pose {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;
};

// The description of one plan timestep
struct PlanStep {
  Map<Str, double> joint_config;
  Pose base_pose;
  Map<Str, Pose> object_poses;
  std::optional<Vec<Str>> symbolic_action;
};

Vec<PlanStep> construct_plan(ompl::geometric::PathGeometric* solution_path,
                             const planner::util::ActionLog& action_log,
                             const structures::robot::Robot& robot,
                             const ob::SpaceInformationPtr& si);

void output_plan(const Vec<PlanStep>& plan, const Str& output_filepath);
}  // namespace output
#endif
