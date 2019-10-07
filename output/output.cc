#include "output.hh"

#include <fstream>

#include <fmt/format.h>

#include <nlohmann/json.hpp>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <fplus/fplus.hpp>

#include "cspace.hh"

namespace output {

namespace cspace = planner::cspace;

using json = nlohmann::json;

namespace {
  auto log = spdlog::stdout_color_mt("plan-output");
}

Vec<PlanStep> construct_plan(ompl::geometric::PathGeometric* const solution_path,
                             const planner::util::ActionLog* const action_log,
                             const structures::robot::Robot* const robot,
                             const ob::SpaceInformationPtr& si) {
  log->info("Plan has {} states", solution_path->getStateCount());
  Vec<PlanStep> result;
  const auto full_space = si->getStateSpace()->as<ob::CompoundStateSpace>();

  const auto robot_space_idx = full_space->getSubspaceIndex(cspace::ROBOT_SPACE);
  const auto robot_space     = full_space->as<ob::CompoundStateSpace>(cspace::ROBOT_SPACE);

  const auto joints_space_idx = robot_space->getSubspaceIndex(cspace::JOINT_SPACE);
  const auto joints_space     = robot_space->as<ob::RealVectorStateSpace>(cspace::JOINT_SPACE);

  const auto objects_space_idx = full_space->getSubspaceIndex(cspace::OBJECT_SPACE);
  const auto objects_space     = full_space->as<ob::CompoundStateSpace>(cspace::OBJECT_SPACE);

  for (const auto& state_ptr : solution_path->getStates()) {
    const auto state = state_ptr->as<ob::CompoundState>();
    auto robot_state = state->as<ob::CompoundState>(robot_space_idx);

    // Get joint poses
    Map<Str, double> joint_poses;
    const auto& joint_states =
    robot_state->as<ob::RealVectorStateSpace::StateType>(joints_space_idx)->values;
    for (unsigned int i = 0; i < joints_space->getDimension(); ++i) {
      joint_poses.emplace(joints_space->getDimensionName(i), joint_states[i]);
    }

    // Get base pose
    Pose base_pose;
    if (robot->base_movable) {
      const auto base_space_idx = robot_space->getSubspaceIndex(cspace::BASE_SPACE);
      const auto& base_state = robot_state->as<cspace::RobotBaseSpace::StateType>(base_space_idx);
      base_pose.translation  = {base_state->getX(), base_state->getY(), base_state->getZ()};
      const auto& base_rotation_state = base_state->rotation();
      const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(base_rotation_state.value, Eigen::Vector3d::UnitZ()));
      base_pose.rotation = {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
    } else {
      const auto& translation = robot->base_pose->translation();
      const Eigen::Quaterniond rotation(robot->base_pose->linear());
      base_pose.translation = {translation[0], translation[1], translation[2]};
      base_pose.rotation    = {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
    }

    // Get symbolic action, if any
    std::optional<Vec<Str>> symbolic_action;
    for (const auto& [log_state, log_action] : *action_log) {
      if (full_space->equalStates(log_state, state_ptr)) {
        // Extract the action instantiation
        symbolic_action.emplace({log_action->action->name});
        for (const auto& [_, param] : log_action->bindings) {
          (*symbolic_action).emplace_back(param);
        }
      }
    }

    // Get object poses
    Map<Str, Pose> object_poses;
    const auto& objects_state = state->as<ob::CompoundState>(objects_space_idx);
    for (unsigned int i = 0; i < objects_space->getSubspaceCount(); ++i) {
      const auto& object_state = objects_state->as<cspace::ObjectSpace::StateType>(i);

      const auto& object_rotation = object_state->rotation();
      object_poses.emplace(
      objects_space->getSubspace(i)->getName(),
      Pose{{object_state->getX(), object_state->getY(), object_state->getZ()},
           {object_rotation.x, object_rotation.y, object_rotation.z, object_rotation.w}});
    }

    result.emplace_back(PlanStep{joint_poses, base_pose, object_poses, symbolic_action});
  }

  return result;
}

// Used implicitly in output_plan
void to_json(json& j, const Pose& pose) {
  j["translation"] = pose.translation;
  j["rotation"]    = pose.rotation;
}

void to_json(json& j, const PlanStep& ps) {
  j["joints"]       = ps.joint_config;
  j["base_pose"]    = ps.base_pose;
  j["object_poses"] = ps.object_poses;
  if (ps.symbolic_action) {
    j["action"] = *ps.symbolic_action;
  }
}


void output_plan(const Vec<PlanStep>& plan, const Str& output_filepath) {
  json out_json(plan);
  std::ofstream output_file(output_filepath);
  if (!output_file.is_open()) {
    log->critical("Couldn't open file {} for plan output!", output_filepath);
    return;
  }

  output_file << std::setw(4) << out_json << std::endl;
  output_file.flush();
}
}  // namespace output
