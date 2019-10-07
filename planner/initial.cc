#include "initial.hh"

#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fplus/fplus.hpp"

namespace planner::initial {
void make_initial_state(ob::ScopedState<cspace::CompositeSpace>& initial_state,
                        const spec::Initial& discrete_init,
                        const structures::object::ObjectSet* const object_init,
                        const structures::robot::Robot* const robot_init,
                        const tsl::hopscotch_map<Str, spec::DimId>& eqclass_dimensions,
                        const tsl::hopscotch_map<Str, spec::DimId>& discrete_dimensions,
                        const ob::SpaceInformationPtr& si) {
  auto robot_space =
  si->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(cspace::ROBOT_SPACE);
  ob::ScopedState<ob::CompoundStateSpace> robot_state(robot_space);

  auto objects_space =
  si->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(cspace::OBJECT_SPACE);
  ob::ScopedState<ob::CompoundStateSpace> objects_state(objects_space);

  auto eqclass_space =
  si->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(cspace::EQCLASS_SPACE);
  ob::ScopedState<ob::CompoundStateSpace> eqclass_state(eqclass_space);

  auto discrete_space =
  si->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(cspace::DISCRETE_SPACE);
  ob::ScopedState<ob::CompoundStateSpace> discrete_state(discrete_space);

  // Fill the discrete state
  for (const auto& dim : discrete_init) {
    if (fplus::map_contains(eqclass_dimensions, dim)) {
      eqclass_state->as<ob::DiscreteStateSpace::StateType>(eqclass_dimensions.at(dim))->value = 1;
    } else {
      discrete_state->as<ob::DiscreteStateSpace::StateType>(discrete_dimensions.at(dim))->value =
      1;
    }
  }

  // Fill the robot state
  if (robot_init->base_movable) {
    const auto& base_pose        = *robot_init->base_pose;
    const auto& base_translation = base_pose.translation();
    const auto& base_rotation    = Eigen::Quaterniond(base_pose.linear());
    ob::ScopedState<cspace::RobotBaseSpace> base_state(
    robot_space->as<ob::CompoundStateSpace>()->getSubspace(cspace::BASE_SPACE));
    base_state->setXYZ(base_translation(0, 0), base_translation(1, 0), base_translation(2, 0));

    const double siny_cosp =
    2.0 * (base_rotation.w() * base_rotation.z() + base_rotation.x() * base_rotation.y());
    const double cosy_cosp =
    1.0 - 2.0 * (base_rotation.y() * base_rotation.y() + base_rotation.z() * base_rotation.z());
    const double yaw          = std::atan2(siny_cosp, cosy_cosp);
    auto& base_state_rotation = base_state->rotation();
    base_state_rotation.value = yaw;

    base_state >> robot_state;
  }

  auto joint_space = robot_space->as<ob::CompoundStateSpace>()->getSubspace(cspace::JOINT_SPACE);
  ob::ScopedState<cspace::RobotJointSpace> joints_state(joint_space);
  for (const auto& joint : robot_init->controllable_joints) {
    const auto& [joint_name, _l, _u, initial_value] = joint.second;
    if (robot_space->as<ob::CompoundStateSpace>()->hasSubspace(joint_name)) {
      // Joint is continuous and has SO(2) state
      const auto& cont_joint_space =
      robot_space->as<ob::CompoundStateSpace>()->getSubspace(joint_name);
      ob::ScopedState<ob::SO2StateSpace> cont_joint_state(cont_joint_space);
      cont_joint_state->value = initial_value;
      cont_joint_state >> robot_state;
    } else if (robot_init->tree_nodes.at(joint_name)->idx) {
      // Joint is bounded and has R state
      const auto idx             = robot_init->tree_nodes.at(joint_name)->idx;
      joints_state->values[*idx] = initial_value;
    }
  }

  joints_state >> robot_state;

  // Fill the object state
  for (const auto& [name, object] : *object_init) {
    const auto& pose = object->initial_pose;
    const auto& translation = pose.getOrigin();
    const auto& rotation    = pose.getRotation();

    ob::ScopedState<cspace::ObjectSpace> object_state(
    objects_space->as<ob::CompoundStateSpace>()->getSubspace(name));
    object_state->setXYZ(translation.x(), translation.y(), translation.z());

    auto& state_rotation = object_state->rotation();
    state_rotation.x     = rotation.x();
    state_rotation.y     = rotation.y();
    state_rotation.z     = rotation.z();
    state_rotation.w     = rotation.w();

    object_state >> objects_state;
  }

  // Fill the full initial state
  objects_state >> initial_state;
  robot_state >> initial_state;
  eqclass_state >> initial_state;
  discrete_state >> initial_state;
}
}  // namespace planner::initial
