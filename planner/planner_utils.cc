#include "planner_utils.hh"

#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <fmt/ostream.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on
//

#include "scenegraph.hh"

namespace planner::util {
ActionLog* action_log = nullptr;
DiscreteSig discrete_of_state(const ob::CompoundState* const state, const int num_dims) {
  DiscreteSig result(num_dims, 0);
  for (int i = 0; i < num_dims; ++i) {
    result.set(i, (*state)[i]->as<ob::DiscreteStateSpace::StateType>()->value == 1);
  }

  return result;
}

void state_of_discrete(const DiscreteSig& discrete, const ob::CompoundState* state) {
  for (size_t i = 0; i < discrete.size(); ++i) {
    state->components[i]->as<ob::DiscreteStateSpace::StateType>()->value = discrete[i] ? 1 : 0;
  }
}

void state_to_pose_data(const ob::CompoundState* const robot_state,
                        const ob::RealVectorStateSpace::StateType* const joint_state,
                        const Vec<int>& cont_joint_idxs,
                        const int base_space_idx,
                        double* const cont_vals,
                        double** const joint_vals,
                        Transform3r* base_tf) {
  for (size_t i = 0; i < cont_joint_idxs.size(); ++i) {
    cont_vals[i] = robot_state->as<ob::SO2StateSpace::StateType>(cont_joint_idxs[i])->value;
  }

  *joint_vals = joint_state->values;
  // If the base is immovable, cspace sets the index to -1
  if (base_space_idx >= 0) {
    const auto& base_state     = robot_state->as<RobotBaseSpace::StateType>(base_space_idx);
    base_tf->translation().x() = base_state->getX();
    base_tf->translation().y() = base_state->getY();
    base_tf->translation().z() = base_state->getZ();
    const auto& base_rotation  = base_state->rotation();
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(base_rotation.value, Vector3r::UnitZ()));
    base_tf->linear() = rotation.toRotationMatrix();
    // spdlog::warn("For base state:\nRot:\n{}\nTrans:\n{},\nbase tf is:\n{}",
    //              rotation.toRotationMatrix(),
    //              translation,
    //              base_tf->matrix());
  }
}

void state_to_pose_map(const ob::CompoundState* const objects_state,
                       const ob::CompoundStateSpace* const objects_space,
                       Map<Str, Transform3r>& pose_map) {
  for (unsigned int l = 0; l < objects_space->getSubspaceCount(); ++l) {
    const auto& obj_pose = objects_state->as<ob::SE3StateSpace::StateType>(l);
    const auto& obj_name = objects_space->getSubspace(l)->as<ob::SE3StateSpace>()->getName();
    auto object_tf       = Transform3r::Identity();

    object_tf.translation().x() = obj_pose->getX();
    object_tf.translation().y() = obj_pose->getY();
    object_tf.translation().z() = obj_pose->getZ();
    const auto& obj_rot         = obj_pose->rotation();
    object_tf.linear() =
    Eigen::Quaterniond(obj_rot.w, obj_rot.x, obj_rot.y, obj_rot.z).toRotationMatrix();
    pose_map[obj_name] = object_tf;
  }
}
}  // namespace planner::util
