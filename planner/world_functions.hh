#pragma once
#ifndef WORLD_FUNCTIONS_HH
#define WORLD_FUNCTIONS_HH

#include <cstring>
#include <memory>
#include <unordered_map>
#include <utility>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

// clang-format off
#include "lua.hpp"
#include "lauxlib.h"
#include "lualib.h"
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/util/Exception.h>

#include "autodiff.hh"
#include "cspace.hh"
#include "object.hh"
#include "robot.hh"
#include "sampler.hh"

namespace symbolic::worldfns {
namespace {
  extern std::shared_ptr<spdlog::logger> log;
}

namespace cspace  = planner::cspace;
namespace sampler = planner::sampler;
namespace ob      = ompl::base;
namespace object  = structures::object;

using ObjectSet = structures::object::ObjectSet;
using Robot     = structures::robot::Robot;

constexpr int OBJECT_DATA_SIZE         = 7;
constexpr int OBJECT_METADATA_SIZE     = 3;
constexpr int GRASP_FRAME_DATA_SIZE    = 7;
constexpr int GRASP_TEMPLATE_DATA_SIZE = 2;
constexpr char const* OBJECT_FIELD_NAMES[]{"px", "py", "pz", "rx", "ry", "rz", "rw"};
constexpr int OBJ_STATE_SIZE = OBJECT_DATA_SIZE + OBJECT_METADATA_SIZE;


// TODO(Wil): Global variables are bad! Find a better way! Later!
extern ompl::base::CompoundStateSpace* robot_space;
extern int state_size;
extern ObjectSet* objects;
extern ObjectSet* obstacles;
extern Robot* robot;

constexpr char OBJECT_FN_NAME[] = "generate_objects";
int generate_objects(lua_State* L);
}  // namespace symbolic::worldfns
#endif
