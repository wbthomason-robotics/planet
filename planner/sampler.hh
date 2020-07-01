#pragma once
#ifndef SAMPLER_HH
#define SAMPLER_HH

#include <mutex>
#include <random>
#include <utility>

#include "common.hh"

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/util/RandomNumbers.h>

#include "cspace.hh"
#include "robot.hh"
#include "heuristic.hh"
#include "predicate.hh"
#include "specification.hh"
#include "scene.hh"
#include "planner_utils.hh"

#include "debug.hh"

namespace planner::sampler {
namespace ob    = ompl::base;
namespace spec  = input::specification;
namespace scene = input::scene;
using namespace planner::util;

// Needed for goal sampling
extern UniversePtr goal_universe_data;

extern ActionLog* action_log;

struct Counters {
  int uniformNear;
  int gaussian;
  int uniformTotal;
  int heuristic;
  int uniformNormal;
};

extern Counters sample_counter;
extern std::mutex counter_mutex;

class TampSampler : public ob::StateSampler {
 public:
  TampSampler(const ob::StateSpace* si,
              const spec::Domain* const domain,
              const structures::robot::Robot* const robot
              IF_ACTION_LOG((, std::shared_ptr<debug::GraphLog> graph_log)));
  void sampleUniform(ob::State* state) override;
  void sampleUniformNear(ob::State* state, const ob::State* near, double distance) override;
  void sampleGaussian(ob::State* state, const ob::State* mean, double stdDev) override;
  void cleanup() const;

  // This is public because the universe histogram logic needs access
  static util::UniverseMap* universe_map;
  static std::mutex universe_mutex;
  static unsigned int NUM_SOLVER_TRIES;
  static double COIN_BIAS;

 protected:
  ob::StateSamplerPtr robot_config_sampler;
  Str name;
  void ordinary_sample(ob::State* state);
  void ordinary_sample_with_uni(ob::State* state,
                                const Universe* universe,
                                const Config* config,
                                bool copy_uni,
                                bool update_sg_objs);
  void heuristic_sample(ob::State* state);

 private:
  static unsigned int sampler_count;
  void apply_action(const Action& action, UniverseSig& universe, ConfigSig& result_config) const;
  void pose_objects(structures::scenegraph::Graph* sg,
                    const ob::CompoundState* robot_state,
                    const ob::CompoundState* object_poses,
                    ob::CompoundState* const objects_state,
                    Map<Str, Transform3r>* const pose_map);
  const unsigned int objects_space_idx;
  const unsigned int universe_space_idx;
  const unsigned int discrete_space_idx;
  const unsigned int robot_space_idx;
  const ob::CompoundStateSpace* objects_space;
  const ob::CompoundStateSpace* universe_space;
  const ob::CompoundStateSpace* discrete_space;
  const ob::CompoundStateSpace* robot_space;
  const unsigned int joint_space_idx;

  ob::CompoundState* robot_state;
  ob::CompoundState* robot_near_state;
  ob::CompoundState* objects_state;
  ob::CompoundState* universe_state;
  ob::CompoundState* discrete_state;
  cspace::CompositeSpace::StateType* start_state;

  std::shared_ptr<spdlog::logger> log;
  std::random_device rando;
  const spec::Domain* const domain;

  // Needed for gradient descent
  const structures::robot::Robot* const robot;

  std::unique_ptr<symbolic::predicate::LuaEnv<bool>> correctness_env;
  std::unique_ptr<symbolic::predicate::LuaEnv<double>> gradient_env;
  IF_ACTION_LOG(std::shared_ptr<debug::GraphLog> graph_log;)
};

ob::StateSamplerPtr allocTampSampler(const ob::StateSpace* space,
                                     const spec::Domain* const domain,
                                     const structures::robot::Robot* const robot IF_ACTION_LOG(
                                     (, std::shared_ptr<debug::GraphLog> graph_log)));
}  // namespace planner::sampler
#endif
