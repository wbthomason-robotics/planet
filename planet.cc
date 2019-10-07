#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "common.hh"

#include <cxxopts.hpp>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include "cpptoml.h"
#include <boost/filesystem.hpp>
#include <boost/dynamic_bitset.hpp>
#include <fmt/ostream.h>
#include "fplus/fplus.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/PathSimplifier.h>

#include "collision.hh"
#include "compositenn.hh"
#include "cspace.hh"
#include "goal.hh"
#include "initial.hh"
#include "motion.hh"
#include "output.hh"
#include "planner_utils.hh"
#include "predicate.hh"
#include "robot.hh"
#include "sampler.hh"
#include "scene.hh"
#include "specification.hh"
#include "world_functions.hh"
#include "rrt.hh"

namespace cspace        = planner::cspace;
namespace goal          = planner::goal;
namespace initial       = planner::initial;
namespace ob            = ompl::base;
namespace sampler       = planner::sampler;
namespace scene         = input::scene;
namespace specification = input::specification;
namespace worldfns      = symbolic::worldfns;
namespace util          = planner::util;
namespace rrt           = planner::rrt;
namespace fwd           = fplus::fwd;

inline bool try_to_get_path(const Str& name,
                            const std::shared_ptr<cpptoml::table>& config,
                            Str* result,
                            const std::shared_ptr<spdlog::logger>& log,
                            const Str& message);

inline void print_unimap_data(const specification::Domain* const domain_ptr,
                              const std::shared_ptr<spdlog::logger>& log);

int main(int argc, char* argv[]) {
  spdlog::set_pattern("[%H:%M:%S] %^[%l @ %n]%$ %v");
  // spdlog::set_async_mode(1024); // Only enable if we need faster logging
  auto log = spdlog::stdout_color_mt("planet");

  cxxopts::Options opts("planet", "Task and Motion Planning through Magic");
  // clang-format off
  opts.add_options()
    ("p,problem", "The problem configuration file", cxxopts::value<Str>())
    ("r,reps", "Run repeatedly for testing", cxxopts::value<unsigned int>()->default_value("1"))
    ("v,verbose", "Be verbose (display debug output)")
    ("h,help", "Display this help message");
  // clang-format on
  opts.parse_positional({"problem"});
  auto args = opts.parse(argc, argv);
  if (args.count("help") > 0) {
    std::cout << opts.help();
    return EXIT_SUCCESS;
  }

  const auto v_count = args.count("verbose");
  if (v_count == 0) {
    spdlog::set_level(spdlog::level::info);
  } else if (v_count == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (v_count > 1) {
    spdlog::set_level(spdlog::level::trace);
  }

  if (args.count("problem") == 0) {
    spdlog::set_pattern("*** %^%v%$ ***");
    log->error("Please provide a problem configuration file");
    return EXIT_FAILURE;
  }

  auto problem_config = cpptoml::parse_file(args["problem"].as<Str>());
  auto homedir        = Str(getenv("HOME"));
  auto tilde_helper   = [&](Str tilde_path) {
    if (tilde_path[0] == '~') {
      return homedir + tilde_path.substr(1);
    }

    return tilde_path;
  };

  // Load the hyperparameters
  Str hyperparam_path;
  if (!try_to_get_path("params",
                       problem_config,
                       &hyperparam_path,
                       log,
                       "Could not load parameters file from problem config!")) {
    return EXIT_FAILURE;
  }

  auto hyperparams = cpptoml::parse_file(tilde_helper(hyperparam_path));

  // Load the domain and problem
  Str domain_path;
  if (!try_to_get_path("domain",
                       problem_config,
                       &domain_path,
                       log,
                       "Could not get domain file path from problem config!")) {
    return EXIT_FAILURE;
  }

  // Load the semantics file path
  Str semantics_file;
  if (!try_to_get_path("semantics",
                       problem_config,
                       &semantics_file,
                       log,
                       "Could not get semantics file path from problem config!")) {
    return EXIT_FAILURE;
  }

  Str output_file;
  if (!try_to_get_path("output",
                       problem_config,
                       &output_file,
                       log,
                       "Could not get output file path from problem config!")) {
    return EXIT_FAILURE;
  }

  Str perf_filename;
  if (!try_to_get_path("perf_file",
                       problem_config,
                       &perf_filename,
                       log,
                       "Could not get perf file path from problem config!")) {
    return EXIT_FAILURE;
  }

  log->debug("Parsing domain from {}", domain_path);
  Str err;
  std::ifstream domain_file(tilde_helper(domain_path));
  std::stringstream domain_stream;
  domain_stream << domain_file.rdbuf();
  auto domain_sexp = sexpresso::parse(domain_stream.str(), err);
  if (!err.empty()) {
    log->error("Error parsing domain: {}", err);
    return EXIT_FAILURE;
  }

  Str problem_path;
  if (!try_to_get_path("problem",
                       problem_config,
                       &problem_path,
                       log,
                       "Could not get problem file path from problem config!")) {
    return EXIT_FAILURE;
  }

  Str scene_path;
  if (!try_to_get_path("scene",
                       problem_config,
                       &scene_path,
                       log,
                       "Could not get scene file path from problem config!")) {
    return EXIT_FAILURE;
  }

  Str obj_dir;
  if (!try_to_get_path("objdir",
                       problem_config,
                       &obj_dir,
                       log,
                       "Could not get object directory path from problem config!")) {
    return EXIT_FAILURE;
  }

  log->debug("Parsing problem from {}", problem_path);
  std::ifstream problem_file(tilde_helper(problem_path));
  std::stringstream problem_stream;
  problem_stream << problem_file.rdbuf();
  auto problem_sexp = sexpresso::parse(problem_stream.str(), err);
  if (!err.empty()) {
    log->error("Error parsing problem: {}", err);
    return EXIT_FAILURE;
  }

  log->info("Loading problem and domain definitions...");
  auto [domain, init_atoms, goal_atoms] =
  specification::load(&domain_sexp, &problem_sexp, semantics_file);

  if (!domain) {
    log->error("Failed to load the domain!");
    return EXIT_FAILURE;
  }

  auto domain_ptr = std::make_unique<specification::Domain>(*domain);

  if (!init_atoms) {
    log->error("Failed to load the initial state!");
    return EXIT_FAILURE;
  }

  if (!goal_atoms) {
    log->error("Failed to load the goal!");
    return EXIT_FAILURE;
  }

  auto goal_ptr = std::make_unique<specification::Goal>(*goal_atoms);

  log->info("Loading scene with data files from {}.\n\nThis can take a long time if there's "
            "a lot of URDF to convert",
            obj_dir);
  auto scene_information = scene::load(tilde_helper(scene_path), tilde_helper(obj_dir));

  if (!scene_information) {
    log->error("Failed to load the scene and initial config!");
    return EXIT_FAILURE;
  }

  log->debug("Loaded scene and initial robot configuration");

  auto [objects, obstacles, robot, init_sg, workspace_bounds] = std::move(*scene_information);
  const auto objects_ptr   = std::make_unique<scene::ObjectSet>(objects);
  const auto obstacles_ptr = std::make_unique<scene::ObjectSet>(obstacles);
  const auto robot_ptr     = std::make_unique<structures::robot::Robot>(std::move(robot));

  auto [conf_space, robot_space, objects_space, universe_space, discrete_space] =
  cspace::make_cspace(robot_ptr.get(),
                      init_sg.get(),
                      domain_ptr.get(),
                      objects_ptr.get(),
                      obstacles_ptr.get(),
                      workspace_bounds);
  log->debug("Constructed configuration space");

  // Initialize state for world_functions
  worldfns::robot_space = robot_space.get();
  worldfns::robot       = robot_ptr.get();
  worldfns::objects     = objects_ptr.get();
  worldfns::obstacles   = obstacles_ptr.get();
  worldfns::state_size  = cspace::num_dims;
  log->debug("Initialized global state for Lua interface function");

  auto si = std::make_shared<ob::SpaceInformation>(conf_space);
  conf_space->setStateSamplerAllocator(
  std::bind(sampler::allocTampSampler, std::placeholders::_1, domain_ptr.get(), robot_ptr.get()));
  si->setMotionValidator(std::make_shared<planner::motion::UniverseMotionValidator>(si));
  ob::ScopedState<cspace::CompositeSpace> initial_state(si);
  initial::make_initial_state(initial_state,
                              *init_atoms,
                              objects_ptr.get(),
                              robot_ptr.get(),
                              domain_ptr->eqclass_dimension_ids,
                              domain_ptr->discrete_dimension_ids,
                              si);
  initial_state->sg = init_sg.get();
  log->debug("Initializing universe map");
  const auto action_log_ptr = std::make_unique<planner::util::ActionLog>();
  planner::util::action_log = action_log_ptr.get();
  sampler::action_log       = action_log_ptr.get();

  // Set hyperparameters
  sampler::TampSampler::NUM_GD_TRIES = hyperparams->get_as<unsigned int>("gd_tries").value_or(3);
  goal::MAX_SAMPLES = hyperparams->get_as<unsigned int>("max_samples").value_or(10);
  symbolic::heuristic::SUCCESS_SCALE =
  hyperparams->get_as<double>("success_scale").value_or(100.0);
  sampler::TampSampler::COIN_BIAS = hyperparams->get_as<double>("coin_bias").value_or(0.3);
  planner::util::GOAL_WEIGHT      = hyperparams->get_as<double>("goal_weight").value_or(2.0);

  const auto universe_map_ptr =
  std::make_unique<planner::util::UniverseMap>(*init_atoms,
                                               initial_state.get(),
                                               init_sg,
                                               goal_ptr.get(),
                                               domain_ptr.get(),
                                               objects_space.get(),
                                               si->getStateSpace().get(),
                                               conf_space->eqclass_space_idx,
                                               conf_space->discrete_space_idx,
                                               conf_space->objects_space_idx,
                                               robot_ptr->base_movable);
  sampler::TampSampler::universe_map                     = universe_map_ptr.get();
  cspace::CompositeSpace::universe_map                   = universe_map_ptr.get();
  planner::motion::UniverseMotionValidator::universe_map = universe_map_ptr.get();
  goal::CompositeGoal::universe_map                      = universe_map_ptr.get();
  const auto blacklist_path = problem_config->get_as<Str>("blacklist");
  log->info("Using Bullet for collisions");
  si->setStateValidityChecker(std::make_shared<planner::collisions::BulletCollisionChecker>(
  si,
  *objects_ptr,
  *obstacles_ptr,
  robot_ptr.get(),
  // NOTE: This is dumb, but cpptoml uses its own option type and I
  // don't want to include the entirety of cpptoml in collision just
  // for that one type in this one place
  blacklist_path ? std::make_optional(*blacklist_path) : std::nullopt,
  init_sg.get()));
  log->debug("Running space information setup...");
  si->setup();

  bool initial_bounds = si->satisfiesBounds(initial_state.get());
  if (!initial_bounds) {
    log->critical("Initial state doesn't satisfy bounds!");
    return EXIT_FAILURE;
  }

  log->info("Initial state satisfies bounds");

  bool initial_valid = si->isValid(initial_state.get());
  if (!initial_valid) {
    log->critical("Initial state doesn't pass validity check!");
    return EXIT_FAILURE;
  }

  log->info("Initial state is valid");
  log->info("Running sanity checks on configuration space...");
  conf_space->sanityChecks();

  auto goal_def =
  std::make_shared<goal::CompositeGoal>(si, domain_ptr.get(), goal_ptr.get(), robot_ptr.get());
  const auto reps = args["reps"].as<unsigned int>();
  for (unsigned int i = 0; i < reps; ++i) {
    universe_map_ptr->clear(*init_atoms,
                            initial_state.get(),
                            init_sg,
                            domain_ptr.get(),
                            goal_ptr.get(),
                            objects_space.get(),
                            conf_space->objects_space_idx);
    action_log_ptr->clear();
    log->info("Running rep {} of {}", i + 1, reps);
    auto problem_def = std::make_shared<ob::ProblemDefinition>(si);
    problem_def->addStartState(initial_state);
    problem_def->setGoal(goal_def);
    log->debug(
    "Made space information pointer, goal sampler, initial state, and problem definition. "
    "Ready to run!");
    const auto& humanize_atom = [&](const auto& e) {
      return std::make_tuple(domain_ptr->discrete_dimension_names.at(e.first), e.first, e.second);
    };

    const auto& humanize_eqatom = [&](const auto& e) {
      return std::make_tuple(domain_ptr->eqclass_dimension_names.at(e.first), e.first, e.second);
    };

    // Create and run planner
    auto planner          = std::make_shared<rrt::CompositeRRT>(si);
    planner->universe_map = universe_map_ptr.get();
    planner->setProblemDefinition(problem_def);
    log->info("Dim is: {}", domain->num_eqclass_dims + domain->num_symbolic_dims);
    planner->setNearestNeighbors<planner::nn::CompositeNearestNeighbors>();
    double time_limit = 20.0;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    int iters                   = -1;
    const unsigned int MAX_TIME = problem_config->get_as<unsigned int>("timeout").value_or(700);
    do {
      log->info("Attempting to plan for {}s...", time_limit);
      start = std::chrono::high_resolution_clock::now();
      planner->solve(
      ob::timedPlannerTerminationCondition(time_limit, std::min(time_limit / 100.0, 0.1)));
      end = std::chrono::high_resolution_clock::now();

      ob::PlannerData pd(si);
      planner->getPlannerData(pd);
      log->debug("Planner is {} far away", pd.properties["approx goal distance REAL"]);
      if (v_count > 1) {
        print_unimap_data(domain_ptr.get(), log);
      }

      log->info("Sample count:\n\t{} uniform near\n\t{} gaussian near\n\t{} uniform "
                "total\n\t{} "
                "uniform regular\n\t{} heuristic",
                sampler::sample_counter.uniformNear,
                sampler::sample_counter.gaussian,
                sampler::sample_counter.uniformTotal,
                sampler::sample_counter.uniformNormal,
                sampler::sample_counter.heuristic);
      log->info("Valid states: {}",
                sampler::sample_counter.uniformTotal -
                (planner::collisions::oob_count + planner::collisions::self_coll_count +
                 planner::collisions::world_coll_count));
      log->info("Invalid sample count:\n\t{} out of bounds\n\t{} self collision\n\t{} world "
                "collision",
                planner::collisions::oob_count,
                planner::collisions::self_coll_count,
                planner::collisions::world_coll_count);
      log->info("Invalid end states: {}\nInvalid interpolation states: {}",
                planner::motion::invalid_end,
                planner::motion::invalid_interp);

      if (v_count > 0) {
        // Get state config/uni histogram
        Map<sampler::UniverseSig, Map<sampler::ConfigSig, unsigned int>> histogram;
        histogram.reserve(sampler::TampSampler::universe_map->graph.size());
        for (unsigned int j = 0; j < pd.numVertices(); ++j) {
          const auto& state = pd.getVertex(j).getState();
          const auto& uni   = util::discrete_of_state(
          state->as<cspace::CompositeSpace::StateType>()->as<cspace::CompositeSpace::StateType>(
          conf_space->eqclass_space_idx),
          conf_space->eqclass_subspace_count);
          const auto& cf = util::discrete_of_state(
          state->as<cspace::CompositeSpace::StateType>()->as<cspace::CompositeSpace::StateType>(
          conf_space->discrete_space_idx),
          conf_space->discrete_subspace_count);

          histogram[uni][cf]++;
        }

        for (const auto& [uni, data] : histogram) {
          Vec<std::tuple<Str, size_t, bool>> uni_desc;
          for (size_t j = 0; j < uni.size(); ++j) {
            if (uni[j]) {
              uni_desc.push_back(humanize_eqatom(std::make_pair(j, uni[j])));
            }
          }

          log->critical("{} ({}):", uni, uni_desc);
          for (const auto& [cf, count] : data) {
            Vec<std::tuple<Str, size_t, bool>> config_desc;
            for (size_t j = 0; j < cf.size(); ++j) {
              if (cf[j]) {
                config_desc.push_back(humanize_atom(std::make_pair(j, cf[j])));
              }
            }

            log->critical("{}: {}, {}", count, cf, config_desc);
          }

          std::cout << "==========================================\n";
        }
      }

      ++iters;
      time_limit *= 2;
    } while (!problem_def->hasSolution() && time_limit < MAX_TIME);

    if (time_limit >= 2 * MAX_TIME) {
      log->error("Failed to find a solution before reaching time limit!");
      std::ofstream perf_file(perf_filename, std::ios_base::app);
      perf_file << "failed"
                << ", "
                << "1200" << std::endl;
      continue;
    }

    auto solution_path = problem_def->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    solution_path->interpolate();

    const auto num_states = solution_path->getStateCount();
    const auto time_taken =
    -(20.0 * (1 - std::pow(2, iters))) +
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
    log->info("Found a solution with {} states in {}s!", num_states, time_taken);
    log->info("Outputting to {}", output_file);
    output::output_plan(
    output::construct_plan(solution_path, action_log_ptr.get(), robot_ptr.get(), si),
    tilde_helper(output_file));
    log->info("Outputting performance info to {}", perf_filename);
    std::ofstream perf_file(perf_filename, std::ios_base::app);
    perf_file << num_states << ", " << time_taken << std::endl;
    log->info("All done!");
    dynamic_cast<sampler::TampSampler*>(planner->sampler_.get())->cleanup();
  }

  return EXIT_SUCCESS;
}

bool try_to_get_path(const Str& name,
                     const std::shared_ptr<cpptoml::table>& config,
                     Str* result,
                     const std::shared_ptr<spdlog::logger>& log,
                     const Str& message) {
  auto value = config->get_as<Str>(name);
  if (!value) {
    spdlog::set_pattern("*** %^%v%$ ***");
    log->error(message);
    return false;
  }

  *result = *value;
  return true;
}

void print_unimap_data(const specification::Domain* const domain_ptr,
                       const std::shared_ptr<spdlog::logger>& log) {
  const auto& humanize_atom = [&](const auto& e) {
    return std::make_tuple(domain_ptr->discrete_dimension_names.at(e.first), e.first, e.second);
  };

  const auto& humanize_eqatom = [&](const auto& e) {
    return std::make_tuple(domain_ptr->eqclass_dimension_names.at(e.first), e.first, e.second);
  };

  for (const auto& [_, uni] : sampler::TampSampler::universe_map->graph) {
    Vec<std::tuple<Str, size_t, bool>> uni_desc;
    const auto& universe = uni->sig;
    for (size_t i = 0; i < universe.size(); ++i) {
      uni_desc.push_back(humanize_eqatom(std::make_pair(i, universe[i])));
    }

    log->critical("Have {}, {} with configs:", universe, uni_desc);
    for (const auto& cf : uni->configs) {
      Vec<std::tuple<Str, size_t, bool>> config_desc;
      for (size_t i = 0; i < cf.first.size(); ++i) {
        config_desc.push_back(humanize_atom(std::make_pair(i, cf.first[i])));
      }

      log->critical("{}, {}", cf.first, config_desc);
    }

    std::cout << "==========================================\n";
  }
}
