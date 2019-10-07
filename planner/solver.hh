#pragma once
#ifndef SOLVER_HH
#define SOLVER_HH
#include <memory>

#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>

#include "cspace.hh"
#include "predicate.hh"
#include "robot.hh"
#include "specification.hh"

namespace planner::solver {
namespace spec = input::specification;
namespace pred = symbolic::predicate;
namespace ob   = ompl::base;
bool gradient_solve(const ob::StateSpace* const space,
                    const pred::LuaEnv<double>& grad_env,
                    const structures::robot::Robot* const robot,
                    const cspace::CompositeSpace::StateType* start,
                    spec::Formula* formula,
                    ob::State* result,
                    double& last_value);
}  // namespace planner::solver
#endif
