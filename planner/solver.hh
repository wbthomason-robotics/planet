#pragma once
#ifndef SOLVER_HH
#define SOLVER_HH
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>

#include <memory>

#include "cspace.hh"
#include "predicate.hh"
#include "robot.hh"
#include "specification.hh"

namespace planner::solver {
namespace spec = input::specification;
namespace pred = symbolic::predicate;
namespace ob   = ompl::base;
bool gradient_solve(const ob::StateSpace* const space,
                    pred::LuaEnv<double>& grad_env,
                    const structures::robot::Robot* const robot,
                    const cspace::CompositeSpace::StateType* start,
                    const spec::Formula& formula,
                    const Map<Str, Str>& bindings,
                    ob::State* result,
                    double& last_value);
}  // namespace planner::solver
#endif
