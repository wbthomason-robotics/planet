#pragma once
#ifndef INITIAL_HH
#define INITIAL_HH

#include "common.hh"

#include <memory>

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

#include "cspace.hh"
#include "object.hh"
#include "robot.hh"
#include "specification.hh"

#include "tsl/hopscotch_map.h"

namespace planner::initial {
namespace ob   = ompl::base;
namespace spec = input::specification;

void make_initial_state(ob::ScopedState<cspace::CompositeSpace>& initial_state,
                        const spec::Initial& discrete_init,
                        const structures::object::ObjectSet& object_init,
                        const structures::robot::Robot& robot_init,
                        const tsl::hopscotch_map<Str, spec::DimId>& eqclass_dimensions,
                        const tsl::hopscotch_map<Str, spec::DimId>& discrete_dimensions,
                        const ob::SpaceInformationPtr& si);
}  // namespace planner::initial
#endif
