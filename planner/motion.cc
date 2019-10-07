#include "motion.hh"

#include <queue>
#include <stdexcept>

#include "fplus/fplus.hpp"

#include "fmt/ostream.h"
// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace planner::motion {
namespace {
  auto log = spdlog::stdout_color_mt("motion");
}  // namespace

unsigned int invalid_end                                 = 0;
unsigned int invalid_interp                              = 0;
std::ofstream* samples_file                              = nullptr;
util::UniverseMap* UniverseMotionValidator::universe_map = nullptr;
bool UniverseMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const {
  // Here we copy from DiscreteMotionValidator since we need to make sure interpolation is only
  // continuous and transitions are valid
  if (!si_->isValid(s2)) {
    // log->info("s2 is invalid");
    invalid_++;
    ++invalid_end;
    return false;
  }

  const auto* cstate1 = s1->as<util::HashableStateSpace::StateType>();
  const auto* cstate2 = s2->as<util::HashableStateSpace::StateType>();

  bool successful_transition = universe_map->check_valid_transition(cstate1, cstate2);
  bool result                = true;
  unsigned int nd            = space->validSegmentCount(s1, s2);

  /* initialize the queue of test positions */
  std::queue<std::pair<int, int>> pos;
  if (nd >= 2) {
    pos.push({1, nd - 1});

    /* temporary storage for the checked state */
    auto* test = si_->allocState()->as<util::HashableStateSpace::StateType>();

    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty()) {
      std::pair<int, int> x = pos.front();

      int mid = (x.first + x.second) / 2;
      space->interpolate(s1, s2, static_cast<double>(mid) / static_cast<double>(nd), test);

      if (!si_->isValid(test)) {
        result = false;
        ++invalid_interp;
        // log->warn("Intermediate state is invalid");
        break;
      }

      if (!successful_transition && universe_map->check_valid_transition(test, cstate2)) {
        // TODO(Wil): We'll need to add to the action log here once precondition checking is
        // added
        successful_transition = true;
      }

      pos.pop();

      if (x.first < mid) {
        pos.push({x.first, mid - 1});
      }

      if (x.second > mid) {
        pos.push({mid + 1, x.second});
      }
    }

    si_->freeState(test);
  }

  // log->info("Result: {} Successful transition: {}", result, successful_transition);
  result &= successful_transition;

  if (result) {
    valid_++;
  } else {
    invalid_++;
  }

  return result;
}

bool UniverseMotionValidator::checkMotion(const ob::State* s1,
                                          const ob::State* s2,
                                          std::pair<ob::State*, double>& lastValid) const {
  // Here we copy from DiscreteMotionValidator since our logic for lastValid is slightly
  // different and we can't just wrap the existing implementation
  bool result         = true;
  unsigned int nd     = space->validSegmentCount(s1, s2);
  const auto* cstate1 = s1->as<util::HashableStateSpace::StateType>();
  const auto* cstate2 = s2->as<util::HashableStateSpace::StateType>();


  bool successful_transition = universe_map->check_valid_transition(cstate1, cstate2);

  ob::State* prev = si_->allocState();
  si_->copyState(prev, s1);

  if (nd > 1) {
    auto* test = si_->allocState()->as<util::HashableStateSpace::StateType>();
    for (unsigned int j = 1; j < nd; ++j) {
      space->interpolate(s1, s2, static_cast<double>(j) / static_cast<double>(nd), test);
      if (!si_->isValid(test)) {
        lastValid.second = static_cast<double>(j - 1) / static_cast<double>(nd);
        if (lastValid.first != nullptr) {
          space->interpolate(s1, s2, lastValid.second, lastValid.first);
        }

        result = false;
        break;
      }

      if (!successful_transition && universe_map->check_valid_transition(test, cstate2)) {
        // TODO(Wil): We'll need to add to the action log here once precondition checking is
        // added
        successful_transition = true;
      }

      si_->copyState(prev, test);
    }

    si_->freeState(test);
  }

  result &= successful_transition;

  if (result) {
    if (!si_->isValid(s2)) {
      lastValid.second = static_cast<double>(nd - 1) / static_cast<double>(nd);
      if (lastValid.first != nullptr) {
        space->interpolate(s1, s2, lastValid.second, lastValid.first);
      }

      result = false;
    }
  }

  if (result) {
    valid_++;
  } else {
    invalid_++;
  }

  si_->freeState(prev);
  return result;
}
}  // namespace planner::motion
