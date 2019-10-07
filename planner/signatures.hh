#pragma once
#ifndef SIGNATURES_HH
#define SIGNATURES_HH

#include <memory>

#include <boost/dynamic_bitset.hpp>

namespace planner::util {
// Because the Universe is determined by a compound space of discrete spaces for each of the
// eqclass-identifying predicates, and we can compactly represent this with a bitvector. Also
// helps with hashing.
using DiscreteSig    = boost::dynamic_bitset<>;
using UniverseSig    = DiscreteSig;
using ConfigSig      = DiscreteSig;
using UniverseSigPtr = std::shared_ptr<UniverseSig>;
}  // namespace planner::util
#endif /* end of include guard */
