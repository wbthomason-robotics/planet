#pragma once
#ifndef HASH_HELPERS_HH
#define HASH_HELPERS_HH

#include <boost/container_hash/hash.hpp>
#include <boost/dynamic_bitset.hpp>

namespace std {
template <typename B, typename A> struct hash<boost::dynamic_bitset<B, A>> {
  size_t operator()(const boost::dynamic_bitset<B, A>& x) const {
    return boost::hash_value(x.m_bits);
  }
};

template <typename B, typename A>
struct hash<pair<boost::dynamic_bitset<B, A>, boost::dynamic_bitset<B, A>>> {
  size_t
  operator()(const pair<boost::dynamic_bitset<B, A>, boost::dynamic_bitset<B, A>>& x) const {
    size_t hash_val = 0;
    boost::hash_combine(hash_val, x.first.m_bits);
    boost::hash_combine(hash_val, x.second.m_bits);
    return hash_val;
  }
};
}  // namespace std
#endif /* end of include guard */
