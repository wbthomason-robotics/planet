#pragma once
#ifndef DISCRETE_DISTRIBUTION_HH
#define DISCRETE_DISTRIBUTION_HH

#include "common.hh"

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>

namespace planner::util {
template <typename V> struct DiscreteDistribution {
  struct ValueData {
    ValueData(const double mass, const V& data, double* const sum)
    : mass(mass), data(data), sum(sum) {}
    double mass = 0.0;
    V data;
    double* const sum;
    void update(const double m) {
      *sum -= mass;
      mass = m;
      *sum += m;
    }
  };

  DiscreteDistribution() = default;
  explicit DiscreteDistribution(Vec<std::pair<double, V>> initial_categories) : sum(0.0) {
    distribution.reserve(initial_categories.size());
    for (auto& [p, v] : initial_categories) {
      sum += p;
      distribution.emplace_back(std::make_unique<ValueData>(p, v, &sum));
    }

    // Sort the probabilities in ascending order
    std::sort(distribution.begin(), distribution.end(), [](const auto& a, const auto& b) {
      return a->mass < b->mass;
    });
  }

  [[nodiscard]] bool empty() const { return distribution.empty(); }

  void clear() {
    distribution.clear();
    sum = 0.0;
  }

  void add(const double p, const V& v) {
    sum += p;
    distribution.emplace_back(std::make_unique<ValueData>(p, v, &sum));
  }

  void sort() {
    // Insertion sort to maintain the sorted invariant. This is faster than std::sort (probably)
    // because the list is sorted except for (possibly) one element already, so running time
    // should be close to linear
    for (int i = 1; i < distribution.size(); ++i) {
      std::unique_ptr<ValueData> elem(distribution[i].release());
      int j = i - 1;
      while (j >= 0 && distribution[j]->mass > elem->mass) {
        distribution[j + 1].reset(distribution[j].release());
        --j;
      }

      distribution[j + 1].reset(elem.release());
    }
  }

  ValueData* sample(const double s) {
    // Ensure the sorted invariant
    sort();

    // Find the window containing the sample parameter and return the corresponding value
    double cumulative_prob_mass = 0.0;
    for (const auto& elem : distribution) {
      const double scaled_p = elem->mass / sum;
      if (cumulative_prob_mass < s && s <= cumulative_prob_mass + scaled_p) {
        return elem.get();
      }

      cumulative_prob_mass += scaled_p;
    }

    // We're guaranteed to return before we get here: s\in [0, 1] and Union(over p\in
    // distribution) of [cumulative_prob_mass, cumulative_prob_mass + scaled_p] = [0, 1]
  }

 private:
  double sum = 0.0;
  Vec<std::unique_ptr<ValueData>> distribution;
};
}  // namespace planner::util
#endif
