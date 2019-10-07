#pragma once
#ifndef COMMON_HH
#define COMMON_HH

// Common typedefs for a the other modules, to allow easy container/type swapping

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <bullet/btBulletCollisionCommon.h>

template <typename T> using Vec             = std::vector<T>;
template <typename K, typename V> using Map = std::unordered_map<K, V>;
template <typename T> using Set             = std::unordered_set<T>;
using Str                                   = std::string;
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#define NDEBUG
template <typename T> using Transform3     = Eigen::Transform<T, 3, Eigen::Isometry>;
using Transform3r                          = Eigen::Transform<double, 3, Eigen::Isometry>;
using Vector3r                             = Eigen::Vector3d;
template <typename T> using TransformPairT = std::pair<Transform3<T>, Transform3r>;
using TransformPair                        = TransformPairT<double>;

using CollisionShape     = btCollisionShape;
using CollisionTransform = btTransform;
#endif
