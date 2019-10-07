#pragma once

#include "common.hh"

#include <limits>
#include <memory>
#include <optional>
#include <variant>

#include <bullet/btBulletCollisionCommon.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "scenegraph.hh"

namespace structures::object {
struct DiscreteGrasp {
  DiscreteGrasp(const Transform3r& frame) : frame(frame) {}
  const Transform3r frame;
};

struct ContinuousGrasp {
  ContinuousGrasp(const Transform3r& template_frame, Vector3r axis)
  : template_frame(template_frame), axis(std::move(axis)) {}
  const Transform3r template_frame;
  const Vector3r axis;
};

struct Bounds {
  Bounds(const double low, const double high) : low(low), high(high) {}
  const double low;
  const double high;
};

struct StableRegion {
  StableRegion(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
  : bounds({{"x", {x_min, x_max}}, {"y", {y_min, y_max}}, {"z", {z_min, z_max}}}) {}
  const Map<Str, Bounds> bounds;
};

using StablePlane = Vec<Vector3r>;

using Grasp = std::variant<DiscreteGrasp, ContinuousGrasp>;

using StableFace = std::variant<StableRegion, StablePlane>;

struct StablePose {
  StablePose(const Eigen::Quaterniond& template_rotation, Vector3r axis, const double distance)
  : template_rotation(template_rotation), axis(std::move(axis)), distance(distance) {}
  const Eigen::Quaterniond template_rotation;
  const Vector3r axis;
  const double distance;
};

struct Object {
  Object(Str name, const bool movable) : movable(movable), name(std::move(name)), geom(nullptr) {}
  const bool movable;
  const Str name;
  std::shared_ptr<CollisionShape> geom;
  CollisionTransform initial_pose;
  Vec<Grasp> grasps;
  std::optional<StableFace> stable_face;
  Vec<StablePose> stable_poses;

  void get_bounding_sphere(double& x, double& y, double& z, double& radius) {
    btVector3 center;
    btScalar bRadius;
    geom->getBoundingSphere(center, bRadius);
    const auto& trans = initial_pose.getOrigin();
    radius            = bRadius;
    x                 = trans.getX();
    y                 = trans.getY();
    z                 = trans.getZ();
  }
};

using ObjectSet = Map<Str, std::shared_ptr<Object>>;
}  // namespace structures::object
