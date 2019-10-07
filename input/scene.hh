#pragma once
#ifndef SCENE_HH
#define SCENE_HH
#include "common.hh"

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "object.hh"
#include "robot.hh"
#include "scenegraph.hh"

namespace input::scene {
using ObjectSet = structures::object::ObjectSet;
using Object    = structures::object::Object;
using Robot     = structures::robot::Robot;
using JointData = structures::robot::JointData;
using Node      = structures::scenegraph::Node;
using Graph     = structures::scenegraph::Graph;
using Bounds    = structures::object::Bounds;

std::optional<
std::tuple<ObjectSet, ObjectSet, Robot, std::shared_ptr<Graph>, std::optional<std::array<Bounds, 3>>>>
load(const std::string& scene_path, const std::string& obj_dir);
}  // namespace input::scene
#endif
