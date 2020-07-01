#include "scenegraph.hh"

#include <algorithm>
#include <stdexcept>

namespace structures::scenegraph {
// namespace {
//   auto log = spdlog::stdout_color_st("scenegraph");
// }

void Node::add_child(Node& child) {
  children.push_back(child.self_idx);
  child.parent = self_idx;
}

bool Node::has_child(const Node& child_candidate) const {
  return child_candidate.parent == self_idx;
}

void Node::remove_child(const Node& child) {
  for (auto it = children.begin(); it != children.end(); ++it) {
    if (*it == child.self_idx) {
      children.erase(it);
    }
  }
}

/// Deep-copy the scenegraph
// Node* Node::copy(tsl::robin_map<Str, std::unique_ptr<Node>>& node_map) const {
//   std::shared_ptr<CollisionShape> geom_copy(geom);
//   auto self_copy =
//   std::make_unique<Node>(name, type, transform, collision_transform, axis, geom_copy);
//   self_copy->idx         = idx;
//   self_copy->dn_cache    = dn_cache;
//   self_copy->real_cache  = real_cache;
//   self_copy->is_base     = is_base;
//   self_copy->is_obstacle = is_obstacle;
//   self_copy->is_object   = is_object;
//   for (const auto& child : children) {
//     self_copy->add_child(child->copy(node_map));
//   }
//
//   const auto& [self_copy_it, _] = node_map.emplace(name, std::move(self_copy));
//   return self_copy_it->second.get();
// }

// std::shared_ptr<Graph> Graph::copy() {
//   auto result   = std::make_shared<Graph>();
//   result->nodes = nodes;
//   result->trees = trees;
//
//   return result;
// }

Node& Graph::add_node(int parent_idx, Node node) {
  auto& new_node    = nodes.emplace_back(node);
  new_node.self_idx = nodes.size() - 1;
  idx_index.emplace(new_node.name, new_node.self_idx);
  if (parent_idx >= 0) {
    auto& parent = nodes[parent_idx];
    parent.add_child(new_node);
  } else {
    trees.push_back(new_node.self_idx);
  }

  return new_node;
}

Node& Graph::extract(const Str& name) {
  // NOTE: We assume that name is *always* in the graph for a slight optimization
  const auto idx = idx_index.at(name);
  auto& node     = nodes[idx];
  auto tree_it   = std::find(trees.begin(), trees.end(), node.self_idx);
  if (tree_it != trees.end()) {
    trees.erase(tree_it);
  }

  // nodes.erase(node_it);
  if (node.parent >= 0) {
    auto& parent = nodes[node.parent];
    parent.remove_child(node);
    node.parent = -1;
  }

  return node;
}

Node& Graph::find(const Str& name) {
  const auto idx = idx_index.at(name);
  return nodes[idx];
}

void Graph::pose_objects(const Map<Str, Transform3r>& poses) {
  for (auto& tree_idx : trees) {
    auto& tree = nodes[tree_idx];
    if (tree.is_object) {
      tree.transform = poses.at(tree.name);
    }
  }
}

Map<Str, Node*> Graph::make_robot_nodes_map(const Map<Str, Str>& name_puns) {
  Map<Str, Node*> result;
  const auto robot_walker = [&](const auto& f, const auto node_idx) -> void {
    const auto& node = nodes[node_idx];
    result.emplace(node.name, &nodes[node_idx]);
    auto pun_it = name_puns.find(node.name);
    if (pun_it != name_puns.end()) {
      result.emplace(pun_it->second, &nodes[node_idx]);
    }

    for (const auto child_idx : node.children) {
      f(f, child_idx);
    }
  };

  // Find the base node
  int base_idx = -1;
  for (const auto tree_idx : trees) {
    if (nodes[tree_idx].is_base) {
      base_idx = tree_idx;
      break;
    }
  }

  robot_walker(robot_walker, base_idx);

  return result;
}
}  // namespace structures::scenegraph
