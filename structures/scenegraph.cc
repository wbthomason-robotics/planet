#include <algorithm>
#include <stdexcept>

#include "scenegraph.hh"

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

Node& Graph::get_node_at_idx(int idx) { return nodes[idx]; }

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
