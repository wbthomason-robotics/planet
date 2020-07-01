#define TINYOBJLOADER_IMPLEMENTATION
#include <fmt/format.h>
#include <fmt/ostream.h>

#include <array>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <unordered_map>
#include <utility>

#include "scene.hh"

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include <boost/filesystem.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fplus/fplus.hpp"

#include "tinyxml2.h"

#include "tiny_obj_loader.h"

#include <urdf_model/model.h>
#include <urdf_model/joint.h>
#include <urdf_parser/urdf_parser.h>

namespace input::scene {
// TODO(Wil): Should pose/surface/grasp info be bundled into Objects or contained separately for
// faster searches?
namespace {
  auto log = spdlog::stdout_color_st("scene");
  // NOTE: Taken from ImportURDFDemo in Bullet repo
  constexpr float DEFAULT_COLLISION_MARGIN = 0.001;

  template <int dim> auto parse_template(const Vec<Str>& elems);
  template <> auto parse_template<3>(const Vec<Str>& elems) {
    Eigen::Matrix<double, 3, 3> transform_mat(Eigen::Matrix<double, 3, 3>::Identity());
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        transform_mat(i, j) = std::stod(elems[3 * i + j]);
      }
    }

    return transform_mat;
  }

  template <> auto parse_template<4>(const Vec<Str>& elems) {
    Eigen::Matrix<double, 4, 4> transform_mat(Eigen::Matrix<double, 4, 4>::Identity());
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        transform_mat(i, j) = std::stod(elems[4 * i + j]);
      }
    }

    return transform_mat;
  }

  auto transform_of_pose(const urdf::Pose& pose) {
    Transform3r result;
    result.translation() = Vector3r(pose.position.x, pose.position.y, pose.position.z);
    result.linear() =
    Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z)
    .toRotationMatrix();
    return result;
  }

  auto parse_point(const tinyxml2::XMLNode* point_node) {
    auto x = point_node->FirstChildElement("x")->Value();
    auto y = point_node->FirstChildElement("y")->Value();
    auto z = point_node->FirstChildElement("z")->Value();
    return Vector3r(std::stod(x), std::stod(y), std::stod(z));
  }

  std::unique_ptr<btCollisionShape> load_bullet_mesh(const Vec<tinyobj::shape_t>& shapes,
                                                     const tinyobj::attrib_t& mesh_attrib,
                                                     const urdf::Vector3& scale,
                                                     const bool maybe_concave) {
    btVector3 geom_scale(scale.x, scale.y, scale.z);
    std::unique_ptr<btCollisionShape> model;
    if (maybe_concave) {
      // NOTE: We assume that there is only ever one shape in an obj file for a concave shape
      const auto& shape   = shapes[0];
      size_t index_offset = 0;
      auto* trimesh       = new btTriangleMesh();
      std::array<btVector3, 3> verts;
      for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
        for (size_t v = 0; v < 3; ++v) {
          auto idx = shape.mesh.indices[index_offset + v];
          auto vx  = mesh_attrib.vertices[3 * idx.vertex_index + 0];
          auto vy  = mesh_attrib.vertices[3 * idx.vertex_index + 1];
          auto vz  = mesh_attrib.vertices[3 * idx.vertex_index + 2];
          btVector3 point(vx, vy, vz);
          verts[v] = point * geom_scale;
        }

        trimesh->addTriangle(verts[0], verts[1], verts[2]);
        index_offset += 3;
      }

      model = std::make_unique<btBvhTriangleMeshShape>(trimesh, true, true);
      model->setMargin(DEFAULT_COLLISION_MARGIN);
    } else {
      // If we know something isn't possibly concave, it's because it's a convex decomposition
      btTransform identity;
      identity.setIdentity();
      auto compound_model = std::make_unique<btCompoundShape>();
      for (const auto& shape : shapes) {
        auto hull_model = new btConvexHullShape();
        for (const auto& idx : shape.mesh.indices) {
          auto vx = mesh_attrib.vertices[3 * idx.vertex_index + 0];
          auto vy = mesh_attrib.vertices[3 * idx.vertex_index + 1];
          auto vz = mesh_attrib.vertices[3 * idx.vertex_index + 2];
          btVector3 point(vx, vy, vz);
          hull_model->addPoint(point * geom_scale, false);
        }

        hull_model->setMargin(DEFAULT_COLLISION_MARGIN);
        hull_model->recalcLocalAabb();
        hull_model->optimizeConvexHull();
        compound_model->addChildShape(identity, hull_model);
      }

      compound_model->setMargin(DEFAULT_COLLISION_MARGIN);
      compound_model->recalculateLocalAabb();
      model = std::move(compound_model);
    }

    return model;
  }

  std::unique_ptr<CollisionShape> parse_obj_mesh(const Str& obj_mesh_path,
                                                 const Str& obj_dir,
                                                 const urdf::Vector3& scale,
                                                 const bool maybe_concave) {
    tinyobj::attrib_t mesh_attrib;
    Vec<tinyobj::shape_t> shapes;
    Vec<tinyobj::material_t> materials;
    Str err;
    Str warn;
    if (!tinyobj::LoadObj(
        &mesh_attrib, &shapes, &materials, &warn, &err, obj_mesh_path.c_str(), obj_dir.c_str())) {
      log->error("Error loading mesh from {}: '{}'", obj_mesh_path, err);
      return nullptr;
    }

    return load_bullet_mesh(shapes, mesh_attrib, scale, maybe_concave);
  }

  std::unique_ptr<CollisionShape> geom_of_collision(const urdf::CollisionSharedPtr& collision,
                                                    const boost::filesystem::path& model_dir) {
    std::unique_ptr<CollisionShape> geom;
    switch (collision->geometry->type) {
      case urdf::Geometry::MESH: {
        auto collision_geom          = dynamic_cast<const urdf::Mesh&>(*collision->geometry);
        auto collision_geom_filename = collision_geom.filename;
        log->debug("Loading from mesh: {}", collision_geom_filename);
        char const* ros_package_path = std::getenv("ROS_PACKAGE_PATH");
        if (ros_package_path == nullptr) {
          log->warn("Environment variable ROS_PACKAGE_PATH is undefined! Assuming default value "
                    "(/opt/ros/melodic/share)");
          ros_package_path = "/opt/ros/melodic/share";
        }

        // Resolve the path to mesh files
        const std::regex package_re("^package:/");
        collision_geom_filename =
        std::regex_replace(collision_geom_filename, package_re, ros_package_path);
        auto collision_geom_path = boost::filesystem::path(collision_geom_filename);
        bool is_obj              = collision_geom_path.extension() == ".obj";
        if (collision_geom_path.is_relative()) {
          collision_geom_path = model_dir / collision_geom_path;
        }

        if (!is_obj) {
          collision_geom_path.replace_extension(".obj");
          collision_geom_path =
          boost::filesystem::temp_directory_path() / collision_geom_path.filename();
        }

        // TODO(Wil): Find a better way of doing this. Right now (a) using std::system is bad and
        // (b) this assumes that the right script is at the relative path scripts/mesh_to_obj,
        // which is also bad.
        if (!is_obj && !boost::filesystem::is_regular_file(collision_geom_path)) {
          log->debug("Building .obj file for {}", collision_geom_filename);
          std::system(fmt::format("blender -b -P scripts/mesh_to_obj -- {} -o {}",
                                  collision_geom_filename,
                                  collision_geom_path.string())
                      .c_str());
        } else {
          // TODO(Wil): This could be a problem if there are multiple distinct meshes with the
          // same name.
          log->debug("Using cached .obj file for {}", collision_geom_filename);
        }
        // Assume success...
        geom = parse_obj_mesh(collision_geom_path.string(), "", collision_geom.scale, false);
      } break;

      case urdf::Geometry::BOX: {
        const auto& collision_geom = dynamic_cast<const urdf::Box&>(*collision->geometry);
        // Bullet expects half-extents here
        const auto x = collision_geom.dim.x / 2.0;
        const auto y = collision_geom.dim.y / 2.0;
        const auto z = collision_geom.dim.z / 2.0;
        geom         = std::make_unique<btBoxShape>(btVector3(x, y, z));
      } break;

      case urdf::Geometry::CYLINDER: {
        const auto& collision_geom = dynamic_cast<const urdf::Cylinder&>(*collision->geometry);
        // URDF cylinders are Z-aligned, and Bullet expects half-extents here
        geom = std::make_unique<btCylinderShapeZ>(
        btVector3(collision_geom.radius, collision_geom.radius, collision_geom.length / 2.0));
      } break;

      case urdf::Geometry::SPHERE: {
        const auto& collision_geom = dynamic_cast<const urdf::Sphere&>(*collision->geometry);
        geom                       = std::make_unique<btSphereShape>(collision_geom.radius);
      } break;

      default:
        log->error("Got a mesh type I don't know how to deal with! Got: {}",
                   collision->geometry->type);
        throw std::runtime_error("Unknown mesh type");
    }

    geom->setMargin(DEFAULT_COLLISION_MARGIN);
    return geom;
  }


  Map<Str, Str> scenegraph_from_model_helper(const urdf::LinkSharedPtr& node,
                                             const boost::filesystem::path& model_dir,
                                             std::shared_ptr<Graph>& sg,
                                             int parent_idx) {
    // Handle this node
    std::unique_ptr<btCollisionShape> collision = nullptr;
    urdf::Pose coll_pose;
    // NOTE: We assume there's at most one collision/node, which might be wrong for some URDF
    // models
    if (node->collision_array.size() > 1) {
      log->warn("More than one collision for {}", node->name);
    }

    Transform3r coll_tf;
    if (node->collision != nullptr) {
      collision = geom_of_collision(node->collision, model_dir);
      coll_pose = node->collision->origin;
      coll_tf   = transform_of_pose(coll_pose);
    } else {
      coll_tf = Transform3r::Identity();
    }

    auto pose = node->parent_joint->parent_to_joint_origin_transform;
    auto tf   = transform_of_pose(pose);
    Vector3r axis;
    auto node_axis = node->parent_joint->axis;
    Node::Type type;
    switch (node->parent_joint->type) {
      case urdf::Joint::PRISMATIC:
        axis = Vector3r(node_axis.x, node_axis.y, node_axis.z);
        type = Node::Type::PRISMATIC;
        break;

      case urdf::Joint::REVOLUTE:
        axis = Vector3r(node_axis.x, node_axis.y, node_axis.z);
        type = Node::Type::REVOLUTE;
        break;

      case urdf::Joint::FIXED:
        type = Node::Type::FIXED;
        axis = Vector3r::Zero();
        break;

      case urdf::Joint::CONTINUOUS:
        type = Node::Type::CONTINUOUS;
        axis = Vector3r(node_axis.x, node_axis.y, node_axis.z);
        break;

      default:
        log->warn("Unsupported joint type '{}'! Treating as fixed.", node->parent_joint->type);
        type = Node::Type::FIXED;
        axis = Vector3r::Zero();
    }

    Node graph_node(node->name, type, tf, coll_tf, std::move(axis), std::move(collision));
    auto& new_node     = sg->add_node(parent_idx, graph_node);
    const auto new_idx = new_node.self_idx;

    // Then the children, recursively
    Map<Str, Str> name_puns;
    for (const auto& child : node->child_links) {
      auto child_name_puns = scenegraph_from_model_helper(child, model_dir, sg, new_idx);
      name_puns.emplace(child->name, child->parent_joint->name);
      name_puns.merge(child_name_puns);
      // tree_nodes.insert({child->parent_joint->name, child_tree.get()});
      // tree_nodes.insert({child->name, child_tree.get()});
      // tree_nodes.merge(child_tree_nodes);
    }

    return name_puns;
  }

  /// Build a scenegraph from a URDF model
  Map<Str, Str> scenegraph_from_model(const std::shared_ptr<urdf::ModelInterface>& model,
                                      const boost::filesystem::path& model_dir,
                                      std::shared_ptr<Graph>& sg) {
    // First, extract the root
    auto model_root       = model->getRoot();
    auto root_collision   = geom_of_collision(model_root->collision, model_dir);
    auto root_tf          = Transform3r::Identity();
    const auto& coll_pose = model_root->collision->origin;
    const auto coll_tf    = transform_of_pose(coll_pose);
    Node root(model_root->name,
              Node::Type::FIXED,
              root_tf,
              coll_tf,
              Vector3r::Zero(),
              std::move(root_collision));

    // Then, recursively handle the children
    auto& new_root      = sg->add_node(-1, root);
    new_root.is_base    = true;
    const auto root_idx = new_root.self_idx;
    Map<Str, Str> name_puns;
    for (const auto& child : model_root->child_links) {
      auto child_name_puns = scenegraph_from_model_helper(child, model_dir, sg, root_idx);
      name_puns.emplace(child->name, child->parent_joint->name);
      name_puns.merge(child_name_puns);
      // tree_nodes.emplace(child->parent_joint->name, child_tree.get());
      // tree_nodes.emplace(child->name, child_tree.get());
      // tree_nodes.merge(child_tree_nodes);
      // sg->add_node(root.get(), std::move(child_tree));
    }

    return name_puns;
  }

  std::pair<Object, Transform3r> parse_object(const Str& obj_dir, tinyxml2::XMLNode* object) {
    auto name = object->FirstChildElement("name")->FirstChild()->Value();
    // Check movability
    auto movable =
    strncmp(object->FirstChildElement("moveable")->FirstChild()->Value(), "true", 4) == 0;
    Object result(name, movable);

    // Load mesh
    auto geom_path =
    fmt::format("{}/{}", obj_dir, object->FirstChildElement("geom")->FirstChild()->Value());

    // Load transform matrix
    auto pose_elems = fplus::split_by_token<Str>(
    " ", false, object->FirstChildElement("pose")->FirstChild()->Value());
    auto transform_mat = parse_template<4>(pose_elems);

    // Get SSSP (only one is currently allowed)
    auto sssp_node = object->FirstChildElement("sssp");
    auto pssp_node = object->FirstChildElement("pssp");
    if (sssp_node != nullptr) {
      auto xmin = std::stod(sssp_node->FirstChildElement("xmin")->FirstChild()->Value());
      auto xmax = std::stod(sssp_node->FirstChildElement("xmax")->FirstChild()->Value());

      auto ymin = std::stod(sssp_node->FirstChildElement("ymin")->FirstChild()->Value());
      auto ymax = std::stod(sssp_node->FirstChildElement("ymax")->FirstChild()->Value());

      auto zmin = std::stod(sssp_node->FirstChildElement("zmin")->FirstChild()->Value());
      auto zmax = std::stod(sssp_node->FirstChildElement("zmax")->FirstChild()->Value());

      result.stable_face.emplace<structures::object::StableRegion>(
      structures::object::StableRegion(xmin, xmax, ymin, ymax, zmin, zmax));
    } else if (pssp_node != nullptr) {
      auto point_node = pssp_node->FirstChildElement("point");
      structures::object::StablePlane plane;
      while (point_node != nullptr) {
        plane.emplace_back(parse_point(point_node));
        point_node = point_node->NextSiblingElement("point");
      }

      result.stable_face.emplace<structures::object::StablePlane>(
      structures::object::StablePlane(plane));
    } else {
      log->debug("No SSSP for {}", name);
    }

    // Get SOPs
    auto sop_node = object->FirstChildElement("sop");
    while (sop_node != nullptr) {
      auto template_elems = fplus::split_by_token<Str>(
      " ", false, sop_node->FirstChildElement("template")->FirstChild()->Value());
      Eigen::Quaterniond template_rot(parse_template<3>(template_elems));
      auto axis_elems = fplus::split_by_token<Str>(
      " ", false, sop_node->FirstChildElement("axis")->FirstChild()->Value());
      auto axis =
      Vector3r(std::stod(axis_elems[0]), std::stod(axis_elems[1]), std::stod(axis_elems[2]));
      auto distance = std::stod(sop_node->FirstChildElement("distance")->FirstChild()->Value());
      result.stable_poses.emplace_back(template_rot, axis, distance);
      sop_node = sop_node->NextSiblingElement("sop");
    }

    // Get grasps - First discrete, then continuous
    auto grasp_node = object->FirstChildElement("gf");
    while (grasp_node != nullptr) {
      auto template_elems =
      fplus::split_by_token<Str>(" ", false, grasp_node->FirstChild()->Value());
      auto frame = parse_template<4>(template_elems);
      result.grasps.emplace_back(Transform3r(frame));
      grasp_node = grasp_node->NextSiblingElement("gf");
    }

    grasp_node = object->FirstChildElement("gc");
    while (grasp_node != nullptr) {
      auto template_elems = fplus::split_by_token<Str>(
      " ", false, grasp_node->FirstChildElement("template")->FirstChild()->Value());
      auto frame      = parse_template<4>(template_elems);
      auto axis_elems = fplus::split_by_token<Str>(
      " ", false, grasp_node->FirstChildElement("axis")->FirstChild()->Value());
      auto axis =
      Vector3r(std::stod(axis_elems[0]), std::stod(axis_elems[1]), std::stod(axis_elems[2]));
      result.grasps.emplace_back(std::in_place_type_t<structures::object::ContinuousGrasp>(),
                                 Transform3r(frame),
                                 axis);
      grasp_node = grasp_node->NextSiblingElement("gc");
    }

    result.geom = parse_obj_mesh(geom_path, obj_dir, urdf::Vector3(1.0, 1.0, 1.0), true);
    const Transform3r eigen_transform(transform_mat);
    btTransform pose_transform;
    auto& pose_origin            = pose_transform.getOrigin();
    const auto& pose_translation = eigen_transform.translation();
    pose_origin.setX(pose_translation.x());
    pose_origin.setY(pose_translation.y());
    pose_origin.setZ(pose_translation.z());
    const Eigen::Quaterniond eigen_rotation(eigen_transform.linear());
    btQuaternion pose_rotation(
    eigen_rotation.x(), eigen_rotation.y(), eigen_rotation.z(), eigen_rotation.w());
    pose_transform.setRotation(pose_rotation);
    result.initial_pose = pose_transform;
    return std::make_pair(std::move(result), eigen_transform);
  }

  std::optional<Robot> parse_robot(tinyxml2::XMLNode* robot_node, std::shared_ptr<Graph>& sg) {
    auto name = robot_node->FirstChildElement("name")->FirstChild()->Value();

    // Load kinematic tree
    auto urdf_path = robot_node->FirstChildElement("urdf")->FirstChild()->Value();
    auto model     = urdf::parseURDFFile(urdf_path);
    if (model == nullptr) {
      log->error("Failed to load URDF for {}!", name);
      return std::nullopt;
    }

    // We need the path for handling relative paths to meshes in the URDF
    auto model_dir = boost::filesystem::path(urdf_path).parent_path();
    auto name_puns = scenegraph_from_model(model, model_dir, sg);
    // Load base pose transform matrix
    auto pose_elems = fplus::split_by_token<Str>(
    " ", false, robot_node->FirstChildElement("basepose")->FirstChild()->Value());
    auto transform_mat = parse_template<4>(pose_elems);

    auto tf = std::make_unique<Transform3r>(transform_mat);

    // Load torso configuration
    auto torso_node = robot_node->FirstChildElement("torso");
    auto torso_cfg  = (torso_node != nullptr) ?
                     std::optional<double>(std::stod(torso_node->FirstChild()->Value())) :
                     std::nullopt;

    // Setup info for controllable joints

    auto controllable_joints_element    = robot_node->FirstChildElement("controllable_joints");
    tinyxml2::XMLElement* joint_element = nullptr;
    if (controllable_joints_element == nullptr) {
      log->critical("No controllable joints found! Good luck solving manipulation problems...");
    } else {
      joint_element = controllable_joints_element->FirstChildElement("joint");
    }

    std::map<Str, JointData> controllable_joints;
    auto unbounded_limits   = std::make_shared<urdf::JointLimits>();
    unbounded_limits->lower = -std::numeric_limits<double>::infinity();
    unbounded_limits->upper = std::numeric_limits<double>::infinity();
    while (joint_element != nullptr) {
      auto joint_name       = joint_element->FindAttribute("name")->Value();
      auto joint_init_value = std::stod(joint_element->FirstChild()->Value());
      auto& joint_bounds    = model->joints_[joint_name]->type == urdf::Joint::CONTINUOUS ?
                           unbounded_limits :
                           model->joints_[joint_name]->limits;

      controllable_joints.emplace(
      joint_name,
      JointData{joint_name, joint_bounds->lower, joint_bounds->upper, joint_init_value});

      joint_element = joint_element->NextSiblingElement();
    }

    // Now that everything is added into the scenegraph, we can make the tree_nodes map for the
    // robot
    const auto tree_nodes = sg->make_robot_nodes_map(name_puns);

    // Check base movability
    auto base_movable =
    strncmp(robot_node->FirstChildElement("movebase")->FirstChild()->Value(), "true", 4) == 0;
    return std::make_optional<Robot>(
    tree_nodes, controllable_joints, base_movable, std::move(tf), torso_cfg);
  }
}  // namespace

std::optional<
std::tuple<ObjectSet, ObjectSet, Robot, std::shared_ptr<Graph>, std::optional<std::array<Bounds, 3>>>>
load(const Str& scene_path, const Str& obj_dir) {
  log->info("Loading scene from: {}", scene_path);
  tinyxml2::XMLDocument scene_doc;
  if (scene_doc.LoadFile(scene_path.c_str()) != tinyxml2::XML_SUCCESS) {
    log->error("Failed to load scene! XML parsing error: {}", scene_doc.ErrorStr());
    return std::nullopt;
  }

  log->debug("Loaded scene XML! Starting parsing...");

  auto problem     = scene_doc.RootElement();
  auto object_node = problem->FirstChildElement("objects")->FirstChild();
  ObjectSet objects;
  ObjectSet obstacles;
  auto scenegraph = std::make_shared<Graph>();
  if (object_node != nullptr) {
    while (object_node != nullptr) {
      auto [obj, eigen_pose] = parse_object(obj_dir, object_node);
      // To avoid use-after-move
      auto name = obj.name;
      Node node(
      name, Node::Type::FIXED, eigen_pose, Transform3r::Identity(), Vector3r::Zero(), obj.geom);
      if (obj.movable) {
        objects.emplace(name, std::make_shared<Object>(std::move(obj)));
        node.is_object = true;
      } else {
        obstacles.emplace(name, std::make_shared<Object>(std::move(obj)));
        node.is_obstacle = true;
      }

      scenegraph->add_node(-1, node);
      object_node = object_node->NextSiblingElement("obj");
    }
  } else {
    log->warn("No objects!");
  }

  log->debug("Loaded {} objects", objects.size());

  // Note: We assume there is only ever a single robot, for now
  auto robot_node = problem->FirstChildElement("robots")->FirstChild();
  if (robot_node == nullptr) {
    log->error("No robot found!");
    return std::nullopt;
  }

  auto robot_result = parse_robot(robot_node, scenegraph);
  if (!robot_result) {
    log->error("Failed to parse robot config!");
    return std::nullopt;
  }

  // scenegraph->add_node(nullptr, std::move(robot_result->second));

  auto* bounds_node = problem->FirstChildElement("workspace_bounds");
  std::optional<std::array<Bounds, 3>> workspace_bounds = std::nullopt;
  if (bounds_node != nullptr) {
    auto* x_bounds_node = bounds_node->FirstChildElement("x");
    auto* y_bounds_node = bounds_node->FirstChildElement("y");
    auto* z_bounds_node = bounds_node->FirstChildElement("z");
    double x_low, x_high, y_low, y_high, z_low, z_high;
    x_bounds_node->FirstChildElement("low")->QueryDoubleText(&x_low);
    x_bounds_node->FirstChildElement("high")->QueryDoubleText(&x_high);
    y_bounds_node->FirstChildElement("low")->QueryDoubleText(&y_low);
    y_bounds_node->FirstChildElement("high")->QueryDoubleText(&y_high);
    z_bounds_node->FirstChildElement("low")->QueryDoubleText(&z_low);
    z_bounds_node->FirstChildElement("high")->QueryDoubleText(&z_high);

    workspace_bounds.emplace(
    std::array<Bounds, 3>{Bounds(x_low, x_high), Bounds(y_low, y_high), Bounds(z_low, z_high)});
  }

  return std::make_optional(std::make_tuple(std::move(objects),
                                            std::move(obstacles),
                                            std::move(*robot_result),
                                            std::move(scenegraph),
                                            std::move(workspace_bounds)));
}
}  // namespace input::scene
