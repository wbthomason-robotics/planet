project('planet', 'cpp', default_options: ['cpp_std=c++17'], license: 'MIT', version: '0.1.0')

add_global_arguments('-DFMT_HEADER_ONLY', language: 'cpp')
add_global_arguments('-DSPDLOG_FMT_EXTERNAL', language: 'cpp')
add_global_arguments('-march=native', language: 'cpp')
add_global_arguments('-ffast-math', language: 'cpp')
# add_global_arguments('-ffp-contract=fast', language: 'cpp')
# add_global_arguments('-fno-PIC', language: 'cpp')

# Common dependencies
thread_dep = dependency('threads')
# log_dep = dependency('spdlog')
log_inc = include_directories('/usr/include/spdlog')
log_dep = declare_dependency(include_directories: log_inc)

boost_dep = dependency('boost', modules: ['system', 'filesystem'])

fp_inc = include_directories('lib/FunctionalPlus/include')
fp_dep = declare_dependency(include_directories: fp_inc)

hm_inc = include_directories('lib/robin-map/include', 'lib/hopscotch-map/include')
hm_dep = declare_dependency(include_directories: hm_inc)

# TOML
toml_inc = include_directories('lib/cpptoml/include')
toml_dep = declare_dependency(include_directories: toml_inc)

# Note that fmt *must* be compiled with CMAKE_POSITION_INDEPENDENT_CODE in order to link
fmt_dep = meson.get_compiler('cpp').find_library('fmt', required: true)

# Sexpresso: Note that you must modify the sexpresso build script to compile with -fPIC
sexp_inc = include_directories('lib/sexpresso/sexpresso')
sexp_lib = static_library('sexpresso', objects: 'lib/sexpresso/sexpresso.o')
sexp_dep = declare_dependency(link_with: sexp_lib, include_directories: sexp_inc)

# LuaJIT
lua_dep = dependency('luajit')

# Tinyobjloader
obj_dep = dependency('tinyobjloader')

# OMPL
ompl_dep = dependency('ompl')

# Bullet for collisions
bullet_dep = dependency('bullet')

# URDF
urdf_dep = dependency('urdfdom')

# TinyXML
tinyxml_dep = dependency('tinyxml2')

# Eigen
eigen_dep = dependency('eigen3')

# Liboptim - for gradient descent
optim_inc = include_directories('/usr/local/include/optim')
optim_lib = meson.get_compiler('cpp').find_library('optim',
  required: true,
  dirs: ['/usr/local/lib'])
optim_dep = declare_dependency(dependencies: [optim_lib], include_directories: optim_inc)

# Armadillo - for Liboptim and related functions
arma_dep = dependency('armadillo')

# nlohmann JSON - for output
json_dep = dependency('nlohmann_json')

# Shared aliases for easy container swapping
common_inc = include_directories('common')
common_dep = declare_dependency(include_directories: common_inc, dependencies: [hm_dep])

# General/shared data structures and representations
structures_inc = include_directories('structures')
_structures_deps = [common_dep, bullet_dep, eigen_dep, fmt_dep, fp_dep, log_dep]
structures_lib = static_library('structures',
  'structures/scenegraph.cc',
  dependencies: _structures_deps)
structures_dep = declare_dependency(link_with: structures_lib,
  include_directories: structures_inc,
  dependencies: _structures_deps)

# Problem input: Specifications, models, scene loading
input_inc = include_directories('input')
_input_deps = [common_dep,
  boost_dep,
  bullet_dep,
  eigen_dep,
  fmt_dep,
  fp_dep,
  log_dep,
  obj_dep,
  sexp_dep,
  structures_dep,
  tinyxml_dep,
  urdf_dep]
input_lib = static_library('input',
  'input/specification.cc',
  'input/scene.cc',
  dependencies: _input_deps)
input_dep = declare_dependency(link_with: input_lib,
  include_directories: input_inc,
  dependencies: _input_deps)

# Planner: Symbolic and continuous
planner_inc = include_directories('planner')
_planner_deps = [common_dep,
  arma_dep,
  bullet_dep,
  eigen_dep,
  fmt_dep,
  fp_dep,
  hm_dep,
  input_dep,
  json_dep,
  log_dep,
  lua_dep,
  ompl_dep,
  optim_dep,
  structures_dep,
  thread_dep,
  urdf_dep]
planner_lib = static_library('plannerlib',
  'planner/bulletCollision.cc',
  'planner/cspace.cc',
  'planner/goal.cc',
  'planner/hashable_statespace.cc',
  'planner/heuristic.cc',
  'planner/initial.cc',
  'planner/motion.cc',
  'planner/planner_utils.cc',
  'planner/predicate.cc',
  'planner/rrt.cc',
  'planner/sampler.cc',
  'planner/solver.cc',
  'planner/universe_map.cc',
  'planner/world_functions.cc',
  dependencies: _planner_deps)
planner_dep = declare_dependency(link_with: planner_lib,
  include_directories: planner_inc,
  dependencies: _planner_deps)

# Solution output
output_inc = include_directories('output')
_output_deps = [common_dep,
  fmt_dep,
  fp_dep,
  json_dep,
  log_dep,
  ompl_dep,
  planner_dep]
output_lib = static_library('output', 'output/output.cc', dependencies: _output_deps)
output_dep = declare_dependency(link_with: output_lib,
  include_directories: output_inc,
  dependencies: _output_deps)

# Main executable
executable('planet',
  'planet.cc',
  dependencies: [common_dep,
    fmt_dep,
    input_dep,
    log_dep,
    output_dep,
    planner_dep,
    toml_dep])
