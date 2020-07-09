project('planet', 'cpp',
  default_options: ['cpp_std=c++17', 'warning_level=0'],
  license: 'MIT',
  version: '0.2.0')

if get_option('buildtype') == 'debug'
  add_project_arguments('-DIS_DEBUG', language: 'cpp')
else
  add_project_arguments('-march=native', language: 'cpp')
endif

if get_option('log_actions').enabled()
  add_project_arguments('-DLOG_ACTIONS', language: 'cpp')
endif

add_project_arguments('-DFMT_HEADER_ONLY', '-DSPDLOG_FMT_EXTERNAL', language: 'cpp')

if get_option('profile')
  add_project_arguments('-pg', language: 'cpp')
endif

# Common dependencies
boost = dependency('boost', modules: ['system', 'filesystem'])
bullet = dependency('bullet')
dbg = declare_dependency(include_directories: 'lib/dbg-macro')
eigen = dependency('eigen3')
expected = declare_dependency(include_directories: 'lib/expected/include')
fmt = dependency('fmt')
fp = declare_dependency(include_directories: 'lib/FunctionalPlus/include')
hm = declare_dependency(include_directories: ['lib/robin-map/include', 'lib/hopscotch-map/include'])
json = dependency('nlohmann_json')
log = dependency('spdlog')
lua = dependency('luajit')
nlopt = dependency('nlopt')
obj = declare_dependency(include_directories: 'lib/tinyobjloader')
# Nasty hack workaround to prevent OMPL from forcing C++14
ompl = dependency('ompl')
ompl = declare_dependency(
  dependencies: ompl.partial_dependency(
    compile_args : false, link_args : true, links : true, includes : true, sources : true),
  include_directories: ompl.get_pkgconfig_variable('includedir')
  )
patterns = declare_dependency(include_directories: 'lib/patterns/include')

sexp_lib = static_library('sexpresso', 'lib/sexpresso/sexpresso/sexpresso.cpp', include_directories: 'lib/sexpresso/sexpresso')
sexp = declare_dependency(link_with: sexp_lib, include_directories: 'lib/sexpresso/sexpresso')

threads = dependency('threads')
tinyxml = dependency('tinyxml2')
toml = declare_dependency(include_directories: 'lib/cpptoml/include')
urdf = dependency('urdfdom')

# Shared aliases for easy container swapping
common = declare_dependency(include_directories: 'common', dependencies: [bullet, eigen, expected, hm, dbg, patterns])

# General/shared data structures and representations
structures_deps = [common, fmt, fp, log]
structures_lib = static_library('structures', 'structures/scenegraph.cc', dependencies: structures_deps)

structures = declare_dependency(link_with: structures_lib, include_directories: 'structures', dependencies: structures_deps)

# Problem input: Specifications, models, scene loading
input_deps = [common, boost, obj, sexp, structures, tinyxml, urdf]

input_lib = static_library('input',
  'input/specification.cc',
  'input/scene.cc',
  include_directories: 'input',
  dependencies: input_deps)

input = declare_dependency(link_with: input_lib, include_directories: 'input', dependencies: input_deps)

# Planner: Symbolic and continuous
planner_deps = [common, input, json, lua, ompl, nlopt, structures, threads, urdf]

planner_lib = static_library('plannerlib',
  'planner/bulletCollision.cc',
  'planner/cspace.cc',
  'planner/goal.cc',
  'planner/hashable_statespace.cc',
  'planner/heuristic.cc',
  'planner/initial.cc',
  'planner/motion.cc',
  'planner/planner_utils.cc',
  'planner/predicate-impl.cc',
  'planner/predicate.cc',
  'planner/rrt.cc',
  'planner/sampler.cc',
  'planner/solver.cc',
  'planner/universe_map.cc',
  'planner/utils.cc',
  include_directories: 'planner',
  dependencies: planner_deps)

planner = declare_dependency(link_with: planner_lib, include_directories: 'planner', dependencies: planner_deps)

# Solution output
output_deps = [common, planner]

output_lib = static_library('output', 'output/output.cc', include_directories: 'output', dependencies: output_deps)

output = declare_dependency(link_with: output_lib, include_directories: 'output', dependencies: output_deps)

# Main executable
executable('planet', 'planet.cc', dependencies: [common, input, output, planner, toml])
