# Planet: A Unified Sampling-Based Approach to Integrated Task and Motion Planning

This is a proof-of-concept implementation of the ISRR 2019 paper [A Unified Sampling-Based Approach
to Integrated Task and Motion
Planning](https://wbthomason.github.io/papers/isrr2019_unifiedtamp.pdf).

## Fetching and Building

This code is designed to be easy (relative to other research code) to install and use. To get it
running, you have two options:

### Docker

You can use the `Dockerfile` in this repo to get a Docker image with Planet and its dependencies
installed.

### Native

To build Planet natively, follow these steps:

1. Install the following dependencies:
  - [`meson`](https://mesonbuild.com/)
  - [`luajit`](https://luajit.org/)
  - A reasonable C++17 compiler (this was built using Clang++ 10.0.0)
  - [`spdlog`](https://github.com/gabime/spdlog)
  - [`cxxopts`](https://github.com/jarro2783/cxxopts)
  - [`boost`](https://www.boost.org/)
  - [`FunctionalPlus`](https://github.com/Dobiasd/FunctionalPlus)
  - [`cpptoml`](https://github.com/skystrife/cpptoml)
  - [`fmt`](https://github.com/fmtlib/fmt)
  - [`sexpresso`](https://github.com/BitPuffin/sexpresso)
  - [`tinyobjloader`](https://github.com/syoyo/tinyobjloader)
  - [`ompl`](https://ompl.kavrakilab.org/)
  - [`bullet`](https://github.com/bulletphysics/bullet3)
  - [`nlopt`](https://github.com/stevengj/nlopt)
  - [`urdf`](https://github.com/ros/urdfdom)
  - [`tinyxml2`](https://github.com/leethomason/tinyxml2)
  - [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - [`nlohmann_json`](https://github.com/nlohmann/json)
2. Run `git clone https://github.com/wbthomason-robotics/planet && cd planet && git submodule update --init --recursive` to get this code and its dependent submodules.
3. Run `./build.sh {debug, debugopt, release}` to build the various configurations of the `planet`
   binary (look at `build.sh` for some other options mostly useful for development). If you have the
   dependencies above installed, this should "just work".

## Usage

You'll need to fetch the benchmark files from
<http://tampbenchmark.aass.oru.se/index.php?title=Main_Page>. Take a look at the `lua` directory for
some example predicate semantics implementations, and replace the PDDL files with those in the
`test_scenarios` directory.

In general: run `./planet <YOUR PROBLEM CONFIG.toml>`. Look at the `.toml` files in this repo for
examples.

You can use `./planet --help` to see flags to control execution. In particular, `-g`/`--greedy` uses
best-first action selection (rather than prioritized sampling), which can lead to speedups for some
problems.

### Helpful tips for running on your own problems

- You must annotate discrete predicates as `:discrete`, and kinematics-altering predicates (see paper for explanation) as `:kinematic`.
- We use LuaJit 2.0.5, which is compatible with Lua 5.1
- Your parameters to actions and predicates (and, correspondingly, their usage in preconditions and effects for actions) *must* be valid Lua variable names.
- Continuous (i.e. non-discrete & non-kinematic) predicates are assumed to have a Lua function definition with the same name as the predicate in your domain semantics file.
- Continuous predicate definitions can only (as of now) use the operator functions `eq`, `And`, `Or`,
  `le`, `ge`, `lt`, `gt`, and basic math operators (note that if a math operator has an overloaded operator
  function you need to use the overloaded version)

## Status

Planet is usable as a proof of concept. No claims are made about the quality of engineering,
performance, etc. Use at your own risk.

## TODO
- [ ] Remove need for `sci-lua`/generally simplify Lua dependencies.
- [ ] Implement syntactic transformation for handling negation
- [ ] Profiling/rewrites for performance improvements
