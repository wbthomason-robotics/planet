#!/bin/sh

extra_args=""
needs_rebuild=false

# Handle options indicating various clang sanitizers
while getopts ":tmau" opt; do
  case ${opt} in
  # Thread sanitizer
  t)
    extra_args="-Db_sanitize=address -Db_lundef=false"
    needs_rebuild=true
    ;;
  # Memory sanitizer
  m)
    extra_args="-Db_sanitize=memory -Db_lundef=false"
    needs_rebuild=true
    ;;
  # Address sanitizer
  a)
    extra_args="-Db_sanitize=address -Db_lundef=false"
    needs_rebuild=true
    ;;
  # Undefined behavior sanitizer
  u)
    extra_args="-Db_sanitize=undefined -Db_lundef=false"
    needs_rebuild=true
    ;;
  \?)
    echo "Unknown sanitizer option. Choices are [t]hread, [m]emory, [u]ndefined behavior, or [a]ddress"
    ;;
  esac
  shift $((OPTIND - 1))
done

if [ "$1" = "tsan" ] || [ "$2" = "tsan" ] || [ "$3" = "tsan" ]; then
  extra_args="$extra_args -Db_sanitize=address -Db_lundef=false"
fi

if [ "$1" = "release" ]; then
  extra_args="$extra_args --buildtype=release"
  needs_rebuild=true
fi

if [ "$1" = "debugopt" ]; then
  extra_args="$extra_args --buildtype=debugoptimized"
  needs_rebuild=true
fi

if [ "$1" = "profile" ] || [ "$2" = "profile" ]; then
  extra_args="$extra_args -Dprofile=true"
  needs_rebuild=true
fi

if [ "$1" = "cache_profile" ] || [ "$2" = "cache_profile" ]; then
  extra_args="--optimization=3 --debug"
  needs_rebuild=true
fi

if [ "$1" = "gravity" ] || [ "$2" = "gravity" ]; then
  extra_args="$extra_args -Dlog_actions=enabled"
  needs_rebuild=true
fi

if [ "$1" = "debug" ] || [ "$1" = "rebuild" ] || [ "$needs_rebuild" = true ]; then
  rm -rf build/
fi

if [ ! -d build ]; then
  CXX=clang++ meson build $extra_args
  # Fix current Meson bug: https://github.com/mesonbuild/meson/issues/5597
  sed -i 's/-Xclang//g' build/compile_commands.json
  success=$?
else
  success=0
fi

# Check again; meson may have failed
if [ $success -eq 0 ] && [ -d build ]; then
  cd build && NINJA_STATUS="[%f/%t %es] " ninja
  cd ..
fi
