#!/bin/sh


extra_args=""
needs_rebuild=false

# Handle options indicating various clang sanitizers
while getopts ":tmau" opt; do
  case ${opt} in
    # Thread sanitizer
    t ) extra_args="-Db_sanitize=address -Db_lundef=false"
    needs_rebuild=true
    ;;
    # Memory sanitizer
    m ) extra_args="-Db_sanitize=memory -Db_lundef=false"
    needs_rebuild=true
    ;;
    # Address sanitizer
    a ) extra_args="-Db_sanitize=address -Db_lundef=false"
    needs_rebuild=true
    ;;
    # Undefined behavior sanitizer
    u ) extra_args="-Db_sanitize=undefined -Db_lundef=false"
    needs_rebuild=true
    ;;
  \? ) echo "Unknown sanitizer option. Choices are [t]hread, [m]emory, [u]ndefined behavior, or [a]ddress"
    ;;
  esac
  shift $((OPTIND -1))
done

if [ "$1" = "tsan" ] || [ "$2" = "tsan" ] || [ "$3" = "tsan" ]; then
  extra_args="-Db_sanitize=address -Db_lundef=false"
fi

if [ "$1" = "release" ]; then
  extra_args="--buildtype=release"
  needs_rebuild=true
fi

if [ "$1" = "debugopt" ]; then
  extra_args="--buildtype=debugoptimized"
  needs_rebuild=true
fi

if [ "$1" = "rebuild" ] || [ "$needs_rebuild" = true ]; then
  rm -rf build/
fi

if [ ! -d build ]; then
  PKG_CONFIG_PATH=pkgconf:$PKG_CONFIG_PATH CXX=clang++ meson build $extra_args
fi

# Check again; meson may have failed
if [ -d build ]; then
  cd build && ninja
  cd ..
fi
