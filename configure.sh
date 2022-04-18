#!/bin/bash
set -e

rebuild=false
while [[ $# -gt 0 ]]
do
    case $1 in
        -h|--help)
            echo "Usage: configure.sh [OPTIONS]"
            echo "Options:"
            echo "  -h, --help                 Print this help message"
            echo "  -r, --rebuild              Force rebuild by deleting the build directory"
            exit 0
            ;;
        -r|--re-build)
            rebuild=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

build_type=Debug
build_type=Release
build_type=RelWithDebInfo

_cmake=/usr/bin/cmake
if [ ! -x "$_cmake" ]; then
    _cmake=/usr/local/bin/cmake
fi
_cc=/bin/gcc
if [ ! -x "$_cc" ]; then
    _cc=/usr/bin/clang
fi
cxx=/bin/g++
if [ ! -x "$cxx" ]; then
    cxx=/usr/bin/clang++
fi
# toolchain=$HOME/application-installs/vcpkg/scripts/buildsystems/vcpkg.cmake

proj_dir=$(pwd)
build_dir=$proj_dir/build

if [ "$rebuild" = true ]; then
    rm -rf $build_dir
fi

$_cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=$build_type -DCMAKE_C_COMPILER:FILEPATH=$_cc -DCMAKE_CXX_COMPILER:FILEPATH=$cxx -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$toolchain "-S$proj_dir" "-B$build_dir" -G "Unix Makefiles"
