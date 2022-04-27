set -e

rebuild=false
target=all
while [[ $# -gt 0 ]]
do
    case $1 in
        -h|--help)
            echo "Usage: configure.sh [OPTIONS]"
            echo "Options:"
            echo "  -h, --help                      Print this help message"
            echo "  -r, --rebuild                   Force rebuild by deleting the build directory"
            echo "  -t, --target [build target]     Build the specified target"
            exit 0
            ;;
        -r|--re-build)
            rebuild=true
            shift
            ;;
        -t|--target)
            target=$2
            shift
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

_cmake=/usr/bin/cmake
if [ ! -x "$_cmake" ]; then
    _cmake=/usr/local/bin/cmake
fi

proj_dir=$(pwd)
build_dir=$proj_dir/build

# if [ ! -d "$build_dir" ]; then
#    echo "'$build_dir' not found! Configuring CMake..."
#    ./configure.sh
# else
#    if [ "$rebuild" = true ]; then
#       echo "Deleting '$build_dir'..."
#       rm -rf $build_dir
#       ./configure.sh
#    fi
# fi

# $_cmake --build "$build_dir" --config RelWithDebInfo --target $target -j 14 --

build_type=Debug
build_type=Release
build_type=RelWithDebInfo

_cc=/bin/clang-10
if [ ! -x "$_cc" ]; then
    _cc=/usr/bin/clang
fi
cxx=/bin/clang++-10
if [ ! -x "$cxx" ]; then
    cxx=/usr/bin/clang++
fi
# toolchain=$HOME/application-installs/vcpkg/scripts/buildsystems/vcpkg.cmake

if [ "$rebuild" = true ]; then
    rm -rf $build_dir
fi

catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=$build_type -DCMAKE_C_COMPILER:FILEPATH=$_cc -DCMAKE_CXX_COMPILER:FILEPATH=$cxx -DCMAKE_TOOLCHAIN_FILE:FILEPATH=$toolchain
. devel/setup.bash