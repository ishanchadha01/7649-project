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

_cmake=/usr/bin/cmake

proj_dir=$(pwd)
build_dir=$proj_dir/build

if [ ! -d "$build_dir" ]; then
   echo "'$DIR' not found! Configuring CMake..."
   ./configure.sh
else
   if [ "$rebuild" = true ]; then
      echo "Deleting '$DIR'..."
      rm -rf $build_dir
      ./configure.sh
   fi
fi

$_cmake --build "$build_dir" --config RelWithDebInfo --target all -j 14 --
