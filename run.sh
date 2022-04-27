#!/bin/bash
set -e

world_name=main
_r=
while [[ $# -gt 0 ]]
do
    case $1 in
        -h|--help)
            echo "Usage: configure.sh [OPTIONS]"
            echo "Options:"
            echo "  -h, --help                      Print this help message"
            echo "  -w, --world [world]             Run the specified world file"
            echo "  -r, --rebuild                   Rebuild files from scratch"
            echo "  -- [gazebo args...]             Forward the remaining args to gazebo"
            exit 0
            ;;
        -w|--world)
            world_name=$2
            shift
            shift
            ;;
        -r|--rebuild)
            _r=-r
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

#install multiview from npm if necessary
# if ! command -v multiview >/dev/null; then
#     echo "Installing multiview..."
#     npm install -g multiview
# fi

proj_dir=$(pwd)
build_dir=$proj_dir/build

plugin_dir=$build_dir/farrt-bot_gazebo/plugins
export GAZEBO_PLUGIN_PATH="$plugin_dir":$GAZEBO_PLUGIN_PATH

model_dir=$proj_dir/src/farrt-bot_gazebo/models:$proj_dir/src
export GAZEBO_MODEL_PATH="$model_dir":$GAZEBO_MODEL_PATH

# world_dir=$proj_dir/worlds

# listener_dir=$proj_dir/listener

build_and_run() {
    # ./build.sh $_r

    echo "Project built! Running roslaunch..."
    # gazebo "$world_dir/$world_name.world" $@
    roslaunch farrt-bot_gazebo farrt-bot.launch $@
}
build_and_run
# multiview [ build_and_run ]
# echo $$
# echo $BASHPID
# pid=$BASHPID
# (build_and_run | multiview -s) & (./listener | multiview -s) & multiview

# (trap 'kill 0' SIGINT; (build_and_run | multiview -s) & (./listener | multiview -s) & multiview)