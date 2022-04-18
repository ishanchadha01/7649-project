#!/bin/bash
set -e

world_name=main
while [[ $# -gt 0 ]]
do
    case $1 in
        -h|--help)
            echo "Usage: configure.sh [OPTIONS]"
            echo "Options:"
            echo "  -h, --help                      Print this help message"
            echo "  -w, --world [world]             Run the specified world file"
            echo "  -- [gazebo args...]             Forward the remaining args to gazebo"
            exit 0
            ;;
        -w|--world)
            world_name=$2
            shift
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

proj_dir=$(pwd)
build_dir=$proj_dir/build

plugin_dir=$build_dir/plugins
export GAZEBO_PLUGIN_PATH="$plugin_dir":$GAZEBO_PLUGIN_PATH

model_dir=$proj_dir/models
export GAZEBO_MODEL_PATH="$model_dir":$GAZEBO_MODEL_PATH

world_dir=$proj_dir/worlds

listener_dir=$proj_dir/listener

./build.sh

echo "Project built! Running Gazebo..."
gazebo "$world_dir/$world_name.world" $@