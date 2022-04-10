#!/bin/bash
set -e

proj_dir=$(pwd)
build_dir=$proj_dir/build


plugin_dir=$build_dir/plugins
export GAZEBO_PLUGIN_PATH=$plugin_dir:${GAZEBO_PLUGIN_PATH}

world_dir=$proj_dir/worlds
world_name=main

./build.sh

echo "Project built! Running Gazebo..."
gazebo $world_dir/$world_name.world $@