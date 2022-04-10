# 7649-project

In order to build and run the Gazebo simulation, first install Gazebo version 11. Then, execute the following in the repository:
```
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/tbb@2020_u3
export CPATH=${CPATH}:/usr/local/opt/tbb@2020_u3/include
export LIBRARY_PATH=${LIBRARY_PATH}:/usr/local/opt/tbb@2020_u3/lib
cd main/sim/build
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:main/sim/build
cd main/sim/build
cmake ../
make
gazebo world_edit.world
```