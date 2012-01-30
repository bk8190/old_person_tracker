cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


rosbuild_add_library(graphcutlib src/GMMGraphCutSegmentation/GCv2p3/GCoptimization.cpp src/GMMGraphCutSegmentation/GCv2p3/graph.cpp src/GMMGraphCutSegmentation/GCv2p3/maxflow.cpp src/GMMGraphCutSegmentation/GCv2p3/LinkedBlockList.cpp)
rosbuild_add_compile_flags(graphcutlib -fpermissive)

rosbuild_add_executable(silhouette_tracker src/SilhouetteTracker.cpp)
rosbuild_add_executable(rate_limiter src/RateLimiter.cpp)

#rosbuild_add_executable(graph_example src/GMMGraphCutSegmentation/GCv2p3/example.cpp)
#target_link_libraries(graph_example graphcutlib)
#rosbuild_add_compile_flags(graph_example -fpermissive)

#rosbuild_add_executable(graphcut src/GMMGraphCutSegmentation/graphcut.cpp)
#target_link_libraries(graphcut graphcutlib)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
