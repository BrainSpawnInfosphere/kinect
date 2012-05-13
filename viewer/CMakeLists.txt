cmake_minimum_required(VERSION 2.8.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
	message("======== FOUND OpenCV ${OpenCV_VERSION} =============")
	include_directories(${OpenCV_INCLUDE_DIRS})
endif(OpenCV_FOUND)

rosbuild_add_boost_directories()

################ libfreenect ############################
#set(FREENET_LINKER_FLAGS "-L/usr/local/lib -Wl,-rpath,/usr/local/lib -lfreenect -lfreenect_sync")
#include_directories("/usr/local/include/libfreenect")

################ Eigen ##############################
pkg_check_modules(EIGEN3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	message("======== FOUND EIGEN ${EIGEN3_VERSION} =============")
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(EIGEN3_FOUND)

################ PCL ##############################
find_package(PCL 1.5 QUIET REQUIRED common filters)
if(PCL_FOUND)
	message("======== FOUND PCL ${PCL_VERSION} =============")
	include_directories(${PCL_INCLUDE_DIRS} )
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
endif(PCL_FOUND)

################ GLUT #########################
find_package(GLUT)
if(GLUT_FOUND)
	message("======== FOUND Glut =============")
    include_directories(${GLUT_INCLUDE_DIRS} )
    link_directories(${GLUT_LIBRARY_DIRS})
    add_definitions(${GLUT_DEFINITIONS})
endif(GLUT_FOUND)

################ OpenCV Viewer #########################
rosbuild_add_executable(depth_view src/depth_viewer/depth_viewer.cpp)
target_link_libraries(depth_view ${OpenCV_LIBS})


################ PCL Viewer #########################
rosbuild_add_executable(pcl_view src/pcl_viewer/pcl_viewer.cpp)
target_link_libraries(pcl_view  ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${PCL_LIBRARIES})
set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL")
