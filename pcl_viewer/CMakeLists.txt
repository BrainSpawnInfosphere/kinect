cmake_minimum_required(VERSION 2.8.3)
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


#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#message("--- APPLE ---")

find_package(OpenCV REQUIRED)
rosbuild_add_boost_directories()

################ libfreenect ############################
#set(FREENET_LINKER_FLAGS "-L/usr/local/lib -Wl,-rpath,/usr/local/lib -lfreenect -lfreenect_sync")
#include_directories("/usr/local/include/libfreenect")

################ Eigen ##############################
pkg_check_modules(EIGEN3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	message("======== FOUND EIGEN 3 =============")
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(EIGEN3_FOUND)

################ PCL ##############################
find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################ GLUT #########################
find_package(GLUT)
if(GLUT_FOUND)
	message("======== FOUND Glut ${GLUT_VERSION} =============")
    include_directories(${GLUT_INCLUDE_DIRS} )
    link_directories(${GLUT_LIBRARY_DIRS})
    add_definitions(${GLUT_DEFINITIONS})
endif(GLUT_FOUND)

rosbuild_add_executable(pcl src/main_pcl.cpp)
target_link_libraries(pcl  ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} ${FREENET_LINKER_FLAGS} ${PCL_LIBRARIES})
set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL")
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")


