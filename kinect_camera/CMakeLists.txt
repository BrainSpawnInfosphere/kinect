cmake_minimum_required (VERSION 2.8.3)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -Wno-dev -lpthread)
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})


################ OpenCV #################################
find_package(OpenCV REQUIRED core)
message("OpenCV: ${FREENET_LINKER_FLAGS}")

################ USB ####################################
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})
message("======== USB ${LIBUSB_VERSION} =============")

################ libfreenect ############################
# NOTE: you may have to edit where you installed libfreenect if not using homebrew
set(FREENET_LINKER_FLAGS "-L/usr/local/lib -Wl,-rpath,/usr/local/lib -lfreenect")
include_directories("/usr/local/include/libfreenect")

################ Dynamic reconfigure ####################
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

################ Eigen ##############################
pkg_check_modules(EIGEN3 REQUIRED eigen3)
if(EIGEN3_FOUND)
	message("======== FOUND EIGEN 3 =============")
	include_directories(${EIGEN3_INCLUDE_DIRS})
endif(EIGEN3_FOUND)

# ---[ Kinect Driver
rosbuild_add_library (kinect_driver src/kinect_driver.cpp)
rosbuild_add_compile_flags (kinect_driver ${SSE_FLAGS})
target_link_libraries (kinect_driver  ${FREENET_LINKER_FLAGS})

rosbuild_add_library (kinect_driver_nodelets src/kinect_driver_nodelets.cpp)
rosbuild_add_compile_flags (kinect_driver_nodelets ${SSE_FLAGS})
target_link_libraries (kinect_driver_nodelets kinect_driver  ${FREENET_LINKER_FLAGS})

rosbuild_add_executable (kinect_node src/kinect_node.cpp)
target_link_libraries (kinect_node kinect_driver  ${FREENET_LINKER_FLAGS})
