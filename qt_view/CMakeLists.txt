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

find_package(OpenCV REQUIRED core)
if(OpenCV_FOUND)
	message("======== FOUND OpenCV ${OpenCV_VERSION} =============")
	include_directories(${OpenCV_INCLUDE_DIRS})
	message("${OpenCV_LIBRARIES}")
endif(OpenCV_FOUND)

rosbuild_add_boost_directories()

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
	message("${PCL_LIBRARIES}")
endif(PCL_FOUND)

# Qt
#find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
pkg_check_modules(QT REQUIRED QtXml QtCore QtGui QtOpenGL)
if(QT_FOUND)
	message("======== FOUND Qt ${QT_VERSION} =============")
	#include(${QT_USE_FILE})
	add_definitions(-DQT_NO_KEYWORDS)
	include_directories(${QT_INCLUDE_DIRS} /usr/local/Cellar/qt/4.8.0/include/QtXml)
	link_directories(${QT_LIBRARY_DIRS})
	message("--${QT_INCLUDE_DIRS}")
	message("--${QT_LIBS}")
endif(QT_FOUND)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories( ${QT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} /usr/local/Cellar/libqglviewer/2.3.15/QGLViewer.framework/Headers )

################ OpenCV Viewer #########################
#rosbuild_add_executable(depth_view src/depth_viewer/depth_viewer.cpp)
#target_link_libraries(depth_view ${OpenCV_LIBS})


################ PCL Viewer #########################
#rosbuild_add_executable(pcl_view src/pcl_viewer/pcl_viewer.cpp)
#target_link_libraries(pcl_view  ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${PCL_LIBRARIES})
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL")

#rosbuild_add_executable(aaa src/test3/main.cpp)
#target_link_libraries(aaa ${OPENGL_LIBRARY} ${PCL_LIBRARIES} ${QT_LIBRARIES})
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -F/usr/local/Cellar/libqglviewer/2.3.15 -framework QGLViewer -F/usr/local/Cellar/qt/4.8.0/lib -framework QtCore -framework QtOpenGL -framework QtXml -framework QtGui")


################ PCL Ball Viewer #########################
rosbuild_add_executable(ball src/ball_cloud/main.cpp)
target_link_libraries(ball ${OPENGL_LIBRARY} ${PCL_LIBRARIES} ${QT_LIBRARIES})
set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -F/usr/local/Cellar/libqglviewer/2.3.15 -framework QGLViewer -F/usr/local/Cellar/qt/4.8.0/lib -framework QtCore -framework QtOpenGL -framework QtXml -framework QtGui")
