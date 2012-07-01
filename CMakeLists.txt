
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

find_package(OpenCV REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
 src/depth_image_display.h
)

set(SOURCE_FILES
  src/image_view.cpp
  src/depth_image_display.cpp
  ${MOC_FILES}
) 

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OpenCV_LIBS})

