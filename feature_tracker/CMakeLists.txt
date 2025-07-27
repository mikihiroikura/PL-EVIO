cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    # dvs_msgs      #要加上
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ~/catkin_ws_dvs/src/EVIO/feature_tracker/src/line_descriptor/include # find LSD 原版的这里写了绝对路径！
)


# 事件特征
add_executable(feature_tracker
    src/event_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/arc_star/arc_star_detector.cc
    src/utility/visualization.cpp

    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/bitarray_custom.hpp
    src/line_descriptor/src/bitops_custom.hpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
    src/line_descriptor/src/precomp_custom.hpp
    src/line_descriptor/src/types_custom.hpp
    )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

# 图像特征
add_executable(image_feature_tracker
    src/image_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/arc_star/arc_star_detector.cc

    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/bitarray_custom.hpp
    src/line_descriptor/src/bitops_custom.hpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
    src/line_descriptor/src/precomp_custom.hpp
    src/line_descriptor/src/types_custom.hpp
    )
target_link_libraries(image_feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

# 双目图像特征
add_executable(stereo_image_tracker
    src/stereo_image_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/arc_star/arc_star_detector.cc

    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/bitarray_custom.hpp
    src/line_descriptor/src/bitops_custom.hpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
    src/line_descriptor/src/precomp_custom.hpp
    src/line_descriptor/src/types_custom.hpp
    )
target_link_libraries(stereo_image_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

