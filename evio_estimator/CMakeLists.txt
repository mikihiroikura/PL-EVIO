cmake_minimum_required(VERSION 2.8.3)
project(evio_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)
#find_package(LibQPEP REQUIRED)  #用于改善pnp初始化

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  #${LibQPEP_INCLUDE_DIRS}
)

catkin_package()

add_executable(evio_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/sampson_factor.cpp#
    src/factor/sampson_jacobian.cpp#
    src/factor/projectionOneFrameTwoCamFactor.cpp #
    src/factor/projectionTwoFrameOneCamFactor.cpp#
    src/factor/projectionTwoFrameTwoCamFactor.cpp #
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/solve_opt.cpp #
    src/initial/translate_factor.cpp #
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    # 新加入的线特征相关的
    src/sophus/common.hpp
    src/sophus/types.hpp
    src/sophus/so3.hpp
    src/utility/line_geometry.h
    src/utility/line_geometry.cpp
    src/factor/line_parameterization.h
    src/factor/line_parameterization.cpp
    src/factor/line_projection_factor.h
    src/factor/line_projection_factor.cpp
    )

target_link_libraries(evio_estimator 
${catkin_LIBRARIES} 
${OpenCV_LIBS} 
${CERES_LIBRARIES} 
#${LibQPEP_LIBRARIES}
#~/LibQPEP/build/libLibQPEP.so  #要加这个路径的映射
) 

add_executable(stereo_evio_estimator
    src/stereo_estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/sampson_factor.cpp#
    src/factor/sampson_jacobian.cpp#
    src/factor/projectionOneFrameTwoCamFactor.cpp #
    src/factor/projectionTwoFrameOneCamFactor.cpp#
    src/factor/projectionTwoFrameTwoCamFactor.cpp #
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/solve_opt.cpp #
    src/initial/translate_factor.cpp #
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    # 新加入的线特征相关的
    src/sophus/common.hpp
    src/sophus/types.hpp
    src/sophus/so3.hpp
    src/utility/line_geometry.h
    src/utility/line_geometry.cpp
    src/factor/line_parameterization.h
    src/factor/line_parameterization.cpp
    src/factor/line_projection_factor.h
    src/factor/line_projection_factor.cpp
    )

  target_link_libraries(stereo_evio_estimator 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CERES_LIBRARIES} 
  #${LibQPEP_LIBRARIES}
  #~/LibQPEP/build/libLibQPEP.so  #要加这个路径的映射
  ) 


