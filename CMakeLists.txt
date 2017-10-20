cmake_minimum_required(VERSION 2.8.12)

set(PROJECT_NAME my_vio)
project( ${PROJECT_NAME})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
else() 
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# cmake -DTEST=ON to enable the tests.
option(USE_TEST "Build tests." OFF)
option(PROFILE_CPU "Profile CPU." OFF)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (USE_TEST)
  enable_testing()
  include_directories(thirdparty/googletest/googletest/include)
  add_subdirectory(thirdparty/googletest/googletest)

  # Configure path for test data
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp @ONLY IMMEDIATE)

  install(FILES config.hpp 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
endif()

if (ENABLE_VIZ)
  add_definitions("-DUSE_VISUALIZATION")
endif()

if (PROFILE_CPU)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lprofiler")
endif()

# FindX.cmake files
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Ceres )

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Use <package>_<component>_FOUND for a module in package.
if (OPENCV_SFM_FOUND)
  add_definitions("-DSFM_FOUND")
  # Fix for using cv::reconstruct
  add_definitions("-DCERES_FOUND")
  message(STATUS "OpenCV sfm is supported. Used in Landmark Initialization.")
else()
  message(WARNING "Libmv reconstruction is disabled. OpenCV sfm for reconstruction API is required.")
endif()

if (OPENCV_VIZ_FOUND)
  add_definitions("-DOPENCV_VIZ_FOUND")
endif()

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen library status:")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")

# For config.hpp used in tests in subdirectories.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(util)
add_subdirectory(mapdata)
add_subdirectory(camera_model)
add_subdirectory(simulator)
add_subdirectory(feature_tracker)
add_subdirectory(imu_integrator)
add_subdirectory(orientation_tracking)
add_subdirectory(multiview_helper)
add_subdirectory(map_initializer)
add_subdirectory(visual_inertial_odometry)

###########################################################################
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE my_vio)

# Add all targets to the build-tree export set
export(TARGETS 
  util
  mapdata
  feature_tracker 
  imu_integrator
  orientation_tracking
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

message(STATUS "Added Include Directory: ${CMAKE_INSTALL_PREFIX}")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/my_vioConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/my_vioConfig.cmake @ONLY IMMEDIATE)
