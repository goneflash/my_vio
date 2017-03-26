cmake_minimum_required(VERSION 2.8.12)

project(my_vio)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else() 
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# cmake -DTEST=ON to enable the tests.
option(USE_TEST "Build tests." OFF)
option(PROFILE_CPU "Profile CPU." OFF)

if (USE_TEST)
  enable_testing()
  include_directories(thirdparty/googletest/googletest/include)
  add_subdirectory(thirdparty/googletest/googletest)
endif()

if (PROFILE_CPU)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lprofiler")
endif()

# FindX.cmake files
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen library status:")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")

add_subdirectory(util)
add_subdirectory(mapdata)
