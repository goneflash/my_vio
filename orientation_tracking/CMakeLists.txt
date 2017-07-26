include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

message(STATUS "${SOURCE_FILE}")

set(TARGET "orientation_tracking")

add_library(${TARGET} STATIC ${SOURCE_FILE})
target_link_libraries( ${TARGET}
                       ${EIGEN3_LIBS}
                       imu_integrator )

target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
