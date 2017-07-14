include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

message(STATUS "${SOURCE_FILE}")

set(TARGET "imu_integrator")

add_library(${TARGET} STATIC ${SOURCE_FILE})
target_link_libraries( ${TARGET} ${EIGEN3_LIBS})

target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
