include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

set(TARGET "orientation_tracking")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET}
                       ${EIGEN3_LIBS}
                       imu_integrator )

target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

install(FILES ${headers} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
install(TARGETS ${TARGET} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
