include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

set(TARGET "simulator")

if (OPENCV_VIZ_FOUND)
  add_definitions("-DOPENCV_VIZ_FOUND")
endif()

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET} 
                       ${EIGEN3_LIBS}
                       ${OpenCV_LIBS}
                       camera_model
                       util)

target_include_directories(${TARGET} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

install(FILES ${headers} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
install(TARGETS ${TARGET} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )


# Add application
add_executable(${TARGET}_app ${TARGET}_app.cpp)
target_link_libraries(${TARGET}_app
                      ${TARGET}
                      ${OpenCV_LIBS} )
