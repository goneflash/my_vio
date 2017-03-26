include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

message(STATUS "${SOURCE_FILE}")

set(TARGET "mapdata")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET} 
                       ${OpenCV_LIBS} )


target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


install(FILES ${headers} 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/)
install(TARGETS ${TARGET} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
