include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

set(TARGET "map_initializer")

add_library(${TARGET} SHARED ${SOURCE_FILE})
target_link_libraries( ${TARGET} 
                       multiview_helper
                       util
                       ${OpenCV_LIBS} )

target_include_directories(${TARGET} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

# Headers
# TODO: Change to use {PROJECT_NAME}/{TARGET} as include directory.
install(FILES ${headers} 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
# Libraries
install(TARGETS ${TARGET}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )

if (USE_TEST)
  file(GLOB TEST_SOURCE_FILE
      "test/*.cpp" )
       
  add_executable(${TARGET}_test ${TEST_SOURCE_FILE})
  target_link_libraries(${TARGET}_test ${TARGET}
    gtest gtest_main
    ${OpenCV_LIBS} )
  if (OPENCV_VIZ_FOUND)
    target_link_libraries(${TARGET}_test ${TARGET}
      simulator)
  endif()
  add_test( MapInitializerTest ${TARGET}_test )
endif()
