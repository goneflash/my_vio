set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB SOURCE_FILE
    "include/*.hpp"
    "src/*.cpp"
)

# TODO: Find other way than use as library
set(TARGET "util")

add_library(${TARGET} STATIC ${SOURCE_FILE})

target_include_directories(${TARGET} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
)
