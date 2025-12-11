# CMakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(BulletPhysic)

set(CMAKE_CXX_STANDARD 20)

# library sources
set(LIB_NAME ${PROJECT_NAME})

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX ".*/.*build.*/.*")

add_library(${LIB_NAME} STATIC ${SOURCES})

# public includes
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# define paths
target_compile_definitions(${LIB_NAME} PUBLIC DRAG_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/data/drag") # drag curves files path
