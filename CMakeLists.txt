cmake_minimum_required(VERSION 3.16.0)

project(pointcloud_object_segmenter_lib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra -O2")
# -Werror 
# -Wall

# PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost

find_package(Boost)
link_libraries(${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIR})

# Conan

message( "CMAKE_BINARY_DIR : " ${CMAKE_BINARY_DIR} )
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)

conan_basic_setup(TARGETS)

include_directories(${CONAN_INCLUDE_DIRS} "include")

# add source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(
        ${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES} ${PCL_LIBRARIES})


# install
install(TARGETS ${PROJECT_NAME} DESTINATION lib) 
install(DIRECTORY include/ DESTINATION include
        FILES_MATCHING PATTERN "*.h")

# add tests
add_subdirectory(test)