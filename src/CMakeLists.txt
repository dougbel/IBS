cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project(IBS)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Find PCL library
find_package(PCL 1.8 REQUIRED)


#include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${QHULL_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


add_definitions(${PCL_DEFINITIONS})


#for compiler
set(SOURCES voronoi.cpp ibs.cpp StopWatch.cpp)
set(project_SOURCES main.cpp ${SOURCES})


#for linker
set  (QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)



add_executable  (${PROJECT_NAME} ${project_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_NAME} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})


#for testing
set(test_SOURCES main_test.cpp ${SOURCES})
add_executable  (${PROJECT_NAME}_test ${test_SOURCES})
set_target_properties(${PROJECT_NAME}_test PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_NAME}_test ${QHULL_LIBRARIES} ${PCL_LIBRARIES})


