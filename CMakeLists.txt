
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project(ibs_lib)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Find PCL library
find_package(PCL 1.8 REQUIRED)


add_definitions(${PCL_DEFINITIONS})


#for linker
set(QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)

#for reference ibs

set(IBS_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
set(IBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}  PARENT_SCOPE)
set(IBS_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src )
set(IBS_SOURCES_DIR ${IBS_SOURCES_DIR} PARENT_SCOPE )
set(IBS_SOURCES ${IBS_SOURCES_DIR}/voronoi.cpp ${IBS_SOURCES_DIR}/ibs.cpp)
set(IBS_SOURCES ${IBS_SOURCES} PARENT_SCOPE)
set(IBS_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include )
set(IBS_HEADERS_DIR ${IBS_HEADERS_DIR} PARENT_SCOPE)



#include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${QHULL_INCLUDE_DIR})
include_directories(${IBS_HEADERS_DIR})


add_library(ibs_lib STATIC  ${IBS_SOURCES} )

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_NAME} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})


add_subdirectory(apps)
add_subdirectory(test)
