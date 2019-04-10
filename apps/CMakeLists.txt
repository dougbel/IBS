
set(PROJECT_MAIN ${PROJECT_NAME}_main)

#for compiler
set(THIRD_PARTY_SOURCES ${IBS_DIR}/third_party/StopWatch/StopWatch.cpp)
set(THIRD_PARTY_HEADERS ${IBS_DIR}/third_party/StopWatch/header)

set(PROJECT_MAIN_SOURCES main.cpp  ${THIRD_PARTY_SOURCES})


include_directories(${THIRD_PARTY_HEADERS})

add_executable  (${PROJECT_MAIN} ${PROJECT_MAIN_SOURCES})

set_target_properties(${PROJECT_MAIN} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})
