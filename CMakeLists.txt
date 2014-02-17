cmake_minimum_required(VERSION 2.6)
project(HuboCan)

add_definitions("-Wall")

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(HuboState)
include_directories(HuboState/include)

add_subdirectory(HuboCan)
include_directories(HuboCan/include)

add_subdirectory(HuboCmd)
include_directories(HuboCmd/include)

add_subdirectory(HuboAgg)
include_directories(HuboAgg/include)

add_subdirectory(HuboRT)
include_directories(HuboRT/include)

file(GLOB bin_source "src/*.cpp")
list(SORT bin_source)

add_executable(hubo_interface ${bin_source})
target_link_libraries(hubo_interface HuboRT HuboCan HuboAgg HuboState rt)


