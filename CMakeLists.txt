cmake_minimum_required(VERSION 2.6)
project(HuboCan)

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB can_source "can/*.cpp" "HuboCan/*")
list(SORT can_source)

file(GLOB cmd_source "cmd/*.cpp" "HuboCmd/*")
list(SORT cmd_source)

file(GLOB agg_source "agg/*.cpp" "HuboAgg/*")
list(SORT agg_source)

file(GLOB bin_source "src/*.cpp")
list(SORT bin_source)

add_library(HuboCan SHARED ${can_source})
add_library(HuboCmd SHARED ${cmd_source})
add_library(HuboAgg SHARED ${agg_source})

add_executable(hubo_interface ${bin_source})

