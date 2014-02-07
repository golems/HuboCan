cmake_minimum_required(VERSION 2.6)
project(HuboCan)

add_subdirectory(HuboCan)
add_subdirectory(HuboCmd)
add_subdirectory(HuboAgg)
add_subdirectory(HuboState)

#set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
#set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include_directories(${PROJECT_SOURCE_DIR})

#file(GLOB can_source "can/*.cpp" "HuboCan/*.hpp")
#list(SORT can_source)

#file(GLOB cmd_source "cmd/*.cpp" "HuboCmd/*.hpp")
#list(SORT cmd_source)

#file(GLOB agg_source "agg/*.cpp" "HuboAgg/*.hpp")
#list(SORT agg_source)

#file(GLOB state_source "state/*.cpp" "HuboState/*.hpp")
#list(SORT state_source)

#file(GLOB bin_source "src/*.cpp")
#list(SORT bin_source)



#add_library(HuboCan SHARED ${can_source})
#add_library(HuboCmd SHARED ${cmd_source})
#add_library(HuboAgg SHARED ${agg_source})
#add_library(HuboState SHARED ${state_source})

#add_executable(hubo_interface ${bin_source})

