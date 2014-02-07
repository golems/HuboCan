cmake_minimum_required(VERSION 2.6)
project(HuboCan)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(HuboCan)
add_subdirectory(HuboCmd)
add_subdirectory(HuboAgg)
add_subdirectory(HuboState)


file(GLOB bin_source "src/*.cpp")
list(SORT bin_source)

add_executable(hubo_interface ${bin_source})
target_link_libraries(hubo_interface HuboCan HuboAgg HuboState)


