cmake_minimum_required(VERSION 2.6)
project(HuboCan)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected. Default to Release!")
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

option(BuildHuboQt "Build the HuboCan QWidgets" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions("-Wall")

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
set(HEADER_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR})
include_directories("./")

set(library_name ${PROJECT_NAME})
set(lib_source "")

set(tests_source "")

add_subdirectory(HuboState)
add_subdirectory(HuboCan)
add_subdirectory(HuboCmd)
add_subdirectory(HuboRT)
add_subdirectory(HuboPath)

add_library(${library_name} SHARED ${lib_source})
target_link_libraries(${library_name} ach)

file(GLOB bin_source "src/*.cpp")
list(SORT bin_source)

message(STATUS "\n ----- Binaries ----- ")
foreach(binary_src_file ${bin_source})
    get_filename_component(binary_name ${binary_src_file} NAME_WE)
    message(STATUS "Adding binary ${binary_name}")
    add_executable(${binary_name} ${binary_src_file})
    target_link_libraries(${binary_name} ${library_name} ach rt)
    install(TARGETS ${binary_name} DESTINATION ${EXECUTABLE_INSTALL_PATH})
endforeach(binary_src_file)

message(STATUS "\n -- ${library_name} UNIT TEST: ")
add_custom_target(check_${library_name} COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${library_name} ach rt)
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check_${library_name} ${test_base})
endforeach(utest_src_file)

install(TARGETS ${library_name} DESTINATION ${LIBRARY_INSTALL_PATH})

if(BuildHuboQt)
  include_directories(${CMAKE_BINARY_DIR}/HuboQt)
  add_subdirectory(HuboQt)
endif(BuildHuboQt)

