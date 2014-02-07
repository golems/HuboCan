cmake_minimum_required(VERSION 2.6)

get_filename_component(library_name ${CMAKE_CURRENT_LIST_DIR} NAME)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
file(GLOB lib_source "src/*.cpp" "include/*.hpp" "include/*.h")
list(SORT lib_source)

add_library(${library_name} SHARED ${lib_source})
