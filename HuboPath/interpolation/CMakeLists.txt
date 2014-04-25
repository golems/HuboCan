cmake_minimum_required(VERSION 2.6)

get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

file(GLOB interp_source "*.cpp" "*.c" "*.hpp" "*.h")
list(SORT interp_source)

set(dir_source ${dir_source} ${interp_source} PARENT_SCOPE)
