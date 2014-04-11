cmake_minimum_required(VERSION 2.6)

get_filename_component(dir_name ${CMAKE_CURRENT_LIST_DIR} NAME)

file(GLOB dir_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
list(SORT dir_source)

set(lib_source ${lib_source} ${dir_source} PARENT_SCOPE)

file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)

set(tests_source ${tests_source} ${unit_tests_source} PARENT_SCOPE)

file(GLOB include_files "*.h*")
LIST(SORT include_files)
install(FILES ${include_files} DESTINATION "${HEADER_INSTALL_PATH}/${dir_name}")
