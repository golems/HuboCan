cmake_minimum_required(VERSION 2.6)

get_filename_component(library_name ${CMAKE_CURRENT_LIST_DIR} NAME)

file(GLOB lib_source "src/*.cpp" "src/*.c" "*.hpp" "*.h")
list(SORT lib_source)

add_library(${library_name} SHARED ${lib_source})
target_link_libraries(${library_name} ach rt)


file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)

message(STATUS "\n -- ${library_name} UNIT TEST: ")
add_custom_target(check_${library_name} COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${library_name} ach rt)
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check_${library_name} ${test_base})
endforeach(utest_src_file)


file(GLOB ${library_name}_includes "*.h*")
LIST(SORT ${library_name}_includes)
install(FILES ${${library_name}_includes} DESTINATION "${HEADER_INSTALL_PATH}/${library_name}")

install(TARGETS ${library_name} DESTINATION ${LIBRARY_INSTALL_PATH})
