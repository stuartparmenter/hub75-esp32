# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/examples/01_simple_test/*.*)

idf_component_register(SRCS ${app_sources})
