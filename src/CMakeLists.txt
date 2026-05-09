# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# MatterBridge.cpp needs esp_matter (and CHIP) headers; without REQUIRES, the
# compile command for this component omits managed_components include paths.
idf_component_register(SRCS ${app_sources} REQUIRES esp_matter)
