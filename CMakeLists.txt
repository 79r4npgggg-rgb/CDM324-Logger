# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)
set(IDF_TARGET "esp32s3")
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/board"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/storage"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/sensors/cdm324"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(CDM324-Logger)
