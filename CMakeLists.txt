# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(rc_car)

# Generate a self-contained wiring reference from the same pin definitions used
# by the firmware. The artifact is build output and is never compiled or flashed.
set(PIN_MAP_TOOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools/esp32_pin_map")
set(PIN_MAP_OUTPUT "${CMAKE_BINARY_DIR}/esp32_pin_map.svg")

add_custom_command(
    OUTPUT "${PIN_MAP_OUTPUT}"
    COMMAND "${PYTHON}" "${PIN_MAP_TOOL_DIR}/generate_pin_map.py"
            --config "${CMAKE_CURRENT_SOURCE_DIR}/main/pin_config.h"
            --background "${PIN_MAP_TOOL_DIR}/esp32_board_minimal.png"
            --output "${PIN_MAP_OUTPUT}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/main/pin_config.h"
        "${PIN_MAP_TOOL_DIR}/generate_pin_map.py"
        "${PIN_MAP_TOOL_DIR}/esp32_board_minimal.png"
    COMMENT "Generating ESP32 powertrain pin map"
    VERBATIM
)

add_custom_target(esp32_pin_map ALL DEPENDS "${PIN_MAP_OUTPUT}")
