cmake_minimum_required(VERSION 3.16)

# Custom component layout — keep ESP-IDF's `main/` + `components/`
# convention but also surface additional components via
# EXTRA_COMPONENT_DIRS so we can group them by role (boards/, drivers/,
# vendor/, services/, ...) instead of dumping everything flat under
# components/.
#
# Single-component entries: the directory itself is the component
# (CMakeLists.txt + idf_component_register at that path).
# Parent-directory entries: ESP-IDF auto-discovers each subdirectory
# as its own component.
set(EXTRA_COMPONENT_DIRS
    boards
    drivers
    services
    vendor
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stability_tracker)
