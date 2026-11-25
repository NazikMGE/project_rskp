cmake_minimum_required(VERSION 3.16)

# Host build of the hardware-independent firmware modules and their tests.
#
# The DSP services (shot_detect, stability) depend on ESP-IDF only through
# esp_log.h, so the unmodified sources are compiled here with a desktop GCC
# against the stub in stubs/. This project is separate from the ESP-IDF
# project in ../../CMakeLists.txt and needs neither IDF_PATH nor the board.
#
#   cmake -S . -B <build dir> -G Ninja -DHOST_COVERAGE=ON
#   cmake --build <build dir>
#   ctest --test-dir <build dir> --output-on-failure
#
# tools/run_host_tests.ps1 wraps these steps and collects the gcov report.
project(stability_tracker_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(HOST_COVERAGE "Instrument the firmware modules for gcov" OFF)

set(FW_ROOT ../..)
get_filename_component(FW_ROOT_ABS ${FW_ROOT} ABSOLUTE)

add_compile_options(-Wall -Wextra)

# Journal behind the esp_log.h stub.
add_library(host_support STATIC host_log.c)
target_include_directories(host_support PUBLIC stubs .)

# One firmware service = one library built from the unmodified source.
function(add_firmware_service name)
  add_library(${name} STATIC ${FW_ROOT}/services/${name}/src/${name}.c)
  target_include_directories(${name} PUBLIC ${FW_ROOT}/services/${name}/include)
  target_link_libraries(${name} PUBLIC host_support m)
  if(HOST_COVERAGE)
    # Only the code under test is instrumented, not the tests themselves.
    # The source is recorded relative to the firmware root, so gcov (run
    # from that directory) finds it even when the checkout path contains
    # non-ASCII characters.
    target_compile_options(${name} PRIVATE --coverage -O0
                           "-fprofile-prefix-map=${FW_ROOT_ABS}=.")
    target_link_options(${name} PUBLIC --coverage)
  endif()
endfunction()

add_firmware_service(shot_detect)
add_firmware_service(stability)

enable_testing()

add_executable(shot_detect_test shot_detect_test.c)
target_link_libraries(shot_detect_test PRIVATE shot_detect)
add_test(NAME shot_detect COMMAND shot_detect_test)

add_executable(stability_test stability_test.c)
target_link_libraries(stability_test PRIVATE stability)
add_test(NAME stability COMMAND stability_test)

# Header-only report-rate selector from main/.
add_executable(imu_profile_test imu_profile_test.c)
target_include_directories(imu_profile_test PRIVATE ${FW_ROOT}/main)
add_test(NAME imu_profile COMMAND imu_profile_test)

# Modelled training fragment through both services; writes its CSV files
# and journal into the build directory.
add_executable(control_example control_example.c)
target_link_libraries(control_example PRIVATE shot_detect stability)
add_test(NAME control_example COMMAND control_example .)
