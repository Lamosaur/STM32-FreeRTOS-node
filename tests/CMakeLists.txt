# Host-side tests for the target-independent application core (FR-0.6).
#
# These run with the host compiler against the doubles in tests/support, and are what CI
# runs on every push (FR-1.6). GoogleTest is fetched at configure time; it is the only
# build that needs the network, and only on a cold tree.

include(FetchContent)

FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.18.0
  GIT_SHALLOW TRUE
  EXCLUDE_FROM_ALL)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Threads REQUIRED)

# The FR-0.2 stress test runs real threads, so ASan and UBSan are worth their cost
# here: they turn a latent data race or a bad index into a hard failure.
option(NODE_SANITIZERS "Build the host tests with ASan and UBSan" ON)

add_library(node_test_support INTERFACE)
target_include_directories(node_test_support INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/firmware/app/include
  ${CMAKE_SOURCE_DIR}/firmware/hal/include
  ${CMAKE_SOURCE_DIR}/firmware/rtos/include)
target_link_libraries(node_test_support INTERFACE
  sensor_core GTest::gtest Threads::Threads)

add_executable(node_tests
  test_i2c_recovery.cpp
  test_i2c_arbiter.cpp
  test_supervisor.cpp
  test_sensor.cpp)

target_link_libraries(node_tests PRIVATE node_test_support GTest::gtest_main)

# Test code gets -Wall -Wextra but not the full -Wconversion set the core library uses:
# GoogleTest's macros and headers would drown the signal in third-party noise.
target_compile_options(node_tests PRIVATE -Wall -Wextra)

if(NODE_SANITIZERS)
  target_compile_options(node_tests PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(node_tests PRIVATE -fsanitize=address,undefined)
endif()

include(GoogleTest)
gtest_discover_tests(node_tests)

# The host-side tooling is part of the deliverable, so its tests run under the same
# ctest invocation as the C++ suite rather than in a separate command someone has to
# remember. The parser decides what a measurement is recorded as, which makes a silent
# parse failure a wrong figure in docs/measurements.md.
find_package(Python3 COMPONENTS Interpreter)
