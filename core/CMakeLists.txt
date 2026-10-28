cmake_minimum_required(VERSION 3.16)
project(aero_core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(AERO_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Werror)

# Shared library loaded by the Python simulator (ctypes).
add_library(aero_core SHARED src/aero_core.c)
target_include_directories(aero_core PUBLIC include)
target_compile_options(aero_core PRIVATE ${AERO_WARNINGS})
if(NOT WIN32)
  target_link_libraries(aero_core PRIVATE m)
endif()

# Plain-C smoke test: proves the core builds and runs without Python.
enable_testing()
add_executable(test_core_c tests/test_core.c src/aero_core.c)
target_include_directories(test_core_c PRIVATE include)
target_compile_options(test_core_c PRIVATE ${AERO_WARNINGS})
if(NOT WIN32)
  target_link_libraries(test_core_c PRIVATE m)
endif()
add_test(NAME test_core_c COMMAND test_core_c)
