cmake_minimum_required(VERSION 3.16)
project(multi_echelon_inventory CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(USE_CBC "Build the CbcMipSolver backend (requires COIN-OR CBC dev libraries)" OFF)

include_directories(include)

set(CORE_SOURCES
    src/MultiEchelonProblem.cpp
    src/AllLocalHeuristicSolver.cpp
    src/ExactDPSolver.cpp
)

add_library(meio_core ${CORE_SOURCES})
target_include_directories(meio_core PUBLIC include)

if(USE_CBC)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(CBC REQUIRED cbc)
  target_compile_definitions(meio_core PUBLIC MEIO_USE_CBC)
  target_include_directories(meio_core PUBLIC ${CBC_INCLUDE_DIRS})
  target_link_libraries(meio_core PUBLIC ${CBC_LIBRARIES})
endif()

add_executable(meio_demo src/main.cpp)
target_link_libraries(meio_demo PRIVATE meio_core)

add_executable(meio_tests
    tests/main.cpp
    tests/EchelonTests.cpp
)
target_include_directories(meio_tests PRIVATE tests)
target_link_libraries(meio_tests PRIVATE meio_core)

enable_testing()
add_test(NAME unit_tests COMMAND meio_tests)
