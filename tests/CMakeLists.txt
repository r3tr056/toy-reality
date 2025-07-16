# Test configuration
cmake_minimum_required(VERSION 3.16)

# Find testing framework
find_package(GTest)
if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9fd7188aa8772630e7c76c6.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()

# Math library tests
add_executable(test_math
    test_math.cpp
)

target_link_libraries(test_math
    PRIVATE 
        ToyAR::toyar
        gtest_main
        gtest
)

target_compile_features(test_math PRIVATE cxx_std_17)

# Core functionality tests
add_executable(test_core
    test_core.cpp
)

target_link_libraries(test_core
    PRIVATE 
        ToyAR::toyar
        gtest_main
        gtest
)

target_compile_features(test_core PRIVATE cxx_std_17)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(test_math)
gtest_discover_tests(test_core)

# Custom test target for convenience
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose
    DEPENDS test_math test_core
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
