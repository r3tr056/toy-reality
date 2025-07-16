cmake_minimum_required(VERSION 3.16)
project(toyar 
    VERSION 1.0.0
    DESCRIPTION "Modern C++ Augmented Reality Library"
    LANGUAGES CXX
)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(TOYAR_BUILD_TESTS "Build unit tests" ON)
option(TOYAR_BUILD_EXAMPLES "Build example applications" ON)
option(TOYAR_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(TOYAR_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(TOYAR_ENABLE_COVERAGE "Enable code coverage" OFF)
option(TOYAR_BUILD_SHARED "Build shared library" OFF)

# Find required packages with proper versioning
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d aruco)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)

# Optional packages
find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(GLFW3 glfw3)
endif()

# Try different GLFW package names
if(NOT GLFW3_FOUND)
    find_package(glfw3 3.3 QUIET)
    if(glfw3_FOUND)
        set(GLFW3_FOUND TRUE)
        set(GLFW3_LIBRARIES glfw)
    endif()
endif()

if(NOT GLFW3_FOUND)
    find_package(GLFW QUIET)
    if(GLFW_FOUND)
        set(GLFW3_FOUND TRUE)
        set(GLFW3_LIBRARIES ${GLFW_LIBRARIES})
    endif()
endif()

find_package(glm REQUIRED)
if(PkgConfig_FOUND)
    pkg_check_modules(GLUT glut)
endif()
if(NOT GLUT_FOUND)
    find_package(GLUT REQUIRED)
endif()

# Set up compiler flags
set(TOYAR_COMPILE_OPTIONS
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wno-unused-parameter>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
)

set(TOYAR_COMPILE_OPTIONS_DEBUG
    $<$<CXX_COMPILER_ID:GNU,Clang>:-g -O0 -DDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/Od /DDEBUG>
)

set(TOYAR_COMPILE_OPTIONS_RELEASE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -DNDEBUG -march=native>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>
)

# Sanitizers
if(TOYAR_ENABLE_ASAN)
    list(APPEND TOYAR_COMPILE_OPTIONS -fsanitize=address -fno-omit-frame-pointer)
    set(TOYAR_LINK_OPTIONS ${TOYAR_LINK_OPTIONS} -fsanitize=address)
endif()

if(TOYAR_ENABLE_TSAN)
    list(APPEND TOYAR_COMPILE_OPTIONS -fsanitize=thread)
    set(TOYAR_LINK_OPTIONS ${TOYAR_LINK_OPTIONS} -fsanitize=thread)
endif()

# Code coverage
if(TOYAR_ENABLE_COVERAGE)
    list(APPEND TOYAR_COMPILE_OPTIONS --coverage)
    set(TOYAR_LINK_OPTIONS ${TOYAR_LINK_OPTIONS} --coverage)
endif()

# Gather source files for the library (excluding main.cpp and legacy files)
file(GLOB_RECURSE LIB_SOURCES
    "toyar/src/*.cpp"
)
list(REMOVE_ITEM LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/toyar/src/main.cpp")

# Remove legacy files
file(GLOB_RECURSE LEGACY_SOURCES "toyar/src/legacy/*.cpp")
if(LEGACY_SOURCES)
    list(REMOVE_ITEM LIB_SOURCES ${LEGACY_SOURCES})
endif()

file(GLOB_RECURSE LIB_HEADERS
    "toyar/include/*.h"
    "toyar/include/*.hpp"
)

# Create the library
if(TOYAR_BUILD_SHARED)
    add_library(toyar SHARED ${LIB_SOURCES} ${LIB_HEADERS})
    target_compile_definitions(toyar PRIVATE TOYAR_EXPORTS)
    target_compile_definitions(toyar PUBLIC TOYAR_SHARED)
else()
    add_library(toyar STATIC ${LIB_SOURCES} ${LIB_HEADERS})
endif()

# Set target properties
set_target_properties(toyar PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

# Modern target-based configuration
target_include_directories(toyar
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/toyar/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/toyar/src
)

target_compile_features(toyar PUBLIC cxx_std_17)

target_compile_options(toyar 
    PRIVATE 
        ${TOYAR_COMPILE_OPTIONS}
        $<$<CONFIG:Debug>:${TOYAR_COMPILE_OPTIONS_DEBUG}>
        $<$<CONFIG:Release>:${TOYAR_COMPILE_OPTIONS_RELEASE}>
)

if(TOYAR_LINK_OPTIONS)
    target_link_options(toyar PRIVATE ${TOYAR_LINK_OPTIONS})
endif()

# Link libraries to toyar library with proper targets
target_link_libraries(toyar 
    PUBLIC
        ${OpenCV_LIBS}
        OpenGL::GL
        GLEW::GLEW
        glm::glm
    PRIVATE
        Threads::Threads
        ${GLUT_LIBRARIES}
)

# Add GLFW if found
if(GLFW3_FOUND)
    target_link_libraries(toyar PUBLIC ${GLFW3_LIBRARIES})
    target_compile_definitions(toyar PUBLIC TOYAR_HAS_GLFW=1)
else()
    message(WARNING "GLFW not found. Some features may not be available.")
    target_compile_definitions(toyar PUBLIC TOYAR_HAS_GLFW=0)
endif()

# Add alias for consistent usage
add_library(ToyAR::toyar ALIAS toyar)

# Demo executable
if(TOYAR_BUILD_EXAMPLES)
    add_executable(toyar_demo toyar/src/main.cpp)
    target_link_libraries(toyar_demo PRIVATE ToyAR::toyar)
    
    set_target_properties(toyar_demo PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    
    target_compile_options(toyar_demo PRIVATE ${TOYAR_COMPILE_OPTIONS})
    
    if(TOYAR_LINK_OPTIONS)
        target_link_options(toyar_demo PRIVATE ${TOYAR_LINK_OPTIONS})
    endif()
endif()

# Enable testing
if(TOYAR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS toyar
    EXPORT ToyARTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY toyar/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export targets
install(EXPORT ToyARTargets
    FILE ToyARTargets.cmake
    NAMESPACE ToyAR::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ToyAR
)

# Create config files
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ToyARConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ToyARConfig.cmake.in
    ToyARConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ToyAR
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ToyARConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ToyARConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ToyAR
)

# Summary
message(STATUS "ToyAR Configuration Summary:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build tests: ${TOYAR_BUILD_TESTS}")
message(STATUS "  Build examples: ${TOYAR_BUILD_EXAMPLES}")
message(STATUS "  Enable AddressSanitizer: ${TOYAR_ENABLE_ASAN}")
message(STATUS "  Enable ThreadSanitizer: ${TOYAR_ENABLE_TSAN}")
message(STATUS "  Enable coverage: ${TOYAR_ENABLE_COVERAGE}")
message(STATUS "  Build shared library: ${TOYAR_BUILD_SHARED}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
