cmake_minimum_required(VERSION 3.10)
project(toyar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(GLUT REQUIRED)

# Include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/toyar/include)
include_directories(${OpenCV_INCLUDE_DIRS})

# Gather source files for the library (excluding main.cpp)
file(GLOB LIB_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/toyar/src/*.cpp"
)
list(REMOVE_ITEM LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/toyar/src/main.cpp")

# Create the library
add_library(toyar STATIC ${LIB_SOURCES})

# Public headers (so they can be found when linking this library)
target_include_directories(toyar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/toyar/include)

# Link libraries to toyar library
target_link_libraries(toyar 
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    glfw
    ${GLUT_LIBRARIES}
)

# Add executable for testing using main.cpp
add_executable(toyar_demo toyar/src/main.cpp)
target_link_libraries(toyar_demo PRIVATE toyar)

# Set compiler flags for better optimization and debugging
target_compile_options(toyar PRIVATE -Wall -Wextra -O2)
target_compile_options(toyar_demo PRIVATE -Wall -Wextra -O2)
