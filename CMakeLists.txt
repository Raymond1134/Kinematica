cmake_minimum_required(VERSION 3.20)
project(Kinematica LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sources
set(SOURCES main.cpp render/Renderer.cpp)

add_executable(app ${SOURCES})

# Directories for headers
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# raylib
find_package(raylib CONFIG REQUIRED)
target_link_libraries(app PRIVATE raylib)

# Windows system libs
if (WIN32)
    target_link_libraries(app PRIVATE winmm gdi32)
endif()

# Group sources
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
