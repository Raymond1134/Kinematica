cmake_minimum_required(VERSION 3.20)
project(Kinematica VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sources
set(SOURCES
    main.cpp
    render/Renderer.cpp
    physics/PhysicsWorld.cpp
    physics/SoftBody.cpp
    physics/PhysicsFactory.cpp
    physics/BroadPhase.cpp
    physics/collision/TriangleMesh.cpp
    physics/Raycast.cpp
    ui/UI.cpp
    utils/JsonUtils.cpp
    utils/MaterialLoader.cpp
    utils/MeshGen.cpp
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
endif()

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

# Copy shaders to build directory
add_custom_command(TARGET app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:app>/shaders
)

# Group sources
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
