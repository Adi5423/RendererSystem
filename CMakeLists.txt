cmake_minimum_required(VERSION 3.16)
project(NoPkgRen VERSION 0.1.0 LANGUAGES CXX)

# --------------------------------------------------
# Global C++ settings
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
elseif (MSVC)
    add_compile_options(/W4)
endif()

# --------------------------------------------------
# Source list
# --------------------------------------------------
set(SOURCES
    src/main.cpp

    src/math/Vec3.hpp

    src/core/Ray.hpp
    src/core/Camera.hpp
    src/core/Triangle.hpp
    src/core/Plane.hpp
    src/core/Geometry.hpp
    src/core/Scene.hpp
    src/core/Light.hpp

    src/platform/Win32Window.hpp
    src/platform/Win32Window.cpp
)

add_executable(NoPkgRen ${SOURCES})

target_include_directories(NoPkgRen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# --------------------------------------------------
# Link against Win32 libraries on Windows
# --------------------------------------------------
if (WIN32)
    target_link_libraries(NoPkgRen PRIVATE user32 gdi32)
endif()
