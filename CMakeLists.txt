cmake_minimum_required(VERSION 3.16)
project(NoPkgRen VERSION 0.1.0 LANGUAGES CXX)

# Global C++ settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
elseif (MSVC)
    add_compile_options(/W4)
endif()

# Our source files
set(SOURCES
    src/main.cpp
#    src/math/Vec3.hpp
#    src/core/Ray.hpp
#    src/core/Camera.hpp
)

# Build a single executable
add_executable(NoPkgRen ${SOURCES})

# For header-only stuff, we just add include dirs
target_include_directories(NoPkgRen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
