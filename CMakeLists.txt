cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

include(cmake/gcc-arm-none-eabi.cmake)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Frameworks)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/hal_oled.c
        Core/Inc/hal_oled.h
        Core/Src/key.c
        Core/Inc/key.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Segger RTT
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Libs/RTT/RTT/SEGGER_RTT.c
        Libs/RTT/RTT/SEGGER_RTT_printf.c
        Libs/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Libs/RTT/Config
        Libs/RTT/RTT
)

# u8g2

file(GLOB_RECURSE U8G2_SOURCES
        "Libs/u8g2/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${U8G2_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Libs/u8g2
)

# 张大头步进

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Libs/emm/Emm_V5.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        Libs/emm
)