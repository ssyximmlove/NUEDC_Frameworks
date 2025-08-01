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
        Core/Inc/key.h
        Core/Src/gimbal.c
        Core/Inc/gimbal.h
        Core/Src/isr.c
        Core/Inc/isr.h
        Core/Src/vision_uart.c
        Core/Inc/vision_uart.h
        Core/Src/stepper_uart.c
        Core/Inc/stepper_uart.h
)

add_library(cmsis_dsp STATIC IMPORTED)

set_target_properties(cmsis_dsp PROPERTIES
        # 设置 .a 文件的实际路径
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a

        # 告诉 CMake，任何链接到 cmsis_dsp 的目标，都应该自动包含这个头文件目录
        INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc

        # 告诉 CMake，任何链接到 cmsis_dsp 的目标，都应该自动添加这个宏定义
        # [重要] 请根据你的 MCU 型号修改这个宏 (例如 ARM_MATH_CM7)
        INTERFACE_COMPILE_DEFINITIONS "ARM_MATH_CM4"
)

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
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include/dsp
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
        cmsis_dsp


    # Add user defined libraries
)
# 开启浮点打印
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -Wl,-u,_printf_float
        -Wl,-u,_scanf_float
)

# 启用FPU浮点运算库
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16

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