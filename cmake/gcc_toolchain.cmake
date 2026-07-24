set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-g++")

# Hardware-Specific Compiler Flags
set(MCU_FLAGS "-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

# Linker Flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs -u _printf_float")

# Cross-Compilation Search Behavior
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force CMake to look in specific folder first for objcopy, size, ar, etc.
list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_LIST_DIR}/../gcc/bin")
