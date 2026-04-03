# 1. Target OS and Architecture
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 2. Compilers
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-g++") # Let GCC handle the .s files

# 3. Hardware-Specific Compiler Flags
# Define the common flags once so you don't repeat yourself
set(MCU_FLAGS "-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

add_compile_options(${MCU_FLAGS})
add_link_options(${MCU_FLAGS})
add_compile_options("-fsingle-precision-constant")


# 4. Linker Flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs")

# 5. Cross-Compilation Search Behavior
# Prevent CMake from accidentally linking against your Pop!_OS desktop libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force CMake to look in your specific folder first for objcopy, size, ar, etc.
set(CMAKE_FIND_ROOT_PATH "${CMAKE_CURRENT_LIST_DIR}/motorlib/gcc")