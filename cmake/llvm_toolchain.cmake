# 1. Target OS and Architecture
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(LLVM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../llvm")
set(BIN_DIR "${LLVM_ROOT}/bin")

set(TARGET_TRIPLE "armv7em-none-eabihf")
set(MCU_FLAGS "-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

# This is the "secret sauce" from your Makefile
set(COMMON_FLAGS "--target=${TARGET_TRIPLE}" "${MCU_FLAGS}" "-fdata-sections" "-ffunction-sections")

set(CMAKE_C_COMPILER "${BIN_DIR}/clang")
set(CMAKE_CXX_COMPILER "${BIN_DIR}/clang++")
set(CMAKE_ASM_COMPILER "${BIN_DIR}/clang")

add_compile_options(${COMMON_FLAGS})
add_link_options(${COMMON_FLAGS})
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x> $<$<COMPILE_LANGUAGE:ASM>:assembler-with-cpp>)

# Match your Makefile LDFLAGS exactly
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -lc -lm -lnosys -Wl,--gc-sections -Wl,--defsym=vfprintf=__f_vfprintf")

# 5. Cross-Compilation Search Behavior
# Prevent CMake from accidentally linking against your Pop!_OS desktop libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force CMake to look in your specific folder first for objcopy, size, ar, etc.
set(CMAKE_FIND_ROOT_PATH "${LLVM_BIN_DIR}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
