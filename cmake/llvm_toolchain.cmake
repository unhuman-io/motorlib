set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(LLVM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../llvm")
set(BIN_DIR "${LLVM_ROOT}/bin")

set(TARGET_TRIPLE "armv7em-none-eabihf")
set(MCU_FLAGS "--target=${TARGET_TRIPLE}" "-mcpu=cortex-m4" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

set(CMAKE_C_COMPILER "${BIN_DIR}/clang")
set(CMAKE_CXX_COMPILER "${BIN_DIR}/clang++")
set(CMAKE_ASM_COMPILER "${BIN_DIR}/clang")

add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x> $<$<COMPILE_LANGUAGE:ASM>:assembler-with-cpp>)

set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -lc -lm -lnosys -Wl,--defsym=vfprintf=__f_vfprintf")

# Cross-Compilation Search Behavior
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force CMake to look in specific folder first for objcopy, size, ar, etc.
set(CMAKE_FIND_ROOT_PATH "${LLVM_BIN_DIR}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
