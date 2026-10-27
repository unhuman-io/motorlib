set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(LLVM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../llvm")
set(LLVM_BIN_DIR "${LLVM_ROOT}/bin")

# --target and core-specific flags live in arch.cmake (arch_cortex_m4f etc.);
# link each target against the arch library it needs.

set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_ASM_COMPILER "clang")

# set(CLANG_CONFIG "--config=newlib.cfg")
# set(CMAKE_C_FLAGS_INIT "${CLANG_CONFIG}")
# set(CMAKE_CXX_FLAGS_INIT "${CLANG_CONFIG}")
# set(CMAKE_ASM_FLAGS_INIT "${CLANG_CONFIG}")

# set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CLANG_CONFIG}")
# set(CMAKE_MODULE_LINKER_FLAGS_INIT "${CLANG_CONFIG}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CLANG_CONFIG} -nostartfiles -lc -lm -Wl,-u,_printf_float")

#-Wl,--defsym=vfprintf=__f_vfprintf

# Cross-Compilation Search Behavior
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force CMake to look in specific folder first for objcopy, size, ar, etc.
list(APPEND CMAKE_PROGRAM_PATH "${LLVM_BIN_DIR}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
