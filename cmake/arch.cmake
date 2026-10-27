# Per-core architecture flags as INTERFACE libraries. Link an executable
# (or object library) against exactly one of these; the compile and link
# flags propagate transitively so the toolchain file stays core-agnostic.

function(add_arch_target name)
    add_library(${name} INTERFACE)
    target_compile_options(${name} INTERFACE ${ARGN})
    target_link_options(${name} INTERFACE ${ARGN})
endfunction()

add_arch_target(arch_cortex_m4f
    $<$<OR:$<C_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:Clang>>:--target=armv7em-none-eabihf>
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)

add_arch_target(arch_cortex_m33f
    $<$<OR:$<C_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:Clang>>:--target=armv8m.main-none-eabihf>
    -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard
)

# Default core for every target that doesn't pick one. A target that links a
# different arch_* library overrides it: INTERFACE options from linked
# libraries land after directory-level ones on the command line, and for both
# gcc and clang the last -mcpu/-mfpu/--target wins.
set(OBOT_DEFAULT_ARCH arch_cortex_m4f CACHE STRING "arch_* library linked into every target by default")
link_libraries(${OBOT_DEFAULT_ARCH})
