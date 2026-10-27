# One OBJECT library per device: core flags, headers, linker script and the
# device.* C++ modules (rcc_init, gpio_init, hrtim_init, ...). Link a
# bare-metal executable against exactly one of these and import the modules.

function(add_device_target name arch ld include_dir)
    add_library(${name} OBJECT)
    target_link_libraries(${name} PUBLIC ${arch})
    target_include_directories(${name} PUBLIC ${include_dir} ${CMAKE_CURRENT_LIST_DIR}/common ${CMAKE_CURRENT_LIST_DIR}/../CMSIS/Include)
    # module interfaces must be built with the same flags as their importers
    target_compile_options(${name} PUBLIC
        -fno-use-cxa-atexit
        -fno-threadsafe-statics
        -fno-exceptions
        -fno-rtti
        -fno-unwind-tables
    )
    target_link_options(${name} INTERFACE
        $<$<CXX_COMPILER_ID:GNU>:-Wl,--no-warn-rwx-segments>
        -T${ld}
    )
    set_target_properties(${name} PROPERTIES INTERFACE_LINK_DEPENDS ${ld})
    target_sources(${name} PUBLIC
        FILE_SET CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}
        FILES ${ARGN} common/hrtim_init.cppm
    )
endfunction()

add_device_target(device_stm32g474 arch_cortex_m4f ${CMAKE_CURRENT_LIST_DIR}/stm32g4/stm32g474.ld ${CMAKE_CURRENT_LIST_DIR}/stm32g4/Include
    stm32g4/rcc_init.cppm stm32g4/gpio_init.cppm)
add_device_target(device_stm32g564 arch_cortex_m33f ${CMAKE_CURRENT_LIST_DIR}/stm32g5/stm32g564.ld ${CMAKE_CURRENT_LIST_DIR}/stm32g5/Include
    stm32g5/rcc_init.cppm stm32g5/gpio_init.cppm stm32g5/play_init.cppm)
