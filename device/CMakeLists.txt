# One INTERFACE library per device: core flags, headers and linker script.
# Link a bare-metal executable against exactly one of these.

function(add_device_target name arch ld)
    add_library(${name} INTERFACE)
    target_link_libraries(${name} INTERFACE ${arch})
    target_include_directories(${name} INTERFACE ${ARGN} ${CMAKE_CURRENT_LIST_DIR}/../CMSIS/Include)
    target_link_options(${name} INTERFACE
        $<$<CXX_COMPILER_ID:GNU>:-Wl,--no-warn-rwx-segments>
        -T${ld}
    )
    set_target_properties(${name} PROPERTIES INTERFACE_LINK_DEPENDS ${ld})
endfunction()

add_device_target(device_stm32g474 arch_cortex_m4f ${CMAKE_CURRENT_LIST_DIR}/stm32g4/stm32g474.ld ${CMAKE_CURRENT_LIST_DIR}/stm32g4/Include)
add_device_target(device_stm32g564 arch_cortex_m33f ${CMAKE_CURRENT_LIST_DIR}/stm32g5/stm32g564.ld ${CMAKE_CURRENT_LIST_DIR}/stm32g5/Include)
