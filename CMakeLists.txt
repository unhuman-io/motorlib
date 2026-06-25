add_library(motorlib OBJECT)
set_target_properties(motorlib PROPERTIES LINKER_LANGUAGE CXX)

target_sources(motorlib
    PUBLIC
        FILE_SET project_headers TYPE HEADERS
        BASE_DIRS device/stm32g4/Include
        FILES device/stm32g4/Include/st_device.h
)

target_sources(motorlib
    PUBLIC
        FILE_SET cmsis_headers TYPE HEADERS
        BASE_DIRS CMSIS/Include
        FILES CMSIS/Include/core_cm4.h
)

target_sources(motorlib
    PUBLIC
        FILE_SET obot_protocol_headers TYPE HEADERS
        BASE_DIRS obot-protocol
        FILES obot-protocol/protocol_parser.h
)

target_sources(motorlib
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}
        FILES
            system.h
            version.h
            peripheral/stm32g4/can.h
            task.h
)

target_sources(motorlib
    PUBLIC
        FILE_SET motorlib_modules TYPE CXX_MODULES
        FILES 
            sensors/encoders/ma7xx_encoder.cppm
            sensors/encoders/ma782_encoder.cppm
)

target_compile_definitions(motorlib
    PUBLIC
        STM32G474xx
)

target_sources(motorlib
    PRIVATE
        device/stm32g4/Src/main.c
        device/stm32g4/Src/system_stm32g4xx.c
        peripheral/stm32g4/startup_stm32g474xx.s
        peripheral/stm32g4/can.cpp
        peripheral/stm32g4/clock_config.cpp
        peripheral/stm32g4/flash.cpp
        peripheral/stm32g4/hrpwm.cpp
        peripheral/stm32g4/spi_dma_usart.cpp
        peripheral/stm32g4/spi_dma.cpp
        peripheral/stm32g4/spi_slave_figure.cpp
        peripheral/stm32g4/spi_slave.cpp
        peripheral/stm32g4/stm32g4_serial.cpp
        peripheral/stm32g4/uart.cpp
        peripheral/stm32g4/usb.cpp
        peripheral/mailbox.cpp
        peripheral/protocol_commands.cpp
        peripheral/protocol_states_spi.cpp
        peripheral/protocol_states_uart.cpp
        peripheral/protocol.cpp
        peripheral/spi_encoder.cpp
        control_fun.cpp
        foc.cpp
        gpio.cpp
        hall.cpp
        malloc.cpp
        parameter_api.cpp
        sincos.cpp
        util.cpp
        system.cpp
        obot-protocol/protocol_parser.cpp
        #calibration_example.c
)

set_source_files_properties(
    peripheral/stm32g4/flash.cpp
    calibration_example.c
    PROPERTIES COMPILE_OPTIONS "-fno-lto"
)

set_source_files_properties(
    peripheral/stm32g4/startup_stm32g474xx.s
    PROPERTIES COMPILE_OPTIONS "-x;assembler-with-cpp"
)

target_link_options(motorlib PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:-Wl,--no-warn-rwx-segments>
    -T${CMAKE_CURRENT_LIST_DIR}/peripheral/stm32g4/STM32G474RETx_FLASH.ld
)

find_package(Git QUIET)
if(NOT GIT_EXECUTABLE)
    set(GIT_EXECUTABLE "git")
endif()

if(NOT DEFINED OBOT_HASH)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE OBOT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    set(OBOT_HASH "${OBOT_HASH}" PARENT_SCOPE)
endif()

if(NOT DEFINED OBOT_VERSION)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --long --dirty --always --abbrev=7
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE OBOT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

if(NOT DEFINED NOTES)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} branch --show-current
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE NOTES
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE MOTORLIB_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

target_compile_definitions(motorlib
    PUBLIC
        OBOT_HASH=\"${OBOT_HASH}\"
        MOTORLIB_HASH=\"${MOTORLIB_HASH}\"
        OBOT_VERSION=\"${OBOT_VERSION}\"
        NOTES=\"${NOTES}\"
        LOGGER_INIT_PRIORITY=200
        SPIDMA_INIT_PRIORITY=201
)


function(add_obot_param_target SOURCE_FILE)
    # Get the filename without extension (e.g., "motor_resolute")
    get_filename_component(FILE_BASE ${SOURCE_FILE} NAME_WLE)
    
    set(TGT_NAME "${FILE_BASE}")
    set(BIN_OUT  "${CMAKE_CURRENT_BINARY_DIR}/${FILE_BASE}.bin")
    
    # Create the data-only executable
    add_executable(${TGT_NAME} EXCLUDE_FROM_ALL ${SOURCE_FILE})
    target_compile_definitions(${TGT_NAME}
        PUBLIC
            OBOT_HASH=\"${OBOT_HASH}\"
    )
    target_compile_options(${TGT_NAME} PRIVATE "-Wno-initializer-overrides")

    # Strip LTO and disable standard startup
    set_target_properties(${TGT_NAME} PROPERTIES 
        INTERPROCEDURAL_OPTIMIZATION FALSE
        INTERPROCEDURAL_OPTIMIZATION_RELEASE FALSE
        INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO FALSE
        INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL FALSE
    )

    target_link_options(${TGT_NAME} PRIVATE 
        "-nostartfiles" 
        "-Wl,--entry=0"
        "-Wl,--no-gc-sections"
    )

    # Binary Extraction
    add_custom_command(
        TARGET ${TGT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary -S -j flash_param $<TARGET_FILE:${TGT_NAME}> ${BIN_OUT}
        BYPRODUCTS ${BIN_OUT}
        COMMENT "Generated Parameter Binary: ${BIN_OUT}"
        VERBATIM
    )
endfunction()

function(add_obot_firmware_target obot_config)
    set(tgt ${obot_config}.elf)
    
    # Use standard variables for the OUTPUT paths
    set(bin_full ${CMAKE_CURRENT_BINARY_DIR}/${obot_config}.bin)
    set(bin_no_param ${CMAKE_CURRENT_BINARY_DIR}/${obot_config}_noparam.bin)
    set(bin_param ${CMAKE_CURRENT_BINARY_DIR}/${obot_config}_param.bin)
    set(bin_calibration ${CMAKE_CURRENT_BINARY_DIR}/${obot_config}_calibration.bin)
    set(tgz_file ${CMAKE_CURRENT_BINARY_DIR}/${obot_config}.tgz)

    # Define the target FIRST
    add_executable(${tgt} EXCLUDE_FROM_ALL config_obot_g474_${obot_config}.cpp ../param/param_obot_g474_${obot_config}.cpp ${CALIBRATION_FILE})
    target_link_libraries(${tgt} PRIVATE motorlib)
    target_compile_definitions(${tgt} PRIVATE CONFIG=\"${obot_config}\")

    set_source_files_properties(
        ../param/param_obot_g474_${obot_config}.c
        ${CALIBRATION_FILE}
        PROPERTIES COMPILE_OPTIONS "-fno-lto;$<$<OR:$<C_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:Clang>>:-Wno-initializer-overrides>"
    )
    add_obot_param_target(../param/param_obot_g474_${obot_config}.cpp)

    # Generate binary variations
    add_custom_command(OUTPUT ${bin_full} ${bin_no_param} ${bin_calibration}
        DEPENDS ${tgt}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${tgt}> ${bin_full}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${tgt}> 
                --remove-section flash_param 
                --remove-section calibration_data 
                ${bin_no_param}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${tgt}> 
                --only-section calibration_data 
                ${bin_calibration}
        COMMENT "Generating bin variations for ${obot_config}..."
    )

    # Create the TGZ package
    set(pkg_dir ${CMAKE_CURRENT_BINARY_DIR}/${obot_config})
    set(load_script ${pkg_dir}/load_${obot_config}.sh)

    add_custom_command(OUTPUT ${tgz_file}
        DEPENDS ${bin_no_param} param_obot_g474_${obot_config} ${bin_calibration}
        
        # Directory Setup
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${pkg_dir}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${pkg_dir}
        COMMAND ${CMAKE_COMMAND} -E copy ${bin_no_param} ${bin_calibration} ${pkg_dir}/
        COMMAND ${CMAKE_COMMAND} -E copy param_obot_g474_${obot_config}.bin ${pkg_dir}/${obot_config}_param.bin
        
        # Script Generation
        COMMAND ${CMAKE_COMMAND} -E echo "#!/bin/bash -e" > ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "cd $(dirname $0)" >> ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "dfu-util -a0 -s 0x8000000 -D ${obot_config}_noparam.bin $@" >> ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "dfu-util -a0 -s 0x8060000 -D ${obot_config}_param.bin $@" >> ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "dfu-util -a0 -s 0x8070000 -D ${obot_config}_calibration.bin $@" >> ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "dfu-util -a0 -s 0x8000000:8:leave -U /tmp/dfu_tmp $@ || true" >> ${load_script}
        COMMAND ${CMAKE_COMMAND} -E echo "rm /tmp/dfu_tmp" >> ${load_script}
        COMMAND chmod +x ${load_script}
        
        # Packaging
        COMMAND ${CMAKE_COMMAND} -E tar czf ${tgz_file} --format=gnutar ${pkg_dir}/
        
        # CRITICAL: This ensures the shell receives the quotes correctly
        VERBATIM 
        COMMENT "Packaging ${obot_config} into tarball..."
    )
    set_target_properties(${tgt} PROPERTIES ADDITIONAL_CLEAN_FILES ${pkg_dir})

    # Final Targets
    add_custom_target(obot_g474_${obot_config}_pkg DEPENDS ${tgz_file})

    add_custom_target(load_${obot_config}
        COMMAND ${load_script}
        DEPENDS ${tgz_file}
        COMMENT "Flashing ${obot_config}..."
    )
endfunction()
