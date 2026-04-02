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
        device/stm32g4/Src/stm32g4xx_it.c
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
        obot-protocol/protocol_parser.cpp
)

set_source_files_properties(
    peripheral/stm32g4/flash.cpp 
    PROPERTIES COMPILE_OPTIONS "-fno-lto"
)

target_link_options(motorlib PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:-Wl,--no-warn-rwx-segments>
    -T${CMAKE_CURRENT_LIST_DIR}/peripheral/stm32g4/STM32G474RETx_FLASH.ld
)

find_package(Git QUIET)
if(NOT GIT_EXECUTABLE)
    set(GIT_EXECUTABLE "git") # Fallback if find_package fails
endif()

if(NOT DEFINED OBOT_HASH)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} # Assuming this runs in your project root
        OUTPUT_VARIABLE OBOT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

if(NOT DEFINED OBOT_VERSION)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --long --dirty --always --abbrev=7
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} # Assuming this runs in your project root
        OUTPUT_VARIABLE OBOT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

if(NOT DEFINED NOTES)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} branch --show-current
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} # Assuming this runs in your project root
        OUTPUT_VARIABLE NOTES
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} # This is the CMake equivalent of $(SELF_DIR)
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
