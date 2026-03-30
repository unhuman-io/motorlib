add_library(motorlib)
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
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}
        FILES system.h
)

target_compile_definitions(motorlib
    PUBLIC
        STM32G474xx
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