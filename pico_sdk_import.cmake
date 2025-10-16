# Looks for PICO_SDK_PATH env var and includes the SDK's import script.
if (DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if (NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Please set it to your pico-sdk path.")
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
