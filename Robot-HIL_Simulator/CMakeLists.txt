# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hil_sim_gemini)



# initialize the Pico SDK
pico_sdk_init()

# rest of your project

add_subdirectory(freertos)

add_executable(hil_sim_gemini
        hil_sim_gemini.cpp
        pwm_in.cpp
        robot_simulation.cpp
)


pico_set_program_name(hil_sim_gemini "freertos")
pico_set_program_version(hil_sim_gemini "0.1")

# Generate the PIO header
pico_generate_pio_header(hil_sim_gemini ${CMAKE_CURRENT_LIST_DIR}/pwm_in.pio)


# Link libraries
target_link_libraries(hil_sim_gemini
    pico_stdlib
    freertos
    hardware_gpio
    hardware_pwm   # Can likely remove this now
    hardware_clocks
    hardware_i2c
    hardware_pio   # Add hardware_pio
    hardware_irq
    hardware_sync
    pico_i2c_slave
    # FreeRTOS libraries if used directly (often handled by kernel include)
)
pico_add_extra_outputs(hil_sim_gemini)
pico_enable_stdio_usb(hil_sim_gemini 1)
