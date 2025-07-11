cmake_minimum_required(VERSION 3.13)

# The Target Board
# These can be found in lib/titan_boards/include/boards or be set to pico
# For example, if you want to use lib/titan_boards/include/boards/mk2/esc_board.h, set this to mk2/esc_board
# Is is preferred to define pins in the board header file and refer to that name rather than hardcoding names
# This also impacts the upload_tool for automatic device selection
set(PICO_BOARD amr/power_board)  # TODO: Set this to your target board

# The Target Robot
# These can be found in lib/titan_boards/include/robots
# Store any robot dependent configuration values in these files
set(UWRT_ROBOT amr)  # TODO: Set this to your target robot

# Include global definitions
execute_process(COMMAND git rev-parse --show-cdup
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE relative_path_to_repository_root OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)
get_filename_component(REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/${relative_path_to_repository_root}" ABSOLUTE)
include(${REPO_DIR}/global_defs.cmake)

# Define Project
# TODO: Rename project and executable to your project name
project(power_board C CXX ASM)
titan_firmware_init()

# Define executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_executable(power_board
    ${SOURCES}
)
target_include_directories(power_board PUBLIC include)

# Set version: major minor (see titan/version.h for more info)
titan_generate_version_tag(power_board 1 0)

# Configure pico-sdk
pico_enable_stdio_uart(power_board 1)
pico_set_program_name(power_board "Template Project")  # TODO: Set the project name

# Enable the 'make upload' and 'make info' commands for project
titan_use_upload_tool(power_board)
titan_enable_bootloader(power_board can)  # TODO: Select the same transport as below, or remove this line if using USB

# Define linking and targets
pico_add_extra_outputs(power_board)
target_link_libraries(power_board PUBLIC
    pico_stdlib     # Pico SDK Standard Library
    driver_led      # Status LED support
    # TODO: Select your transport:
    micro_ros_can   # Micro ros support (supports _can, _usb, _eth)
    titan_logger    # A nice little logging library
    titan_safety    # Handles critical code (Watchdog, Kill Switch, Fault Reporting)

    # TODO: Add any additional hardware here (ex. hardware_adc)
    micro_ros_usb
    driver_mfrc522
)

