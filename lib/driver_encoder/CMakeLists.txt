add_library(driver_encoder INTERFACE)

target_sources(driver_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/quad_encoder.c
)

target_include_directories(driver_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_encoder INTERFACE
    hardware_sync
    driver_canbus
)
