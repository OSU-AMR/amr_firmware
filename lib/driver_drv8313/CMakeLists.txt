add_library(driver_drv8313 INTERFACE)

target_sources(driver_drv8313 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/rp2040_mcu.c
    ${CMAKE_CURRENT_LIST_DIR}/src/bldc_driver_3pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/src/bldc_motor.c
)

target_include_directories(driver_drv8313 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_drv8313 INTERFACE
    hardware_pwm
)
