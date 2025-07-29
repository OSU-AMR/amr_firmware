add_library(driver_thermistor INTERFACE)

target_sources(driver_thermistor INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/thermistor.c
)

target_include_directories(driver_thermistor INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

