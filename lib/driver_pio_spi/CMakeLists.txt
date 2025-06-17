add_library(driver_pio_spi INTERFACE)

target_sources(driver_pio_spi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/pio_spi.c
)
pico_generate_pio_header(driver_pio_spi ${CMAKE_CURRENT_LIST_DIR}/src/spi.pio)

target_include_directories(driver_pio_spi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_pio_spi INTERFACE
    hardware_pio
)
