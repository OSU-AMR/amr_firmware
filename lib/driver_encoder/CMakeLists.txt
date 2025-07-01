add_library(driver_encoder INTERFACE)

target_sources(driver_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/quad_encoder.c
)
pico_generate_pio_header(driver_encoder ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_decoder.pio)

target_include_directories(driver_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_encoder INTERFACE
    hardware_pio
    hardware_dma
)
