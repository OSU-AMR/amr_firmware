add_library(driver_rfid INTERFACE)

target_sources(driver_rfid INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mfrc522.c
)

target_include_directories(driver_rfid INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_rfid INTERFACE
    titan_logger
    driver_pio_spi
)
