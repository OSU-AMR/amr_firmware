add_library(driver_mfrc522 INTERFACE)

target_sources(driver_mfrc522 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mfrc522.c
)

target_include_directories(driver_mfrc522 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(driver_mfrc522 INTERFACE
    titan_logger
    driver_pio_spi
)
