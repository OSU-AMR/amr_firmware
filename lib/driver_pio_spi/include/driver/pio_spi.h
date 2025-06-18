/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_SPI_H
#define _PIO_SPI_H

#include "spi.pio.h"

#include "hardware/pio.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

void pio_spi_write8_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len);

void pio_spi_read8_blocking(const pio_spi_inst_t *spi, uint8_t *dst, size_t len);

void pio_spi_write8_read8_blocking(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst, size_t len);

void pio_spi_init(PIO pio, uint sm, uint prog_offs, uint n_bits, float clkdiv, bool cpha, bool cpol, uint pin_sck,
                  uint pin_mosi, uint pin_miso);

#endif
