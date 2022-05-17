/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
// #include "hardware/dma.h"
// #include "hardware/irq.h"
#include "spi.pio.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

#define PIN_SCK 18
#define PIN_CS 19
#define PIN_MOSI 16
#define PIN_MISO 17

#define BUF_SIZE 1

void __time_critical_func(pio_spi_write16_blocking)(const pio_spi_inst_t *spi, uint16_t *src, size_t len) {
    size_t tx_remain = len;
    // size_t rx_remain = len;
    io_rw_16 *txfifo = (io_rw_16 *) &spi->pio->txf[spi->sm];
    io_rw_16 *rxfifo = (io_rw_16 *) &spi->pio->rxf[spi->sm];
    printf("txfifo = %p\n", txfifo);
    printf("rxfifo = %p\n", rxfifo);
    while (tx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            (void) *rxfifo;  // Have to read data out of the rxfifo
            *txfifo = *src++;
            printf("TX: 0x%x\n", *txfifo);
            --tx_remain;
            printf("tx_remain = %d\n", tx_remain);
        }
        // if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
        //     (void) *rxfifo;
        //     --rx_remain;
        // }
        // printf("tx_remain = %d rx_remain = %d\n", tx_remain, rx_remain);
        // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi->pio, spi->sm));
        // printf("%d in the rx_fifo\n", pio_sm_get_rx_fifo_level(spi->pio, spi->sm));
        sleep_ms(10);
    }
}

void test(const pio_spi_inst_t *spi) {
    // V_outA, V_ref = 4.096
    uint16_t temp = 0x1000;
    while(1) {
        static uint16_t txbuf[BUF_SIZE];

        printf("TX:");
        for (int i = 0; i < BUF_SIZE; ++i) {
            txbuf[i] = temp;
            temp += 0x0f0;
            if(temp >= 0x1fff) temp = 0x1001;
            printf(" %02x", (int) txbuf[i]);
        }
        printf("\n");

        pio_spi_write16_blocking(spi, txbuf, BUF_SIZE);

        // sleep_ms(200);
    }
}

int main() {
    stdio_init_all();
    printf("Here!!!\n");

    /******** SPI configuration*********/
    pio_spi_inst_t spi = {
            .pio = pio0,
            .sm = 0
    };

    float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    uint cpha0_prog_offs = pio_add_program(spi.pio, &spi_cpha0_program);
    uint cpha1_prog_offs = pio_add_program(spi.pio, &spi_cpha1_program);

    int cpha = 1, cpol = 0;
    printf("CPHA = %d, CPOL = %d\n", cpha, cpol);
    pio_spi_init(spi.pio, spi.sm,
                    cpha ? cpha1_prog_offs : cpha0_prog_offs,
                    16,       // 16 bits for the write command
                    clkdiv,
                    cpha,
                    cpol,
                    PIN_SCK,
                    PIN_CS,
                    PIN_MOSI,
                    PIN_MISO
    );

    test(&spi);
}
