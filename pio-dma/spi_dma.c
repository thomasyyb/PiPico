/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "spi.pio.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#define PIN_SCK 18
#define PIN_CS 19
#define PIN_MOSI 16
#define PIN_MISO 17

#define BUF_SIZE 1

#define PWM_REPEAT_COUNT 1
#define N_PWM_LEVELS 32

int dma_chan;

// CPU driven SPI test calls this function
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
        printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi->pio, spi->sm));
        printf("%d in the rx_fifo\n", pio_sm_get_rx_fifo_level(spi->pio, spi->sm));
        sleep_ms(5);
    }
}

// This test is for CPU driven SPI
void test(const pio_spi_inst_t *spi) {
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

// define as global so that dma_handler can refer it
pio_spi_inst_t spi = {
            .pio = pio0,
            .sm = 0
    };

void dma_handler() {
    static int pwm_level = 0;
    static uint16_t wavetable[N_PWM_LEVELS];
    static bool first_run = true;
    // Entry number `i` has `i` one bits and `(32 - i)` zero bits.
    if (first_run) {
        first_run = false;
        // V_outB
        wavetable[0] = 0xB001;
        for (int i = 1; i < N_PWM_LEVELS; ++i) {
            wavetable[i] = 0xB000 + 0x7f * i;
            printf("wavetable[%d] = 0x%x\n", i, wavetable[i]);
        }
    }

    // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi.pio, spi.sm));

    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;

    // sleep for a while before the next transfer
    // sleep_ms(50);
    // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi.pio, spi.sm));

    // Give the channel a new wave table entry to read from, and re-trigger it
    dma_channel_set_read_addr(dma_chan, &wavetable[pwm_level], true);

    // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi.pio, spi.sm));

    pwm_level = (pwm_level + 1) % N_PWM_LEVELS;
    // printf("pwm_level = %d\n", pwm_level);

    // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi.pio, spi.sm));
}

int main() {
    stdio_init_all();

    /******** SPI configuration *********/

    // PIO clock is 4 times faster than the SPI clock
    // e.g. f_sys_clk = 125 MHz
    //      clk_div   = 12.5 MHz
    //      pio_clk   = 125/12.5 = 10 MHz
    //      spi_clk   = 10/4 = 2.5 MHz

    // Good
    // float clkdiv = 312.5f;  // 0.1 MHz @ 125 clk_sys
    // Good
    // float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    // Good
    // float clkdiv = 12.5f;  // 2.5 MHz @ 125 clk_sys
    // Starts to have glitches
    float clkdiv = 6.25f;  // 5 MHz @ 125 clk_sys
    // Getting worse
    // float clkdiv = 3.125f;  // 10 MHz @ 125 clk_sys
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

    /******** DMA configuration *********/

    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);
    // channel_config_set_dreq(&c, pio_get_dreq(spi.pio, spi.sm, true));

    dma_channel_configure(
        dma_chan,
        &c,
        // &spi.pio->txf[spi.sm],
        &pio0_hw->txf[0], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        PWM_REPEAT_COUNT, // Write the same value many times, then halt and interrupt
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Manually call the handler once, to trigger the first transfer
    dma_handler();

    // We're using DMA now
    // test(&spi);

    io_rw_16 *txfifo = (io_rw_16 *) spi.pio->txf[spi.sm];
    io_rw_16 *rxfifo = (io_rw_16 *) spi.pio->rxf[spi.sm];

    while(1) {
        // monitor the data in the fifo
        // printf("%d in the tx_fifo\n", pio_sm_get_tx_fifo_level(spi.pio, spi.sm));
        // printf("%d in the rx_fifo\n\n", pio_sm_get_rx_fifo_level(spi.pio, spi.sm));
        sleep_ms(1000);
    }
}
