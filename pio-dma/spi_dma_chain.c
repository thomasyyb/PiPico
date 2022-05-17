/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Use two DMA channels to make a programmed sequence of data transfers to the
// UART (a data gather operation). One channel is responsible for transferring
// the actual data, the other repeatedly reprograms that channel.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/structs/uart.h"
#include "spi.pio.h"

/***************** SPI ****************/
typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

#define PIN_CS 11
#define PIN_SCK 12
#define PIN_MOSI 13
#define PIN_MISO 17

/***************** DMA ****************/

// const uint16_t wavetable[9] = {
//     0x9001,
//     0x9001,
//     0x9001,
//     0x9fff,
//     0x9fff,
//     0x9001,
//     0x9001,
//     0x9001,
//     0x9001
// };

// const struct {const uint16_t *data;} dummy_block[] = {
//     wavetable
// };

const char word0[] = "Transferring ";
const char word1[] = "one ";
const char word2[] = "word ";
const char word3[] = "at ";
const char word4[] = "a ";
const char word5[] = "time.\n";

// Note the order of the fields here: it's important that the length is before
// the read address, because the control channel is going to write to the last
// two registers in alias 3 on the data channel:
//           +0x0        +0x4          +0x8          +0xC (Trigger)
// Alias 0:  READ_ADDR   WRITE_ADDR    TRANS_COUNT   CTRL
// Alias 1:  CTRL        READ_ADDR     WRITE_ADDR    TRANS_COUNT
// Alias 2:  CTRL        TRANS_COUNT   READ_ADDR     WRITE_ADDR
// Alias 3:  CTRL        WRITE_ADDR    TRANS_COUNT   READ_ADDR
//
// This will program the transfer count and read address of the data channel,
// and trigger it. Once the data channel completes, it will restart the
// control channel (via CHAIN_TO) to load the next two words into its control
// registers.

// const struct {uint32_t len; const char *data;} control_blocks[] = {
//     {count_of(word0) - 1, word0}, // Skip null terminator
//     {count_of(word1) - 1, word1},
//     {count_of(word2) - 1, word2},
//     {count_of(word3) - 1, word3},
//     {count_of(word4) - 1, word4},
//     {count_of(word5) - 1, word5},
//     {0, NULL}                     // Null trigger to end chain.
// };

const struct {unsigned int len; const char *data;} control_blocks[] = {
    {count_of(word0) - 1, word0}, // Skip null terminator
    {count_of(word1) - 1, word1},
    {count_of(word2) - 1, word2},
    {count_of(word3) - 1, word3},
    {count_of(word4) - 1, word4},
    {count_of(word5) - 1, word5},
    {0, NULL}                     // Null trigger to end chain.
};

int main() {

    stdio_init_all();

    /*********** SPI configuration **********/
    pio_spi_inst_t spi = {
            .pio = pio0,
            .sm = 0
    };

    // PIO clock is 4 times faster than the SPI clock
    // e.g. f_sys_clk = 125 MHz
    //      clk_div   = 12.5 MHz
    //      pio_clk   = 125/12.5 = 10 MHz
    //      spi_clk   = 10/4 = 2.5 MHz

    // Good
    float clkdiv = 312.5f;  // 0.1 MHz @ 125 clk_sys
    // Good
    // float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    // Good
    // float clkdiv = 12.5f;  // 2.5 MHz @ 125 clk_sys
    // Starts to have glitches
    // float clkdiv = 6.25f;  // 5 MHz @ 125 clk_sys
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

    uint16_t wavetable[32];
    // V_outA
    for (int i = 0; i < 32; ++i) {
        wavetable[i] = 0x3000 + 0x3f * i;
        printf("wavetable[%d] = 0x%x\n", i, wavetable[i]);
    }

    const struct {const uint16_t *data;} dummy_block[] = {
        wavetable
    };

    /*********** DMA configuration **********/
    puts("DMA configuration:");

    // ctrl_chan loads control blocks into data_chan, which executes them.
    int ctrl_chan = dma_claim_unused_channel(true);
    int data_chan = dma_claim_unused_channel(true);

    // The control channel transfers two words into the data channel's control
    // registers, then halts. The write address wraps on a two-word
    // (eight-byte) boundary, so that the control channel writes the same two
    // registers when it is next triggered.

    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);  // only read from wavetable
    channel_config_set_write_increment(&c, false);  // only write read_addr_trig
    // channel_config_set_ring(&c, true, 3); // 1 << 3 byte(=2 words) boundary on write ptr

    printf("&dummy_block[0] = %p\n", &dummy_block[0]);
    printf("&dummy_block    = %p\n", &dummy_block);
    printf("&wavetable[0]   = %p\n", &wavetable[0]);
    printf("&wavetable      = %p\n", &wavetable);
    printf("wavetable       = %p\n", wavetable);
    dma_channel_configure(
        ctrl_chan,
        &c,
        &dma_hw->ch[data_chan].al3_read_addr_trig, // Initial write address
        &dummy_block[0],                           // Initial read address
        1,                                         // Halt after each control block
        false                                      // Don't start yet
    );

    // The data channel is set up to write to the UART FIFO (paced by the
    // UART's TX data request signal) and then chain to the control channel
    // once it completes. The control channel programs a new read address and
    // data length, and retriggers the data channel.

    c = dma_channel_get_default_config(data_chan);
    // 16 bit for DAC write command
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    // Set data request on TX FIFO
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);
    // Trigger ctrl_chan when data_chan completes
    channel_config_set_chain_to(&c, ctrl_chan);
    // Raise the IRQ flag when 0 is written to a trigger register (end of chain):
    channel_config_set_irq_quiet(&c, true);
    // channel_config_set_irq_quiet(dma_channel_config *c, bool irq_quiet):
    // In QUIET mode, the channel does not generate IRQs at the end of every transfer block. Instead, an IRQ is raised when
    // NULL is written to a trigger register, indicating the end of a control block chain.

    dma_channel_configure(
        data_chan,
        &c,
        &pio0_hw->txf[0],
        NULL,           // Initial read address and transfer count are unimportant;
        32,             // the control channel will reprogram them each time.
        false           // Don't start yet.
    );

    // Everything is ready to go. Tell the control channel to load the first
    // control block. Everything is automatic from here.
    dma_start_channel_mask(1u << ctrl_chan);

    // The data channel will assert its IRQ flag when it gets a null trigger,
    // indicating the end of the control block list. We're just going to wait
    // for the IRQ flag instead of setting up an interrupt handler.
    while (!(dma_hw->intr & 1u << data_chan))
        tight_loop_contents();
    dma_hw->ints0 = 1u << data_chan;

    puts("DMA finished.");

    while(1) {
        printf("Done\n");
        sleep_ms(1);
    }
}
