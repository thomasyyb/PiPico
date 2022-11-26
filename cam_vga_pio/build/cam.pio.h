// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --- //
// cam //
// --- //

#define cam_wrap_target 0
#define cam_wrap 5

static const uint16_t cam_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block           side 0     
    0xe000, //  1: set    pins, 0         side 0     
    0x6101, //  2: out    pins, 1         side 0 [1] 
    0x5001, //  3: in     pins, 1         side 1     
    0x10e2, //  4: jmp    !osre, 2        side 1     
    0xe001, //  5: set    pins, 1         side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program cam_program = {
    .instructions = cam_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config cam_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + cam_wrap_target, offset + cam_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}
#endif

// --------- //
// cam_burst //
// --------- //

#define cam_burst_wrap_target 0
#define cam_burst_wrap 14

static const uint16_t cam_burst_program_instructions[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block           side 0     
    0xa047, //  1: mov    y, osr          side 0     
    0xe000, //  2: set    pins, 0         side 0     
    0x6101, //  3: out    pins, 1         side 0 [1] 
    0x5001, //  4: in     pins, 1         side 1     
    0x10e3, //  5: jmp    !osre, 3        side 1     
    0xafe2, //  6: mov    osr, y          side 0 [15]
    0x6101, //  7: out    pins, 1         side 0 [1] 
    0x5001, //  8: in     pins, 1         side 1     
    0x10e7, //  9: jmp    !osre, 7        side 1     
    0xafe2, // 10: mov    osr, y          side 0 [15]
    0x6101, // 11: out    pins, 1         side 0 [1] 
    0x5001, // 12: in     pins, 1         side 1     
    0x10eb, // 13: jmp    !osre, 11       side 1     
    0xe001, // 14: set    pins, 1         side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program cam_burst_program = {
    .instructions = cam_burst_program_instructions,
    .length = 15,
    .origin = -1,
};

static inline pio_sm_config cam_burst_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + cam_burst_wrap_target, offset + cam_burst_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

#include "hardware/gpio.h"
static inline void cam_spi_program_init(PIO pio, uint sm, uint offset, uint n_bits,
        float clkdiv, uint pin_sck, uint pin_cs, uint pin_mosi, uint pin_miso) {
    // pio_sm_config c = cam_program_get_default_config(offset);
    pio_sm_config c = cam_burst_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin_mosi, 1);
    sm_config_set_set_pins(&c, pin_cs, 1);
    sm_config_set_in_pins(&c, pin_miso);
    sm_config_set_sideset_pins(&c, pin_sck);
    // Only support MSB-first in this example code (shift to left, auto push/pull, threshold=nbits)
    // Turn off autopush
    sm_config_set_out_shift(&c, false, false, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);
    // MOSI, SCK output are low, CS output is high, MISO is input
    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_sck) | (1u << pin_mosi));
    pio_sm_set_pins_with_mask(pio, sm, 1, (1u << pin_cs));
    pio_sm_set_pindirs_with_mask(pio, sm, (1u << pin_sck) | (1u << pin_cs) |(1u << pin_mosi), (1u << pin_sck) | (1u << pin_cs) |(1u << pin_mosi) | (1u << pin_miso));
    pio_gpio_init(pio, pin_mosi);
    pio_gpio_init(pio, pin_miso);
    pio_gpio_init(pio, pin_sck);
    pio_gpio_init(pio, pin_cs);
    gpio_set_outover(pin_sck, GPIO_OVERRIDE_NORMAL);
    // SPI is synchronous, so bypass input synchroniser to reduce input delay.
    hw_set_bits(&pio->input_sync_bypass, 1u << pin_miso);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif

