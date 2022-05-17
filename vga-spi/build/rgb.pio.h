// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --- //
// rgb //
// --- //

#define rgb_wrap_target 2
#define rgb_wrap 8

static const uint16_t rgb_program_instructions[] = {
    0x80a0, //  0: pull   block                      
    0xa047, //  1: mov    y, osr                     
            //     .wrap_target
    0xe000, //  2: set    pins, 0                    
    0xa022, //  3: mov    x, y                       
    0x23c1, //  4: wait   1 irq, 1               [3] 
    0x80a0, //  5: pull   block                      
    0x6403, //  6: out    pins, 3                [4] 
    0x6203, //  7: out    pins, 3                [2] 
    0x0045, //  8: jmp    x--, 5                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program rgb_program = {
    .instructions = rgb_program_instructions,
    .length = 9,
    .origin = -1,
};

static inline pio_sm_config rgb_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + rgb_wrap_target, offset + rgb_wrap);
    return c;
}

static inline void rgb_program_init(PIO pio, uint sm, uint offset, uint pin) {
    // creates state machine configuration object c, sets
    // to default configurations. I believe this function is auto-generated
    // and gets a name of <program name>_program_get_default_config
    // Yes, page 40 of SDK guide
    pio_sm_config c = rgb_program_get_default_config(offset);
    // Map the state machine's SET and OUT pin group to three pins, the `pin`
    // parameter to this function is the lowest one. These groups overlap.
    sm_config_set_set_pins(&c, pin, 3);
    sm_config_set_out_pins(&c, pin, 3);
    // Set clock division (Commented out, this one runs at full speed)
    // sm_config_set_clkdiv(&c, 5) ;
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    // Set the pin direction to output at the PIO (3 pins)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running (commented out, I'll start this in the C)
    // pio_sm_set_enabled(pio, sm, true);
}

#endif

