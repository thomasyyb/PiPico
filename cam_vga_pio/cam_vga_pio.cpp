/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * Mandelbrot set calculation and visualization
 * Uses PIO-assembly VGA driver
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green bit 0
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Green bit 1
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0 and 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Our assembled programs:
// Each gets the name <pio_filename.pio.h>
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"


// needed for cam
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "ArduCAM.h"
#include "hardware/irq.h"
#include "ov5642_regs.h"
#include <cstdlib>
#include "stdio.h"
#include "pico/mutex.h"
#include "cam.pio.h"
#include "pio_spi.h"

// VGA timing constants
#define H_ACTIVE   655    // (active + frontporch - 1) - one cycle delay for mov
#define V_ACTIVE   479    // (active - 1)
#define RGB_ACTIVE 319    // (horizontal active)/2 - 1
// #define RGB_ACTIVE 639 // change to this if 1 pixel/byte

// Length of the pixel array, and number of DMA transfers
#define TXCOUNT 153600 // Total pixels/2 (since we have 2 pixels per byte)

// Pixel color array that is DMA's to the PIO machines and
// a pointer to the ADDRESS of this color array.
// Note that this array is automatically initialized to all 0's (black)
unsigned char vga_data_array[TXCOUNT];
// char vga_data_array[TXCOUNT];
char * address_pointer = (char*)(&vga_data_array[0]) ;

// Bit masks for drawPixel routine
// #define TOPMASK 0b11000111
// #define BOTTOMMASK 0b11111000
// we have four bits right now
#define TOPMASK 0b00001111
#define BOTTOMMASK 0b11110000

// Give the I/O pins that we're using some names that make sense
#define HSYNC       16
#define VSYNC       17
#define RED_PIN     18
#define GREEN_PIN_0 19
#define GREEN_PIN_1 20
#define BLUE_PIN    21
// change to four pins, two for green, needs to change the wiring

// We can only produce 8 colors, so let's give them readable names
// We can expand this right now
#define BLACK   0
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define MAGENTA 5
#define CYAN    6
#define WHITE   7


// A function for drawing a pixel with a specified color.
// Note that because information is passed to the PIO state machines through
// a DMA channel, we only need to modify the contents of the array and the
// pixels will be automatically updated on the screen.
void drawPixel(int x, int y, char color) {
    // Range checks
    if (x > 639) x = 639 ;
    if (x < 0) x = 0 ;
    if (y < 0) y = 0 ;
    if (y > 479) y = 479 ;

    // Which pixel is it?
    int pixel = ((640 * y) + x) ;

    // Is this pixel stored in the first 4 bits
    // of the vga data array index, or the second
    // 4 bits? Check, then mask.
    if (pixel & 1) {
        vga_data_array[pixel>>1] = (vga_data_array[pixel>>1] & TOPMASK) | (color << 4) ;
    }
    else {
        vga_data_array[pixel>>1] = (vga_data_array[pixel>>1] & BOTTOMMASK) | (color) ;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// Stuff for Camera ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// set pin 7 as the slave select for the digital pot:
const uint8_t CS = 5;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;
ArduCAM myCAM( OV5642, CS );
uint8_t read_fifo_burst(ArduCAM myCAM);
uint8_t read_fifo_slow(ArduCAM myCAM);
void cam_init();
void VGA_init();
void test(const pio_spi_inst_t *spi);

// #define PIO_PIN_SCK 10
// #define PIO_PIN_CS 11
// #define PIO_PIN_MOSI 12
// #define PIO_PIN_MISO 13

// use the same set as SDK's SPI
#define PIO_PIN_SCK 2
#define PIO_PIN_CS 5
#define PIO_PIN_MOSI 3
#define PIO_PIN_MISO 4

pio_spi_inst_t spi = {
            .pio = pio1,
            .sm = 0
    };

////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {

    // Initialize stdio
    stdio_init_all();
    printf("Hello\n");

    // Initialize Camera
    cam_init();

    // Initialize VGA
    VGA_init();

    while(1) {
        // comment the abundant communications
        // myCAM.flush_fifo();
        // myCAM.clear_fifo_flag();
        
        // implement in PIO assembly
        // Start capture
        myCAM.start_capture();
        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
            printf("ACK CMD CAM Capture Done.\n");
            read_fifo_slow(myCAM);
            //Clear the capture done flag
            myCAM.clear_fifo_flag();
        }
    }
}

void VGA_init() {
    // Choose which PIO instance to use (there are two instances, each with 4 state machines)
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember these locations!
    //
    // We only have 32 instructions to spend! If the PIO programs contain more than
    // 32 instructions, then an error message will get thrown at these lines of code.
    //
    // The program name comes from the .program part of the pio file
    // and is of the form <program name_program>
    uint hsync_offset = pio_add_program(pio, &hsync_program);
    uint vsync_offset = pio_add_program(pio, &vsync_program);
    uint rgb_offset = pio_add_program(pio, &rgb_program);

    // Manually select a few state machines from pio instance pio0.
    uint hsync_sm = 0;
    uint vsync_sm = 1;
    uint rgb_sm = 2;

    // Call the initialization functions that are defined within each PIO file.
    // Why not create these programs here? By putting the initialization function in
    // the pio file, then all information about how to use/setup that state machine
    // is consolidated in one place. Here in the C, we then just import and use it.
    hsync_program_init(pio, hsync_sm, hsync_offset, HSYNC);
    vsync_program_init(pio, vsync_sm, vsync_offset, VSYNC);
    rgb_program_init(pio, rgb_sm, rgb_offset, RED_PIN);


    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===========================-== DMA Data Channels =================================================
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // DMA channels - 0 sends color data, 1 reconfigures and restarts 0
    int rgb_chan_0 = 0;
    int rgb_chan_1 = 1;

    // Channel Zero (sends color data to PIO VGA machine)
    dma_channel_config c0 = dma_channel_get_default_config(rgb_chan_0);  // default configs
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);              // 8-bit txfers
    channel_config_set_read_increment(&c0, true);                        // yes read incrementing
    channel_config_set_write_increment(&c0, false);                      // no write incrementing
    channel_config_set_dreq(&c0, DREQ_PIO0_TX2) ;                        // DREQ_PIO0_TX2 pacing (FIFO)
    channel_config_set_chain_to(&c0, rgb_chan_1);                        // chain to other channel

    dma_channel_configure(
        rgb_chan_0,                 // Channel to be configured
        &c0,                        // The configuration we just created
        &pio->txf[rgb_sm],          // write address (RGB PIO TX FIFO)
        &vga_data_array,            // The initial read address (pixel color array)
        TXCOUNT,                    // Number of transfers; in this case each is 1 byte.
        false                       // Don't start immediately.
    );

    // Channel One (reconfigures the first channel)
    dma_channel_config c1 = dma_channel_get_default_config(rgb_chan_1);   // default configs
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);              // 32-bit txfers
    channel_config_set_read_increment(&c1, false);                        // no read incrementing
    channel_config_set_write_increment(&c1, false);                       // no write incrementing
    channel_config_set_chain_to(&c1, rgb_chan_0);                         // chain to other channel

    dma_channel_configure(
        rgb_chan_1,                         // Channel to be configured
        &c1,                                // The configuration we just created
        &dma_hw->ch[rgb_chan_0].read_addr,  // Write address (channel 0 read address)
        &address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
    );

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize PIO state machine counters. This passes the information to the state machines
    // that they retrieve in the first 'pull' instructions, before the .wrap_target directive
    // in the assembly. Each uses these values to initialize some counting registers.
    pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE);
    pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE);
    pio_sm_put_blocking(pio, rgb_sm, RGB_ACTIVE);

    // Start the two pio machine IN SYNC
    // Note that the RGB state machine is running at full speed,
    // so synchronization doesn't matter for that one. But, we'll
    // start them all simultaneously anyway.
    pio_enable_sm_mask_in_sync(pio, ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)));

    // Start DMA channel 0. Once started, the contents of the pixel color array
    // will be continously DMA's to the PIO machines that are driving the screen.
    // To change the contents of the screen, we need only change the contents
    // of that array.
    dma_start_channel_mask((1u << rgb_chan_0)) ;
}

// Camera initialization
void cam_init() {
    uint8_t vid, pid;
    uint8_t cameraCommand;
    myCAM.Arducam_init();
    printf("Hello\n");
    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);
    //Reset the CPLD
    myCAM.write_reg(0x07, 0x80);
    sleep_ms(100);
    myCAM.write_reg(0x07, 0x00);
    sleep_ms(100);
    while (1) 
    {
        //Check if the ArduCAM SPI bus is OK
        myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
        cameraCommand = myCAM.read_reg(ARDUCHIP_TEST1);
        if (cameraCommand != 0x55) {
            printf(" SPI interface Error!");
            sleep_ms(1000); continue;
        } else {
            printf("ACK CMD SPI interface OK.END\n"); break;
        }
    }
    while (1) 
    {
        //Check if the camera module type is OV5640
        myCAM.wrSensorReg16_8(0xff, 0x01);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
        if((vid != 0x56) || (pid != 0x42))
        {
            printf("Can't find OV5642 module!\n");
            sleep_ms(1000); continue;
        }
        else 
        {
            printf("OV5642 detected.END\n"); break;
        }
    }

    // Change to RGB capture mode and initialize the OV5642 module
    myCAM.set_format(RGB);
    // Change to JPEG capture mode and initialize the OV5642 module
    // myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
    // myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    sleep_ms(1000);
    myCAM.clear_fifo_flag();
    myCAM.write_reg(ARDUCHIP_FRAMES,0x00);

    // PIO SPI overrides the SDK SPI from now on
    // start to add spi.pio init
    /******** SPI configuration*********/

    // 4 MHz PIO clk @ 125 clk_sys
    // 1 MHz SPI clk
    float clkdiv = 31.25f;
    uint cam_spi_off = pio_add_program(spi.pio, &cam_program);
    uint cam_spi_off = pio_add_program(spi.pio, &cam_burst_program);
    cam_spi_program_init(spi.pio, spi.sm,
                    cam_spi_off,
                    8,       // 8 bits for one command
                    clkdiv,
                    PIO_PIN_SCK,
                    PIO_PIN_CS,
                    PIO_PIN_MOSI,
                    PIO_PIN_MISO
    );
    // for PIO SPI testing
    // test(&spi);
}

void test(const pio_spi_inst_t *spi) {
    uint8_t temp = 0x00;
    while(1) {
        static uint8_t txbuf = BURST_FIFO_READ;
        static uint8_t rxbuf[3];

        // printf("TX:");
        // for (int i = 0; i < 20; ++i) {
        //     txbuf[i] = temp;
        //     temp += 0x0f;
        //     if(temp >= 0x7f) temp = 0x00;
        //     printf(" %02x", (int) txbuf[i]);
        // }
        // printf("\n");

        // pio_spi_write8_read8_blocking(spi, txbuf, rxbuf, 20);
        cam_burst_read(spi, &txbuf, rxbuf);

        printf("rxbuf = %x %x %x", rxbuf[0], rxbuf[1], rxbuf[2]);

        // sleep_ms(200);
    }
}

// not using this anymore
uint8_t read_fifo_burst(ArduCAM myCAM)
{
    int i , count;
    int length = myCAM.read_fifo_length();
    printf("length = %d\n", length);
    uint8_t * imageBuf =(uint8_t *) malloc(length*sizeof(uint8_t));
    i = 0 ;
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();  // Set fifo burst mode
    spi_read_blocking(SPI_PORT, BURST_FIFO_READ, (uint8_t*)vga_data_array, length);
    myCAM.CS_HIGH();
    // This should be replaced with a SPI write
    // SerialUsb(imageBuf, length);
    free(imageBuf);
    return 1;
}

// change this to read into memory
// and then VGA output
uint8_t read_fifo_slow(ArduCAM myCAM)
{
    int length = myCAM.read_fifo_length();
    printf("length = %d\n", length);
    
    for (int m=0; m<240; m++) {
        for (int n=0; n<320; n++) {
            // char tmp_byte[2];
            // myCAM.CS_LOW();
            // myCAM.set_fifo_burst();  // Set fifo burst mode
            // spi_read_blocking(SPI_PORT, BURST_FIFO_READ, (uint8_t*)tmp_byte, 2);
            // myCAM.CS_HIGH();
            static uint8_t txbuf = BURST_FIFO_READ;
            static uint8_t rxbuf[3];
            cam_burst_read(&spi, &txbuf, rxbuf);
            char color = 0;
            color = color | ((( rxbuf[1] >> 3 ) > 16) << 0);  // red
            color = color | ((( rxbuf[1] & 0x03 ) > 1) << 1);  // green low bit
            color = color | ((( rxbuf[1] & 0x07 ) > 3) << 2);  // green high bit
            color = color | ((( rxbuf[2] & 0x1f ) > 16) << 3);  // blue
            // color = color | ((( tmp_byte[0] >> 3 ) > 16) << 0);  // red
            // color = color | ((( tmp_byte[0] & 0x03 ) > 1) << 1);  // green low bit
            // color = color | ((( tmp_byte[0] & 0x07 ) > 3) << 2);  // green high bit
            // color = color | ((( tmp_byte[1] & 0x1f ) > 16) << 3);  // blue
            // 3 blue 2 green_0 1 green_1 0 red
            drawPixel(n, m, color);
        }
    }
    return 1;
}
