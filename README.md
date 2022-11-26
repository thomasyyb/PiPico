# Camera Driver using Programmable I/O on RP2040

## Structure

ArduCam -> PIO driver -> DMA -> Memory -> DMA -> VGA output

## Progress

- Using software SPI to read pixel information, and write into the memory for VGA display
- 4 bit color, can be wired as grey scale picture

## Todo

- cam_vga: software version
- cam_vga_pio: pio driver of camera:
    - use software SPI to initialize camera first
    - setup PIO SPI, override SDK SPI
    - implement start capture, getbit, read_slow, and clear_fifo
