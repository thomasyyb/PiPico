# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(cam_vga)

# initialize the sdk
pico_sdk_init()

#######

# name anything you want
add_executable(cam_vga)

# must match with pio filename and executable name from above
pico_generate_pio_header(cam_vga ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(cam_vga ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(cam_vga ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(cam_vga PRIVATE cam_vga.c)

# must match with executable name
target_link_libraries(cam_vga PRIVATE
                        pico_stdlib
                        hardware_pio
                        hardware_dma
                        pico_bootsel_via_double_reset)

# must match with executable name
pico_add_extra_outputs(cam_vga)
