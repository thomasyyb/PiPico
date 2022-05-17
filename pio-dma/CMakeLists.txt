cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project)

pico_sdk_init()

add_executable(test)

pico_generate_pio_header(test ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(test PRIVATE
        spi_cpu.c
        # spi_dma.c
        # spi_dma_chain.c
        )

pico_add_extra_outputs(test)

target_link_libraries(test 
        pico_stdlib
        hardware_pio
        hardware_dma
        hardware_irq
        pico_bootsel_via_double_reset)

# Entering the bootloader in this way also lets us specify a GPIO to be used
# as a bootloader activity LED:
# target_compile_definitions(test PRIVATE
#         PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
#         )