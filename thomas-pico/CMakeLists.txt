cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project)

pico_sdk_init()

add_executable(test)

pico_generate_pio_header(test ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(test PRIVATE
        test.c
        )

pico_add_extra_outputs(test)

target_link_libraries(test pico_stdlib hardware_pio)





# add_executable(pio_spi_loopback)

# pico_generate_pio_header(pio_spi_loopback ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

# target_sources(pio_spi_loopback PRIVATE
#         spi_loopback.c
#         pio_spi.c
#         pio_spi.h
#         )

# target_link_libraries(pio_spi_loopback PRIVATE pico_stdlib hardware_pio)
# pico_add_extra_outputs(pio_spi_loopback)

# example_auto_set_url(pio_spi_loopback)
