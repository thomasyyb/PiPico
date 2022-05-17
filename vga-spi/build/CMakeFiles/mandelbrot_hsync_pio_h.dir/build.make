# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/thomas/Pico/vga-spi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thomas/Pico/vga-spi/build

# Utility rule file for mandelbrot_hsync_pio_h.

# Include the progress variables for this target.
include CMakeFiles/mandelbrot_hsync_pio_h.dir/progress.make

CMakeFiles/mandelbrot_hsync_pio_h: hsync.pio.h


hsync.pio.h: ../hsync.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/vga-spi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating hsync.pio.h"
	pioasm/pioasm -o c-sdk /home/thomas/Pico/vga-spi/hsync.pio /home/thomas/Pico/vga-spi/build/hsync.pio.h

mandelbrot_hsync_pio_h: CMakeFiles/mandelbrot_hsync_pio_h
mandelbrot_hsync_pio_h: hsync.pio.h
mandelbrot_hsync_pio_h: CMakeFiles/mandelbrot_hsync_pio_h.dir/build.make

.PHONY : mandelbrot_hsync_pio_h

# Rule to build all files generated by this target.
CMakeFiles/mandelbrot_hsync_pio_h.dir/build: mandelbrot_hsync_pio_h

.PHONY : CMakeFiles/mandelbrot_hsync_pio_h.dir/build

CMakeFiles/mandelbrot_hsync_pio_h.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mandelbrot_hsync_pio_h.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mandelbrot_hsync_pio_h.dir/clean

CMakeFiles/mandelbrot_hsync_pio_h.dir/depend:
	cd /home/thomas/Pico/vga-spi/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thomas/Pico/vga-spi /home/thomas/Pico/vga-spi /home/thomas/Pico/vga-spi/build /home/thomas/Pico/vga-spi/build /home/thomas/Pico/vga-spi/build/CMakeFiles/mandelbrot_hsync_pio_h.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mandelbrot_hsync_pio_h.dir/depend

