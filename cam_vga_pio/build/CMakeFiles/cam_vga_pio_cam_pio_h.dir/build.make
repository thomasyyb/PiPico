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
CMAKE_SOURCE_DIR = /home/thomas/PiPico/cam_vga_pio

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thomas/PiPico/cam_vga_pio/build

# Utility rule file for cam_vga_pio_cam_pio_h.

# Include the progress variables for this target.
include CMakeFiles/cam_vga_pio_cam_pio_h.dir/progress.make

CMakeFiles/cam_vga_pio_cam_pio_h: cam.pio.h


cam.pio.h: ../cam.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/PiPico/cam_vga_pio/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating cam.pio.h"
	pioasm/pioasm -o c-sdk /home/thomas/PiPico/cam_vga_pio/cam.pio /home/thomas/PiPico/cam_vga_pio/build/cam.pio.h

cam_vga_pio_cam_pio_h: CMakeFiles/cam_vga_pio_cam_pio_h
cam_vga_pio_cam_pio_h: cam.pio.h
cam_vga_pio_cam_pio_h: CMakeFiles/cam_vga_pio_cam_pio_h.dir/build.make

.PHONY : cam_vga_pio_cam_pio_h

# Rule to build all files generated by this target.
CMakeFiles/cam_vga_pio_cam_pio_h.dir/build: cam_vga_pio_cam_pio_h

.PHONY : CMakeFiles/cam_vga_pio_cam_pio_h.dir/build

CMakeFiles/cam_vga_pio_cam_pio_h.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cam_vga_pio_cam_pio_h.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cam_vga_pio_cam_pio_h.dir/clean

CMakeFiles/cam_vga_pio_cam_pio_h.dir/depend:
	cd /home/thomas/PiPico/cam_vga_pio/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thomas/PiPico/cam_vga_pio /home/thomas/PiPico/cam_vga_pio /home/thomas/PiPico/cam_vga_pio/build /home/thomas/PiPico/cam_vga_pio/build /home/thomas/PiPico/cam_vga_pio/build/CMakeFiles/cam_vga_pio_cam_pio_h.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cam_vga_pio_cam_pio_h.dir/depend

