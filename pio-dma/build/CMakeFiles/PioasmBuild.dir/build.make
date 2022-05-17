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
CMAKE_SOURCE_DIR = /home/thomas/Pico/pio-dma

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thomas/Pico/pio-dma/build

# Utility rule file for PioasmBuild.

# Include the progress variables for this target.
include CMakeFiles/PioasmBuild.dir/progress.make

CMakeFiles/PioasmBuild: CMakeFiles/PioasmBuild-complete


CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-install
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-mkdir
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-download
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-update
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-patch
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-configure
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-build
CMakeFiles/PioasmBuild-complete: pioasm/src/PioasmBuild-stamp/PioasmBuild-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'PioasmBuild'"
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/CMakeFiles
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/CMakeFiles/PioasmBuild-complete
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-done

pioasm/src/PioasmBuild-stamp/PioasmBuild-install: pioasm/src/PioasmBuild-stamp/PioasmBuild-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'PioasmBuild'"
	cd /home/thomas/Pico/pio-dma/build/pioasm && /usr/bin/cmake -E echo_append
	cd /home/thomas/Pico/pio-dma/build/pioasm && /usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-install

pioasm/src/PioasmBuild-stamp/PioasmBuild-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'PioasmBuild'"
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pico-sdk/tools/pioasm
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm/tmp
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm/src
	/usr/bin/cmake -E make_directory /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-mkdir

pioasm/src/PioasmBuild-stamp/PioasmBuild-download: pioasm/src/PioasmBuild-stamp/PioasmBuild-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'PioasmBuild'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-download

pioasm/src/PioasmBuild-stamp/PioasmBuild-update: pioasm/src/PioasmBuild-stamp/PioasmBuild-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'PioasmBuild'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-update

pioasm/src/PioasmBuild-stamp/PioasmBuild-patch: pioasm/src/PioasmBuild-stamp/PioasmBuild-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'PioasmBuild'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-patch

pioasm/src/PioasmBuild-stamp/PioasmBuild-configure: pioasm/tmp/PioasmBuild-cfgcmd.txt
pioasm/src/PioasmBuild-stamp/PioasmBuild-configure: pioasm/src/PioasmBuild-stamp/PioasmBuild-update
pioasm/src/PioasmBuild-stamp/PioasmBuild-configure: pioasm/src/PioasmBuild-stamp/PioasmBuild-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'PioasmBuild'"
	cd /home/thomas/Pico/pio-dma/build/pioasm && /usr/bin/cmake "-GUnix Makefiles" /home/thomas/Pico/pico-sdk/tools/pioasm
	cd /home/thomas/Pico/pio-dma/build/pioasm && /usr/bin/cmake -E touch /home/thomas/Pico/pio-dma/build/pioasm/src/PioasmBuild-stamp/PioasmBuild-configure

pioasm/src/PioasmBuild-stamp/PioasmBuild-build: pioasm/src/PioasmBuild-stamp/PioasmBuild-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/thomas/Pico/pio-dma/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'PioasmBuild'"
	cd /home/thomas/Pico/pio-dma/build/pioasm && $(MAKE)

PioasmBuild: CMakeFiles/PioasmBuild
PioasmBuild: CMakeFiles/PioasmBuild-complete
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-install
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-mkdir
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-download
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-update
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-patch
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-configure
PioasmBuild: pioasm/src/PioasmBuild-stamp/PioasmBuild-build
PioasmBuild: CMakeFiles/PioasmBuild.dir/build.make

.PHONY : PioasmBuild

# Rule to build all files generated by this target.
CMakeFiles/PioasmBuild.dir/build: PioasmBuild

.PHONY : CMakeFiles/PioasmBuild.dir/build

CMakeFiles/PioasmBuild.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PioasmBuild.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PioasmBuild.dir/clean

CMakeFiles/PioasmBuild.dir/depend:
	cd /home/thomas/Pico/pio-dma/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thomas/Pico/pio-dma /home/thomas/Pico/pio-dma /home/thomas/Pico/pio-dma/build /home/thomas/Pico/pio-dma/build /home/thomas/Pico/pio-dma/build/CMakeFiles/PioasmBuild.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PioasmBuild.dir/depend

