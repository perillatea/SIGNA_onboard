# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/gnss/novatel

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gnss/novatel/build

# Include any dependencies generated for this target.
include CMakeFiles/imt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imt.dir/flags.make

CMakeFiles/imt.dir/src/imt_msg.c.o: CMakeFiles/imt.dir/flags.make
CMakeFiles/imt.dir/src/imt_msg.c.o: ../src/imt_msg.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gnss/novatel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/imt.dir/src/imt_msg.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/imt.dir/src/imt_msg.c.o   -c /home/gnss/novatel/src/imt_msg.c

CMakeFiles/imt.dir/src/imt_msg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/imt.dir/src/imt_msg.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/gnss/novatel/src/imt_msg.c > CMakeFiles/imt.dir/src/imt_msg.c.i

CMakeFiles/imt.dir/src/imt_msg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/imt.dir/src/imt_msg.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/gnss/novatel/src/imt_msg.c -o CMakeFiles/imt.dir/src/imt_msg.c.s

CMakeFiles/imt.dir/src/imt_msg.c.o.requires:

.PHONY : CMakeFiles/imt.dir/src/imt_msg.c.o.requires

CMakeFiles/imt.dir/src/imt_msg.c.o.provides: CMakeFiles/imt.dir/src/imt_msg.c.o.requires
	$(MAKE) -f CMakeFiles/imt.dir/build.make CMakeFiles/imt.dir/src/imt_msg.c.o.provides.build
.PHONY : CMakeFiles/imt.dir/src/imt_msg.c.o.provides

CMakeFiles/imt.dir/src/imt_msg.c.o.provides.build: CMakeFiles/imt.dir/src/imt_msg.c.o


CMakeFiles/imt.dir/src/oem4.c.o: CMakeFiles/imt.dir/flags.make
CMakeFiles/imt.dir/src/oem4.c.o: ../src/oem4.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gnss/novatel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/imt.dir/src/oem4.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/imt.dir/src/oem4.c.o   -c /home/gnss/novatel/src/oem4.c

CMakeFiles/imt.dir/src/oem4.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/imt.dir/src/oem4.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/gnss/novatel/src/oem4.c > CMakeFiles/imt.dir/src/oem4.c.i

CMakeFiles/imt.dir/src/oem4.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/imt.dir/src/oem4.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/gnss/novatel/src/oem4.c -o CMakeFiles/imt.dir/src/oem4.c.s

CMakeFiles/imt.dir/src/oem4.c.o.requires:

.PHONY : CMakeFiles/imt.dir/src/oem4.c.o.requires

CMakeFiles/imt.dir/src/oem4.c.o.provides: CMakeFiles/imt.dir/src/oem4.c.o.requires
	$(MAKE) -f CMakeFiles/imt.dir/build.make CMakeFiles/imt.dir/src/oem4.c.o.provides.build
.PHONY : CMakeFiles/imt.dir/src/oem4.c.o.provides

CMakeFiles/imt.dir/src/oem4.c.o.provides.build: CMakeFiles/imt.dir/src/oem4.c.o


# Object files for target imt
imt_OBJECTS = \
"CMakeFiles/imt.dir/src/imt_msg.c.o" \
"CMakeFiles/imt.dir/src/oem4.c.o"

# External object files for target imt
imt_EXTERNAL_OBJECTS =

libimt.a: CMakeFiles/imt.dir/src/imt_msg.c.o
libimt.a: CMakeFiles/imt.dir/src/oem4.c.o
libimt.a: CMakeFiles/imt.dir/build.make
libimt.a: CMakeFiles/imt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gnss/novatel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libimt.a"
	$(CMAKE_COMMAND) -P CMakeFiles/imt.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imt.dir/build: libimt.a

.PHONY : CMakeFiles/imt.dir/build

CMakeFiles/imt.dir/requires: CMakeFiles/imt.dir/src/imt_msg.c.o.requires
CMakeFiles/imt.dir/requires: CMakeFiles/imt.dir/src/oem4.c.o.requires

.PHONY : CMakeFiles/imt.dir/requires

CMakeFiles/imt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imt.dir/clean

CMakeFiles/imt.dir/depend:
	cd /home/gnss/novatel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gnss/novatel /home/gnss/novatel /home/gnss/novatel/build /home/gnss/novatel/build /home/gnss/novatel/build/CMakeFiles/imt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imt.dir/depend

