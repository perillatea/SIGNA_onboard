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
include CMakeFiles/novatel.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/novatel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/novatel.dir/flags.make

CMakeFiles/novatel.dir/src/novatel.cpp.o: CMakeFiles/novatel.dir/flags.make
CMakeFiles/novatel.dir/src/novatel.cpp.o: ../src/novatel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gnss/novatel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/novatel.dir/src/novatel.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/novatel.dir/src/novatel.cpp.o -c /home/gnss/novatel/src/novatel.cpp

CMakeFiles/novatel.dir/src/novatel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novatel.dir/src/novatel.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gnss/novatel/src/novatel.cpp > CMakeFiles/novatel.dir/src/novatel.cpp.i

CMakeFiles/novatel.dir/src/novatel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novatel.dir/src/novatel.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gnss/novatel/src/novatel.cpp -o CMakeFiles/novatel.dir/src/novatel.cpp.s

CMakeFiles/novatel.dir/src/novatel.cpp.o.requires:

.PHONY : CMakeFiles/novatel.dir/src/novatel.cpp.o.requires

CMakeFiles/novatel.dir/src/novatel.cpp.o.provides: CMakeFiles/novatel.dir/src/novatel.cpp.o.requires
	$(MAKE) -f CMakeFiles/novatel.dir/build.make CMakeFiles/novatel.dir/src/novatel.cpp.o.provides.build
.PHONY : CMakeFiles/novatel.dir/src/novatel.cpp.o.provides

CMakeFiles/novatel.dir/src/novatel.cpp.o.provides.build: CMakeFiles/novatel.dir/src/novatel.cpp.o


# Object files for target novatel
novatel_OBJECTS = \
"CMakeFiles/novatel.dir/src/novatel.cpp.o"

# External object files for target novatel
novatel_EXTERNAL_OBJECTS =

libnovatel.a: CMakeFiles/novatel.dir/src/novatel.cpp.o
libnovatel.a: CMakeFiles/novatel.dir/build.make
libnovatel.a: CMakeFiles/novatel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gnss/novatel/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libnovatel.a"
	$(CMAKE_COMMAND) -P CMakeFiles/novatel.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/novatel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/novatel.dir/build: libnovatel.a

.PHONY : CMakeFiles/novatel.dir/build

CMakeFiles/novatel.dir/requires: CMakeFiles/novatel.dir/src/novatel.cpp.o.requires

.PHONY : CMakeFiles/novatel.dir/requires

CMakeFiles/novatel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/novatel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/novatel.dir/clean

CMakeFiles/novatel.dir/depend:
	cd /home/gnss/novatel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gnss/novatel /home/gnss/novatel /home/gnss/novatel/build /home/gnss/novatel/build /home/gnss/novatel/build/CMakeFiles/novatel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/novatel.dir/depend
