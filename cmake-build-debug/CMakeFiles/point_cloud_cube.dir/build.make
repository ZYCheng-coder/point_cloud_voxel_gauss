# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/ubuntu/SoftWare/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/ubuntu/SoftWare/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/partTimeJob/point_cloud_cube

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/point_cloud_cube.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/point_cloud_cube.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/point_cloud_cube.dir/flags.make

CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o: CMakeFiles/point_cloud_cube.dir/flags.make
CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o: ../point_cloud_cube.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o -c /home/ubuntu/partTimeJob/point_cloud_cube/point_cloud_cube.cpp

CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/partTimeJob/point_cloud_cube/point_cloud_cube.cpp > CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.i

CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/partTimeJob/point_cloud_cube/point_cloud_cube.cpp -o CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.s

# Object files for target point_cloud_cube
point_cloud_cube_OBJECTS = \
"CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o"

# External object files for target point_cloud_cube
point_cloud_cube_EXTERNAL_OBJECTS =

point_cloud_cube: CMakeFiles/point_cloud_cube.dir/point_cloud_cube.cpp.o
point_cloud_cube: CMakeFiles/point_cloud_cube.dir/build.make
point_cloud_cube: CMakeFiles/point_cloud_cube.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable point_cloud_cube"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_cloud_cube.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/point_cloud_cube.dir/build: point_cloud_cube

.PHONY : CMakeFiles/point_cloud_cube.dir/build

CMakeFiles/point_cloud_cube.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/point_cloud_cube.dir/cmake_clean.cmake
.PHONY : CMakeFiles/point_cloud_cube.dir/clean

CMakeFiles/point_cloud_cube.dir/depend:
	cd /home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/partTimeJob/point_cloud_cube /home/ubuntu/partTimeJob/point_cloud_cube /home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug /home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug /home/ubuntu/partTimeJob/point_cloud_cube/cmake-build-debug/CMakeFiles/point_cloud_cube.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/point_cloud_cube.dir/depend
