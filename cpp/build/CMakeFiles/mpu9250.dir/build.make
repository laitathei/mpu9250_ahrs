# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/laitathei/Desktop/mpu9250_ahrs/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/laitathei/Desktop/mpu9250_ahrs/cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/mpu9250.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mpu9250.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mpu9250.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mpu9250.dir/flags.make

CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o: CMakeFiles/mpu9250.dir/flags.make
CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o: ../lib/mpu9250.cpp
CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o: CMakeFiles/mpu9250.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laitathei/Desktop/mpu9250_ahrs/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o -MF CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o.d -o CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o -c /home/laitathei/Desktop/mpu9250_ahrs/cpp/lib/mpu9250.cpp

CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laitathei/Desktop/mpu9250_ahrs/cpp/lib/mpu9250.cpp > CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.i

CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laitathei/Desktop/mpu9250_ahrs/cpp/lib/mpu9250.cpp -o CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.s

# Object files for target mpu9250
mpu9250_OBJECTS = \
"CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o"

# External object files for target mpu9250
mpu9250_EXTERNAL_OBJECTS =

libmpu9250.so: CMakeFiles/mpu9250.dir/lib/mpu9250.cpp.o
libmpu9250.so: CMakeFiles/mpu9250.dir/build.make
libmpu9250.so: CMakeFiles/mpu9250.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/laitathei/Desktop/mpu9250_ahrs/cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmpu9250.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpu9250.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mpu9250.dir/build: libmpu9250.so
.PHONY : CMakeFiles/mpu9250.dir/build

CMakeFiles/mpu9250.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mpu9250.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mpu9250.dir/clean

CMakeFiles/mpu9250.dir/depend:
	cd /home/laitathei/Desktop/mpu9250_ahrs/cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laitathei/Desktop/mpu9250_ahrs/cpp /home/laitathei/Desktop/mpu9250_ahrs/cpp /home/laitathei/Desktop/mpu9250_ahrs/cpp/build /home/laitathei/Desktop/mpu9250_ahrs/cpp/build /home/laitathei/Desktop/mpu9250_ahrs/cpp/build/CMakeFiles/mpu9250.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mpu9250.dir/depend

