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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tyh/DB_plan_Project/build/so3_control

# Include any dependencies generated for this target.
include CMakeFiles/SO3Control.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SO3Control.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SO3Control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SO3Control.dir/flags.make

CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o: CMakeFiles/SO3Control.dir/flags.make
CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o: /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control/src/SO3Control.cpp
CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o: CMakeFiles/SO3Control.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tyh/DB_plan_Project/build/so3_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o -MF CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o.d -o CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o -c /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control/src/SO3Control.cpp

CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control/src/SO3Control.cpp > CMakeFiles/SO3Control.dir/src/SO3Control.cpp.i

CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control/src/SO3Control.cpp -o CMakeFiles/SO3Control.dir/src/SO3Control.cpp.s

# Object files for target SO3Control
SO3Control_OBJECTS = \
"CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o"

# External object files for target SO3Control
SO3Control_EXTERNAL_OBJECTS =

/home/tyh/DB_plan_Project/devel/.private/so3_control/lib/libSO3Control.so: CMakeFiles/SO3Control.dir/src/SO3Control.cpp.o
/home/tyh/DB_plan_Project/devel/.private/so3_control/lib/libSO3Control.so: CMakeFiles/SO3Control.dir/build.make
/home/tyh/DB_plan_Project/devel/.private/so3_control/lib/libSO3Control.so: CMakeFiles/SO3Control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tyh/DB_plan_Project/build/so3_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/tyh/DB_plan_Project/devel/.private/so3_control/lib/libSO3Control.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SO3Control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SO3Control.dir/build: /home/tyh/DB_plan_Project/devel/.private/so3_control/lib/libSO3Control.so
.PHONY : CMakeFiles/SO3Control.dir/build

CMakeFiles/SO3Control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SO3Control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SO3Control.dir/clean

CMakeFiles/SO3Control.dir/depend:
	cd /home/tyh/DB_plan_Project/build/so3_control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control /home/tyh/DB_plan_Project/src/read_only/quadrotor_simulator/so3_control /home/tyh/DB_plan_Project/build/so3_control /home/tyh/DB_plan_Project/build/so3_control /home/tyh/DB_plan_Project/build/so3_control/CMakeFiles/SO3Control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SO3Control.dir/depend
