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
CMAKE_SOURCE_DIR = /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build

# Include any dependencies generated for this target.
include test/CMakeFiles/test_servo.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/test_servo.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/test_servo.dir/flags.make

test/CMakeFiles/test_servo.dir/test_servo.cpp.o: test/CMakeFiles/test_servo.dir/flags.make
test/CMakeFiles/test_servo.dir/test_servo.cpp.o: ../test/test_servo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/test_servo.dir/test_servo.cpp.o"
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_servo.dir/test_servo.cpp.o -c /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/test/test_servo.cpp

test/CMakeFiles/test_servo.dir/test_servo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_servo.dir/test_servo.cpp.i"
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/test/test_servo.cpp > CMakeFiles/test_servo.dir/test_servo.cpp.i

test/CMakeFiles/test_servo.dir/test_servo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_servo.dir/test_servo.cpp.s"
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/test/test_servo.cpp -o CMakeFiles/test_servo.dir/test_servo.cpp.s

# Object files for target test_servo
test_servo_OBJECTS = \
"CMakeFiles/test_servo.dir/test_servo.cpp.o"

# External object files for target test_servo
test_servo_EXTERNAL_OBJECTS =

test/test_servo: test/CMakeFiles/test_servo.dir/test_servo.cpp.o
test/test_servo: test/CMakeFiles/test_servo.dir/build.make
test/test_servo: src/lkmotor_controller/liblkmotor_controller.a
test/test_servo: src/can_communicator/libcan_communicator.a
test/test_servo: src/lkmotor_controller/libmotor_config.a
test/test_servo: src/lkmotor_controller/libutils.a
test/test_servo: ../lib/pcan/amd64/libpcan.so
test/test_servo: test/CMakeFiles/test_servo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_servo"
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_servo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/test_servo.dir/build: test/test_servo

.PHONY : test/CMakeFiles/test_servo.dir/build

test/CMakeFiles/test_servo.dir/clean:
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test && $(CMAKE_COMMAND) -P CMakeFiles/test_servo.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/test_servo.dir/clean

test/CMakeFiles/test_servo.dir/depend:
	cd /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/test /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test /home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/build/test/CMakeFiles/test_servo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/test_servo.dir/depend
