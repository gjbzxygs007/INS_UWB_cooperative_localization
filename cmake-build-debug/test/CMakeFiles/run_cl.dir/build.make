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


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu-jianan/Projects/Cooperative

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug

# Include any dependencies generated for this target.
include test/CMakeFiles/run_cl.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/run_cl.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/run_cl.dir/flags.make

test/CMakeFiles/run_cl.dir/run_cl.cpp.o: test/CMakeFiles/run_cl.dir/flags.make
test/CMakeFiles/run_cl.dir/run_cl.cpp.o: ../test/run_cl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_cl.dir/run_cl.cpp.o"
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_cl.dir/run_cl.cpp.o -c /home/ubuntu-jianan/Projects/Cooperative/test/run_cl.cpp

test/CMakeFiles/run_cl.dir/run_cl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_cl.dir/run_cl.cpp.i"
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu-jianan/Projects/Cooperative/test/run_cl.cpp > CMakeFiles/run_cl.dir/run_cl.cpp.i

test/CMakeFiles/run_cl.dir/run_cl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_cl.dir/run_cl.cpp.s"
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu-jianan/Projects/Cooperative/test/run_cl.cpp -o CMakeFiles/run_cl.dir/run_cl.cpp.s

# Object files for target run_cl
run_cl_OBJECTS = \
"CMakeFiles/run_cl.dir/run_cl.cpp.o"

# External object files for target run_cl
run_cl_EXTERNAL_OBJECTS =

../bin/run_cl: test/CMakeFiles/run_cl.dir/run_cl.cpp.o
../bin/run_cl: test/CMakeFiles/run_cl.dir/build.make
../bin/run_cl: ../lib/liblooselycl.so
../bin/run_cl: ../lib/liblibvncxx.a
../bin/run_cl: /usr/local/lib/libopencv_dnn.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_highgui.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_ml.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_objdetect.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_shape.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_stitching.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_superres.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_videostab.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_calib3d.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_features2d.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_flann.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_photo.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_video.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_videoio.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_imgcodecs.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_imgproc.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_viz.so.3.4.10
../bin/run_cl: /usr/local/lib/libopencv_core.so.3.4.10
../bin/run_cl: test/CMakeFiles/run_cl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_cl"
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_cl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/run_cl.dir/build: ../bin/run_cl

.PHONY : test/CMakeFiles/run_cl.dir/build

test/CMakeFiles/run_cl.dir/clean:
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test && $(CMAKE_COMMAND) -P CMakeFiles/run_cl.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_cl.dir/clean

test/CMakeFiles/run_cl.dir/depend:
	cd /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu-jianan/Projects/Cooperative /home/ubuntu-jianan/Projects/Cooperative/test /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test /home/ubuntu-jianan/Projects/Cooperative/cmake-build-debug/test/CMakeFiles/run_cl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_cl.dir/depend

