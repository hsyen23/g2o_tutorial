# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/local/lib/python3.10/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.10/dist-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/g2o_tutorial/ICP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/g2o_tutorial/ICP/build

# Include any dependencies generated for this target.
include CMakeFiles/g2o_ICP.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/g2o_ICP.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/g2o_ICP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/g2o_ICP.dir/flags.make

CMakeFiles/g2o_ICP.dir/main.cpp.o: CMakeFiles/g2o_ICP.dir/flags.make
CMakeFiles/g2o_ICP.dir/main.cpp.o: /home/user/g2o_tutorial/ICP/main.cpp
CMakeFiles/g2o_ICP.dir/main.cpp.o: CMakeFiles/g2o_ICP.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/g2o_tutorial/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/g2o_ICP.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/g2o_ICP.dir/main.cpp.o -MF CMakeFiles/g2o_ICP.dir/main.cpp.o.d -o CMakeFiles/g2o_ICP.dir/main.cpp.o -c /home/user/g2o_tutorial/ICP/main.cpp

CMakeFiles/g2o_ICP.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_ICP.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/g2o_tutorial/ICP/main.cpp > CMakeFiles/g2o_ICP.dir/main.cpp.i

CMakeFiles/g2o_ICP.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_ICP.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/g2o_tutorial/ICP/main.cpp -o CMakeFiles/g2o_ICP.dir/main.cpp.s

# Object files for target g2o_ICP
g2o_ICP_OBJECTS = \
"CMakeFiles/g2o_ICP.dir/main.cpp.o"

# External object files for target g2o_ICP
g2o_ICP_EXTERNAL_OBJECTS =

g2o_ICP: CMakeFiles/g2o_ICP.dir/main.cpp.o
g2o_ICP: CMakeFiles/g2o_ICP.dir/build.make
g2o_ICP: /usr/local/lib/libopencv_gapi.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_highgui.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_ml.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_objdetect.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_photo.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_stitching.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_video.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_videoio.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_dnn.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_calib3d.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_features2d.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_flann.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_imgproc.so.4.6.0
g2o_ICP: /usr/local/lib/libopencv_core.so.4.6.0
g2o_ICP: /usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1
g2o_ICP: CMakeFiles/g2o_ICP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/g2o_tutorial/ICP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable g2o_ICP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_ICP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/g2o_ICP.dir/build: g2o_ICP
.PHONY : CMakeFiles/g2o_ICP.dir/build

CMakeFiles/g2o_ICP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/g2o_ICP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/g2o_ICP.dir/clean

CMakeFiles/g2o_ICP.dir/depend:
	cd /home/user/g2o_tutorial/ICP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/g2o_tutorial/ICP /home/user/g2o_tutorial/ICP /home/user/g2o_tutorial/ICP/build /home/user/g2o_tutorial/ICP/build /home/user/g2o_tutorial/ICP/build/CMakeFiles/g2o_ICP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/g2o_ICP.dir/depend

