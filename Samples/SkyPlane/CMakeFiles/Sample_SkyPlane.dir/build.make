# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /opt/kinect/LAB/Eclipse/TuxSimbad

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/kinect/LAB/Eclipse/TuxSimbad

# Include any dependencies generated for this target.
include Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/depend.make

# Include the progress variables for this target.
include Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/progress.make

# Include the compile flags for this target's objects.
include Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/flags.make

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/flags.make
Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o: Samples/SkyPlane/src/SkyPlane.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -o CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane/src/SkyPlane.cpp

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -E /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane/src/SkyPlane.cpp > CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.i

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -S /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane/src/SkyPlane.cpp -o CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.s

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.requires:
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.requires

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.provides: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.requires
	$(MAKE) -f Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/build.make Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.provides.build
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.provides

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.provides.build: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.provides.build

# Object files for target Sample_SkyPlane
Sample_SkyPlane_OBJECTS = \
"CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o"

# External object files for target Sample_SkyPlane
Sample_SkyPlane_EXTERNAL_OBJECTS =

lib/Sample_SkyPlane.so: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o
lib/Sample_SkyPlane.so: lib/libOgreMain.so.1.7.2
lib/Sample_SkyPlane.so: lib/libOgreRTShaderSystem.so.1.7.2
lib/Sample_SkyPlane.so: /usr/lib/libOIS.so
lib/Sample_SkyPlane.so: lib/libOgreMain.so.1.7.2
lib/Sample_SkyPlane.so: /usr/lib/libfreetype.so
lib/Sample_SkyPlane.so: /usr/lib/libSM.so
lib/Sample_SkyPlane.so: /usr/lib/libICE.so
lib/Sample_SkyPlane.so: /usr/lib/libX11.so
lib/Sample_SkyPlane.so: /usr/lib/libXext.so
lib/Sample_SkyPlane.so: /usr/lib/libXt.so
lib/Sample_SkyPlane.so: /usr/lib/libXaw.so
lib/Sample_SkyPlane.so: /usr/lib/libboost_thread-mt.so
lib/Sample_SkyPlane.so: /usr/lib/libboost_date_time-mt.so
lib/Sample_SkyPlane.so: /usr/lib/libfreeimage.so
lib/Sample_SkyPlane.so: /usr/lib/libzzip.so
lib/Sample_SkyPlane.so: /usr/lib/libz.so
lib/Sample_SkyPlane.so: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/build.make
lib/Sample_SkyPlane.so: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../../lib/Sample_SkyPlane.so"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sample_SkyPlane.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/build: lib/Sample_SkyPlane.so
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/build

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/requires: Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/src/SkyPlane.cpp.o.requires
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/requires

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/clean:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane && $(CMAKE_COMMAND) -P CMakeFiles/Sample_SkyPlane.dir/cmake_clean.cmake
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/clean

Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/depend:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Samples/SkyPlane/CMakeFiles/Sample_SkyPlane.dir/depend

