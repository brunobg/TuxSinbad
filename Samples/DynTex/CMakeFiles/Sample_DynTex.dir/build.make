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
include Samples/DynTex/CMakeFiles/Sample_DynTex.dir/depend.make

# Include the progress variables for this target.
include Samples/DynTex/CMakeFiles/Sample_DynTex.dir/progress.make

# Include the compile flags for this target's objects.
include Samples/DynTex/CMakeFiles/Sample_DynTex.dir/flags.make

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/flags.make
Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o: Samples/DynTex/src/DynTex.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -o CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex/src/DynTex.cpp

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -E /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex/src/DynTex.cpp > CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.i

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -S /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex/src/DynTex.cpp -o CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.s

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.requires:
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.requires

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.provides: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.requires
	$(MAKE) -f Samples/DynTex/CMakeFiles/Sample_DynTex.dir/build.make Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.provides.build
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.provides

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.provides.build: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.provides.build

# Object files for target Sample_DynTex
Sample_DynTex_OBJECTS = \
"CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o"

# External object files for target Sample_DynTex
Sample_DynTex_EXTERNAL_OBJECTS =

lib/Sample_DynTex.so: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o
lib/Sample_DynTex.so: lib/libOgreMain.so.1.7.2
lib/Sample_DynTex.so: lib/libOgreRTShaderSystem.so.1.7.2
lib/Sample_DynTex.so: /usr/lib/libOIS.so
lib/Sample_DynTex.so: lib/libOgreMain.so.1.7.2
lib/Sample_DynTex.so: /usr/lib/libfreetype.so
lib/Sample_DynTex.so: /usr/lib/libSM.so
lib/Sample_DynTex.so: /usr/lib/libICE.so
lib/Sample_DynTex.so: /usr/lib/libX11.so
lib/Sample_DynTex.so: /usr/lib/libXext.so
lib/Sample_DynTex.so: /usr/lib/libXt.so
lib/Sample_DynTex.so: /usr/lib/libXaw.so
lib/Sample_DynTex.so: /usr/lib/libboost_thread-mt.so
lib/Sample_DynTex.so: /usr/lib/libboost_date_time-mt.so
lib/Sample_DynTex.so: /usr/lib/libfreeimage.so
lib/Sample_DynTex.so: /usr/lib/libzzip.so
lib/Sample_DynTex.so: /usr/lib/libz.so
lib/Sample_DynTex.so: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/build.make
lib/Sample_DynTex.so: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../../lib/Sample_DynTex.so"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sample_DynTex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Samples/DynTex/CMakeFiles/Sample_DynTex.dir/build: lib/Sample_DynTex.so
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/build

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/requires: Samples/DynTex/CMakeFiles/Sample_DynTex.dir/src/DynTex.cpp.o.requires
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/requires

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/clean:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex && $(CMAKE_COMMAND) -P CMakeFiles/Sample_DynTex.dir/cmake_clean.cmake
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/clean

Samples/DynTex/CMakeFiles/Sample_DynTex.dir/depend:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/DynTex/CMakeFiles/Sample_DynTex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Samples/DynTex/CMakeFiles/Sample_DynTex.dir/depend

