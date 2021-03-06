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
include Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/depend.make

# Include the progress variables for this target.
include Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/progress.make

# Include the compile flags for this target's objects.
include Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/flags.make

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/flags.make
Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o: Samples/SphereMapping/src/SphereMapping.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -o CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping/src/SphereMapping.cpp

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -E /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping/src/SphereMapping.cpp > CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.i

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DOGRE_GCC_VISIBILITY -fvisibility=hidden -fvisibility-inlines-hidden -S /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping/src/SphereMapping.cpp -o CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.s

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.requires:
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.requires

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.provides: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.requires
	$(MAKE) -f Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/build.make Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.provides.build
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.provides

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.provides.build: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.provides.build

# Object files for target Sample_SphereMapping
Sample_SphereMapping_OBJECTS = \
"CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o"

# External object files for target Sample_SphereMapping
Sample_SphereMapping_EXTERNAL_OBJECTS =

lib/Sample_SphereMapping.so: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o
lib/Sample_SphereMapping.so: lib/libOgreMain.so.1.7.2
lib/Sample_SphereMapping.so: lib/libOgreRTShaderSystem.so.1.7.2
lib/Sample_SphereMapping.so: /usr/lib/libOIS.so
lib/Sample_SphereMapping.so: lib/libOgreMain.so.1.7.2
lib/Sample_SphereMapping.so: /usr/lib/libfreetype.so
lib/Sample_SphereMapping.so: /usr/lib/libSM.so
lib/Sample_SphereMapping.so: /usr/lib/libICE.so
lib/Sample_SphereMapping.so: /usr/lib/libX11.so
lib/Sample_SphereMapping.so: /usr/lib/libXext.so
lib/Sample_SphereMapping.so: /usr/lib/libXt.so
lib/Sample_SphereMapping.so: /usr/lib/libXaw.so
lib/Sample_SphereMapping.so: /usr/lib/libboost_thread-mt.so
lib/Sample_SphereMapping.so: /usr/lib/libboost_date_time-mt.so
lib/Sample_SphereMapping.so: /usr/lib/libfreeimage.so
lib/Sample_SphereMapping.so: /usr/lib/libzzip.so
lib/Sample_SphereMapping.so: /usr/lib/libz.so
lib/Sample_SphereMapping.so: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/build.make
lib/Sample_SphereMapping.so: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../../lib/Sample_SphereMapping.so"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sample_SphereMapping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/build: lib/Sample_SphereMapping.so
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/build

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/requires: Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/src/SphereMapping.cpp.o.requires
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/requires

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/clean:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping && $(CMAKE_COMMAND) -P CMakeFiles/Sample_SphereMapping.dir/cmake_clean.cmake
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/clean

Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/depend:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping /opt/kinect/LAB/Eclipse/TuxSimbad/Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Samples/SphereMapping/CMakeFiles/Sample_SphereMapping.dir/depend

