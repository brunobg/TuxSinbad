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
include Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/depend.make

# Include the progress variables for this target.
include Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/progress.make

# Include the compile flags for this target's objects.
include Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o: Tools/XMLConverter/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/main.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/main.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/main.cpp > CMakeFiles/OgreXMLConverter.dir/src/main.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/main.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/main.cpp -o CMakeFiles/OgreXMLConverter.dir/src/main.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o: Tools/XMLConverter/src/OgreXMLMeshSerializer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLMeshSerializer.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLMeshSerializer.cpp > CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLMeshSerializer.cpp -o CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o: Tools/XMLConverter/src/OgreXMLSkeletonSerializer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLSkeletonSerializer.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLSkeletonSerializer.cpp > CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/OgreXMLSkeletonSerializer.cpp -o CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o: Tools/XMLConverter/src/tinystr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinystr.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinystr.cpp > CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinystr.cpp -o CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o: Tools/XMLConverter/src/tinyxml.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxml.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxml.cpp > CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxml.cpp -o CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o: Tools/XMLConverter/src/tinyxmlerror.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlerror.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlerror.cpp > CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlerror.cpp -o CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.provides.build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/flags.make
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o: Tools/XMLConverter/src/tinyxmlparser.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kinect/LAB/Eclipse/TuxSimbad/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o -c /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlparser.cpp

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.i"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlparser.cpp > CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.i

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.s"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/src/tinyxmlparser.cpp -o CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.s

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.requires:
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.provides: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.requires
	$(MAKE) -f Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.provides.build
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.provides

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.provides.build: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.provides.build

# Object files for target OgreXMLConverter
OgreXMLConverter_OBJECTS = \
"CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o" \
"CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o"

# External object files for target OgreXMLConverter
OgreXMLConverter_EXTERNAL_OBJECTS =

bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o
bin/OgreXMLConverter: lib/libOgreMain.so.1.7.2
bin/OgreXMLConverter: /usr/lib/libfreetype.so
bin/OgreXMLConverter: /usr/lib/libSM.so
bin/OgreXMLConverter: /usr/lib/libICE.so
bin/OgreXMLConverter: /usr/lib/libX11.so
bin/OgreXMLConverter: /usr/lib/libXext.so
bin/OgreXMLConverter: /usr/lib/libXt.so
bin/OgreXMLConverter: /usr/lib/libXaw.so
bin/OgreXMLConverter: /usr/lib/libboost_thread-mt.so
bin/OgreXMLConverter: /usr/lib/libboost_date_time-mt.so
bin/OgreXMLConverter: /usr/lib/libfreeimage.so
bin/OgreXMLConverter: /usr/lib/libzzip.so
bin/OgreXMLConverter: /usr/lib/libz.so
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build.make
bin/OgreXMLConverter: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/OgreXMLConverter"
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OgreXMLConverter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build: bin/OgreXMLConverter
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/build

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/main.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLMeshSerializer.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/OgreXMLSkeletonSerializer.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinystr.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxml.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlerror.cpp.o.requires
Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires: Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/src/tinyxmlparser.cpp.o.requires
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/requires

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/clean:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter && $(CMAKE_COMMAND) -P CMakeFiles/OgreXMLConverter.dir/cmake_clean.cmake
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/clean

Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/depend:
	cd /opt/kinect/LAB/Eclipse/TuxSimbad && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter /opt/kinect/LAB/Eclipse/TuxSimbad /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter /opt/kinect/LAB/Eclipse/TuxSimbad/Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Tools/XMLConverter/CMakeFiles/OgreXMLConverter.dir/depend

