# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build"

# Include any dependencies generated for this target.
include gui/CMakeFiles/Gui.dir/depend.make

# Include the progress variables for this target.
include gui/CMakeFiles/Gui.dir/progress.make

# Include the compile flags for this target's objects.
include gui/CMakeFiles/Gui.dir/flags.make

gui/CMakeFiles/Gui.dir/gui.cpp.o: gui/CMakeFiles/Gui.dir/flags.make
gui/CMakeFiles/Gui.dir/gui.cpp.o: ../gui/gui.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gui/CMakeFiles/Gui.dir/gui.cpp.o"
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Gui.dir/gui.cpp.o -c "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/gui/gui.cpp"

gui/CMakeFiles/Gui.dir/gui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Gui.dir/gui.cpp.i"
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/gui/gui.cpp" > CMakeFiles/Gui.dir/gui.cpp.i

gui/CMakeFiles/Gui.dir/gui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Gui.dir/gui.cpp.s"
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/gui/gui.cpp" -o CMakeFiles/Gui.dir/gui.cpp.s

gui/CMakeFiles/Gui.dir/gui.cpp.o.requires:

.PHONY : gui/CMakeFiles/Gui.dir/gui.cpp.o.requires

gui/CMakeFiles/Gui.dir/gui.cpp.o.provides: gui/CMakeFiles/Gui.dir/gui.cpp.o.requires
	$(MAKE) -f gui/CMakeFiles/Gui.dir/build.make gui/CMakeFiles/Gui.dir/gui.cpp.o.provides.build
.PHONY : gui/CMakeFiles/Gui.dir/gui.cpp.o.provides

gui/CMakeFiles/Gui.dir/gui.cpp.o.provides.build: gui/CMakeFiles/Gui.dir/gui.cpp.o


# Object files for target Gui
Gui_OBJECTS = \
"CMakeFiles/Gui.dir/gui.cpp.o"

# External object files for target Gui
Gui_EXTERNAL_OBJECTS =

gui/libGui.a: gui/CMakeFiles/Gui.dir/gui.cpp.o
gui/libGui.a: gui/CMakeFiles/Gui.dir/build.make
gui/libGui.a: gui/CMakeFiles/Gui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libGui.a"
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && $(CMAKE_COMMAND) -P CMakeFiles/Gui.dir/cmake_clean_target.cmake
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Gui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gui/CMakeFiles/Gui.dir/build: gui/libGui.a

.PHONY : gui/CMakeFiles/Gui.dir/build

gui/CMakeFiles/Gui.dir/requires: gui/CMakeFiles/Gui.dir/gui.cpp.o.requires

.PHONY : gui/CMakeFiles/Gui.dir/requires

gui/CMakeFiles/Gui.dir/clean:
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" && $(CMAKE_COMMAND) -P CMakeFiles/Gui.dir/cmake_clean.cmake
.PHONY : gui/CMakeFiles/Gui.dir/clean

gui/CMakeFiles/Gui.dir/depend:
	cd "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf" "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/gui" "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build" "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui" "/home/stepan/Google Drive/Kastanovo/Škola/Mag/3. sem/MKR/SEM03/mkr2016_pf/build/gui/CMakeFiles/Gui.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : gui/CMakeFiles/Gui.dir/depend

