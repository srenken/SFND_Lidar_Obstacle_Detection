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
CMAKE_SOURCE_DIR = /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build

# Include any dependencies generated for this target.
include CMakeFiles/quizCluster.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quizCluster.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quizCluster.dir/flags.make

CMakeFiles/quizCluster.dir/cluster.cpp.o: CMakeFiles/quizCluster.dir/flags.make
CMakeFiles/quizCluster.dir/cluster.cpp.o: ../cluster.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quizCluster.dir/cluster.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quizCluster.dir/cluster.cpp.o -c /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp

CMakeFiles/quizCluster.dir/cluster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quizCluster.dir/cluster.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp > CMakeFiles/quizCluster.dir/cluster.cpp.i

CMakeFiles/quizCluster.dir/cluster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quizCluster.dir/cluster.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/cluster.cpp -o CMakeFiles/quizCluster.dir/cluster.cpp.s

CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o: CMakeFiles/quizCluster.dir/flags.make
CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o: /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o -c /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp

CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp > CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.i

CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp -o CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.s

# Object files for target quizCluster
quizCluster_OBJECTS = \
"CMakeFiles/quizCluster.dir/cluster.cpp.o" \
"CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o"

# External object files for target quizCluster
quizCluster_EXTERNAL_OBJECTS =

quizCluster: CMakeFiles/quizCluster.dir/cluster.cpp.o
quizCluster: CMakeFiles/quizCluster.dir/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/render/render.cpp.o
quizCluster: CMakeFiles/quizCluster.dir/build.make
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_people.so
quizCluster: /usr/lib/x86_64-linux-gnu/libboost_system.so
quizCluster: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
quizCluster: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
quizCluster: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
quizCluster: /usr/lib/x86_64-linux-gnu/libboost_regex.so
quizCluster: /usr/lib/x86_64-linux-gnu/libqhull.so
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libfreetype.so
quizCluster: /usr/lib/x86_64-linux-gnu/libz.so
quizCluster: /usr/lib/x86_64-linux-gnu/libjpeg.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpng.so
quizCluster: /usr/lib/x86_64-linux-gnu/libtiff.so
quizCluster: /usr/lib/x86_64-linux-gnu/libexpat.so
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_features.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_search.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_io.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
quizCluster: /usr/lib/x86_64-linux-gnu/libpcl_common.so
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libfreetype.so
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
quizCluster: /usr/lib/x86_64-linux-gnu/libz.so
quizCluster: /usr/lib/x86_64-linux-gnu/libGLEW.so
quizCluster: /usr/lib/x86_64-linux-gnu/libSM.so
quizCluster: /usr/lib/x86_64-linux-gnu/libICE.so
quizCluster: /usr/lib/x86_64-linux-gnu/libX11.so
quizCluster: /usr/lib/x86_64-linux-gnu/libXext.so
quizCluster: /usr/lib/x86_64-linux-gnu/libXt.so
quizCluster: CMakeFiles/quizCluster.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable quizCluster"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quizCluster.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quizCluster.dir/build: quizCluster

.PHONY : CMakeFiles/quizCluster.dir/build

CMakeFiles/quizCluster.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quizCluster.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quizCluster.dir/clean

CMakeFiles/quizCluster.dir/depend:
	cd /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build /home/sebastian/workspace/Udacity/SFND_Lidar_Obstacle_Detection/src/quiz/cluster/build/CMakeFiles/quizCluster.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quizCluster.dir/depend

