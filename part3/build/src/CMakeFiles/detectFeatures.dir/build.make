# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/maoli/RGBD_tutorial/code/part3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maoli/RGBD_tutorial/code/part3/build

# Include any dependencies generated for this target.
include src/CMakeFiles/detectFeatures.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/detectFeatures.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/detectFeatures.dir/flags.make

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o: src/CMakeFiles/detectFeatures.dir/flags.make
src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o: ../src/detectFeatures.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/maoli/RGBD_tutorial/code/part3/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o"
	cd /home/maoli/RGBD_tutorial/code/part3/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o -c /home/maoli/RGBD_tutorial/code/part3/src/detectFeatures.cpp

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detectFeatures.dir/detectFeatures.cpp.i"
	cd /home/maoli/RGBD_tutorial/code/part3/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/maoli/RGBD_tutorial/code/part3/src/detectFeatures.cpp > CMakeFiles/detectFeatures.dir/detectFeatures.cpp.i

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detectFeatures.dir/detectFeatures.cpp.s"
	cd /home/maoli/RGBD_tutorial/code/part3/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/maoli/RGBD_tutorial/code/part3/src/detectFeatures.cpp -o CMakeFiles/detectFeatures.dir/detectFeatures.cpp.s

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.requires:
.PHONY : src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.requires

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.provides: src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/detectFeatures.dir/build.make src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.provides.build
.PHONY : src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.provides

src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.provides.build: src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o

# Object files for target detectFeatures
detectFeatures_OBJECTS = \
"CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o"

# External object files for target detectFeatures
detectFeatures_EXTERNAL_OBJECTS =

../bin/detectFeatures: src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o
../bin/detectFeatures: src/CMakeFiles/detectFeatures.dir/build.make
../bin/detectFeatures: ../lib/libslamBase.a
../bin/detectFeatures: /usr/local/lib/libopencv_videostab.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_video.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_ts.a
../bin/detectFeatures: /usr/local/lib/libopencv_superres.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_stitching.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_photo.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_ocl.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_objdetect.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_nonfree.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_ml.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_legacy.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_imgproc.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_highgui.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_gpu.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_flann.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_features2d.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_core.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_contrib.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_calib3d.so.2.4.13
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/detectFeatures: /usr/lib/libpcl_common.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/detectFeatures: /usr/lib/libpcl_kdtree.so
../bin/detectFeatures: /usr/lib/libpcl_octree.so
../bin/detectFeatures: /usr/lib/libpcl_search.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/detectFeatures: /usr/lib/libpcl_surface.so
../bin/detectFeatures: /usr/lib/libpcl_sample_consensus.so
../bin/detectFeatures: /usr/lib/libOpenNI.so
../bin/detectFeatures: /usr/lib/libOpenNI2.so
../bin/detectFeatures: /usr/lib/libvtkCommon.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkFiltering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkImaging.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGraphics.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkIO.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkHybrid.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkWidgets.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkParallel.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkInfovis.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGeovis.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkViews.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkCharts.so.5.8.0
../bin/detectFeatures: /usr/lib/libpcl_io.so
../bin/detectFeatures: /usr/lib/libpcl_filters.so
../bin/detectFeatures: /usr/lib/libpcl_features.so
../bin/detectFeatures: /usr/lib/libpcl_keypoints.so
../bin/detectFeatures: /usr/lib/libpcl_registration.so
../bin/detectFeatures: /usr/lib/libpcl_segmentation.so
../bin/detectFeatures: /usr/lib/libpcl_recognition.so
../bin/detectFeatures: /usr/lib/libpcl_visualization.so
../bin/detectFeatures: /usr/lib/libpcl_people.so
../bin/detectFeatures: /usr/lib/libpcl_outofcore.so
../bin/detectFeatures: /usr/lib/libpcl_tracking.so
../bin/detectFeatures: /usr/lib/libpcl_apps.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/detectFeatures: /usr/lib/libOpenNI.so
../bin/detectFeatures: /usr/lib/libOpenNI2.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/detectFeatures: /usr/lib/libvtkCommon.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkFiltering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkImaging.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGraphics.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkIO.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkHybrid.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkWidgets.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkParallel.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkInfovis.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGeovis.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkViews.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkCharts.so.5.8.0
../bin/detectFeatures: /usr/local/lib/libopencv_nonfree.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_ocl.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_gpu.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_photo.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_objdetect.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_legacy.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_video.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_ml.so.2.4.13
../bin/detectFeatures: /usr/local/cuda-8.0/lib64/libcufft.so
../bin/detectFeatures: /usr/local/lib/libopencv_calib3d.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_features2d.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_highgui.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_imgproc.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_flann.so.2.4.13
../bin/detectFeatures: /usr/local/lib/libopencv_core.so.2.4.13
../bin/detectFeatures: /usr/local/cuda-8.0/lib64/libcudart.so
../bin/detectFeatures: /usr/local/cuda-8.0/lib64/libnppc.so
../bin/detectFeatures: /usr/local/cuda-8.0/lib64/libnppi.so
../bin/detectFeatures: /usr/local/cuda-8.0/lib64/libnpps.so
../bin/detectFeatures: /usr/lib/libvtkViews.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkInfovis.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkWidgets.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkHybrid.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkParallel.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkRendering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkImaging.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkGraphics.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkIO.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkFiltering.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtkCommon.so.5.8.0
../bin/detectFeatures: /usr/lib/libvtksys.so.5.8.0
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/detectFeatures: /usr/lib/libpcl_common.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/detectFeatures: /usr/lib/libpcl_kdtree.so
../bin/detectFeatures: /usr/lib/libpcl_octree.so
../bin/detectFeatures: /usr/lib/libpcl_search.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/detectFeatures: /usr/lib/libpcl_surface.so
../bin/detectFeatures: /usr/lib/libpcl_sample_consensus.so
../bin/detectFeatures: /usr/lib/libOpenNI.so
../bin/detectFeatures: /usr/lib/libOpenNI2.so
../bin/detectFeatures: /usr/lib/libpcl_io.so
../bin/detectFeatures: /usr/lib/libpcl_filters.so
../bin/detectFeatures: /usr/lib/libpcl_features.so
../bin/detectFeatures: /usr/lib/libpcl_keypoints.so
../bin/detectFeatures: /usr/lib/libpcl_registration.so
../bin/detectFeatures: /usr/lib/libpcl_segmentation.so
../bin/detectFeatures: /usr/lib/libpcl_recognition.so
../bin/detectFeatures: /usr/lib/libpcl_visualization.so
../bin/detectFeatures: /usr/lib/libpcl_people.so
../bin/detectFeatures: /usr/lib/libpcl_outofcore.so
../bin/detectFeatures: /usr/lib/libpcl_tracking.so
../bin/detectFeatures: /usr/lib/libpcl_apps.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/detectFeatures: /usr/lib/libpcl_common.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/detectFeatures: /usr/lib/libpcl_kdtree.so
../bin/detectFeatures: /usr/lib/libpcl_octree.so
../bin/detectFeatures: /usr/lib/libpcl_search.so
../bin/detectFeatures: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/detectFeatures: /usr/lib/libpcl_surface.so
../bin/detectFeatures: /usr/lib/libpcl_sample_consensus.so
../bin/detectFeatures: /usr/lib/libOpenNI.so
../bin/detectFeatures: /usr/lib/libOpenNI2.so
../bin/detectFeatures: /usr/lib/libpcl_io.so
../bin/detectFeatures: /usr/lib/libpcl_filters.so
../bin/detectFeatures: /usr/lib/libpcl_features.so
../bin/detectFeatures: /usr/lib/libpcl_keypoints.so
../bin/detectFeatures: /usr/lib/libpcl_registration.so
../bin/detectFeatures: /usr/lib/libpcl_segmentation.so
../bin/detectFeatures: /usr/lib/libpcl_recognition.so
../bin/detectFeatures: /usr/lib/libpcl_visualization.so
../bin/detectFeatures: /usr/lib/libpcl_people.so
../bin/detectFeatures: /usr/lib/libpcl_outofcore.so
../bin/detectFeatures: /usr/lib/libpcl_tracking.so
../bin/detectFeatures: /usr/lib/libpcl_apps.so
../bin/detectFeatures: src/CMakeFiles/detectFeatures.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/detectFeatures"
	cd /home/maoli/RGBD_tutorial/code/part3/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detectFeatures.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/detectFeatures.dir/build: ../bin/detectFeatures
.PHONY : src/CMakeFiles/detectFeatures.dir/build

src/CMakeFiles/detectFeatures.dir/requires: src/CMakeFiles/detectFeatures.dir/detectFeatures.cpp.o.requires
.PHONY : src/CMakeFiles/detectFeatures.dir/requires

src/CMakeFiles/detectFeatures.dir/clean:
	cd /home/maoli/RGBD_tutorial/code/part3/build/src && $(CMAKE_COMMAND) -P CMakeFiles/detectFeatures.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/detectFeatures.dir/clean

src/CMakeFiles/detectFeatures.dir/depend:
	cd /home/maoli/RGBD_tutorial/code/part3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maoli/RGBD_tutorial/code/part3 /home/maoli/RGBD_tutorial/code/part3/src /home/maoli/RGBD_tutorial/code/part3/build /home/maoli/RGBD_tutorial/code/part3/build/src /home/maoli/RGBD_tutorial/code/part3/build/src/CMakeFiles/detectFeatures.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/detectFeatures.dir/depend

