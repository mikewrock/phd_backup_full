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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mike/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mike/catkin_ws/build

# Include any dependencies generated for this target.
include wrock/CMakeFiles/pcl.dir/depend.make

# Include the progress variables for this target.
include wrock/CMakeFiles/pcl.dir/progress.make

# Include the compile flags for this target's objects.
include wrock/CMakeFiles/pcl.dir/flags.make

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o: wrock/CMakeFiles/pcl.dir/flags.make
wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o: /home/mike/catkin_ws/src/wrock/src/pcl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/mike/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcl.dir/src/pcl.cpp.o -c /home/mike/catkin_ws/src/wrock/src/pcl.cpp

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl.dir/src/pcl.cpp.i"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/mike/catkin_ws/src/wrock/src/pcl.cpp > CMakeFiles/pcl.dir/src/pcl.cpp.i

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl.dir/src/pcl.cpp.s"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/mike/catkin_ws/src/wrock/src/pcl.cpp -o CMakeFiles/pcl.dir/src/pcl.cpp.s

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.requires:
.PHONY : wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.requires

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.provides: wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.requires
	$(MAKE) -f wrock/CMakeFiles/pcl.dir/build.make wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.provides.build
.PHONY : wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.provides

wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.provides.build: wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o

# Object files for target pcl
pcl_OBJECTS = \
"CMakeFiles/pcl.dir/src/pcl.cpp.o"

# External object files for target pcl
pcl_EXTERNAL_OBJECTS =

/home/mike/catkin_ws/devel/lib/wrock/pcl: wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o
/home/mike/catkin_ws/devel/lib/wrock/pcl: wrock/CMakeFiles/pcl.dir/build.make
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/liburdf.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosconsole_bridge.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /home/mike/catkin_ws/devel/lib/libm5api.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libntcan.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/liblaser_geometry.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libmean.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libparams.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libincrement.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libmedian.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libtransfer_function.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libpcl_ros_filters.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libpcl_ros_io.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libpcl_ros_tf.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_common.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_octree.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_io.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_kdtree.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_search.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_sample_consensus.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_filters.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_features.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_keypoints.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_segmentation.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_visualization.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_outofcore.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_registration.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_recognition.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_surface.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_people.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_tracking.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libpcl_apps.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libOpenNI.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libvtkCommon.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libvtkRendering.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libvtkHybrid.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libvtkCharts.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libnodeletlib.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libbondcpp.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libclass_loader.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/libPocoFoundation.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libdl.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libroslib.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosbag.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosbag_storage.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libroslz4.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libtopic_tools.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libtf.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libtf2_ros.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libactionlib.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libmessage_filters.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libroscpp.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libtf2.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosconsole.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/liblog4cxx.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/librostime.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libcpp_common.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: /opt/ros/indigo/lib/libpcan.so
/home/mike/catkin_ws/devel/lib/wrock/pcl: wrock/CMakeFiles/pcl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/mike/catkin_ws/devel/lib/wrock/pcl"
	cd /home/mike/catkin_ws/build/wrock && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
wrock/CMakeFiles/pcl.dir/build: /home/mike/catkin_ws/devel/lib/wrock/pcl
.PHONY : wrock/CMakeFiles/pcl.dir/build

wrock/CMakeFiles/pcl.dir/requires: wrock/CMakeFiles/pcl.dir/src/pcl.cpp.o.requires
.PHONY : wrock/CMakeFiles/pcl.dir/requires

wrock/CMakeFiles/pcl.dir/clean:
	cd /home/mike/catkin_ws/build/wrock && $(CMAKE_COMMAND) -P CMakeFiles/pcl.dir/cmake_clean.cmake
.PHONY : wrock/CMakeFiles/pcl.dir/clean

wrock/CMakeFiles/pcl.dir/depend:
	cd /home/mike/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mike/catkin_ws/src /home/mike/catkin_ws/src/wrock /home/mike/catkin_ws/build /home/mike/catkin_ws/build/wrock /home/mike/catkin_ws/build/wrock/CMakeFiles/pcl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : wrock/CMakeFiles/pcl.dir/depend

