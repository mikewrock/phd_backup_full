# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/mike/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mike/catkin_ws/build

# Include any dependencies generated for this target.
include wrock/CMakeFiles/cube_demo.dir/depend.make

# Include the progress variables for this target.
include wrock/CMakeFiles/cube_demo.dir/progress.make

# Include the compile flags for this target's objects.
include wrock/CMakeFiles/cube_demo.dir/flags.make

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o: wrock/CMakeFiles/cube_demo.dir/flags.make
wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o: /home/mike/catkin_ws/src/wrock/src/cube_demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mike/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o -c /home/mike/catkin_ws/src/wrock/src/cube_demo.cpp

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cube_demo.dir/src/cube_demo.cpp.i"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mike/catkin_ws/src/wrock/src/cube_demo.cpp > CMakeFiles/cube_demo.dir/src/cube_demo.cpp.i

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cube_demo.dir/src/cube_demo.cpp.s"
	cd /home/mike/catkin_ws/build/wrock && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mike/catkin_ws/src/wrock/src/cube_demo.cpp -o CMakeFiles/cube_demo.dir/src/cube_demo.cpp.s

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.requires:

.PHONY : wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.requires

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.provides: wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.requires
	$(MAKE) -f wrock/CMakeFiles/cube_demo.dir/build.make wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.provides.build
.PHONY : wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.provides

wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.provides.build: wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o


# Object files for target cube_demo
cube_demo_OBJECTS = \
"CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o"

# External object files for target cube_demo
cube_demo_EXTERNAL_OBJECTS =

/home/mike/catkin_ws/devel/lib/wrock/cube_demo: wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: wrock/CMakeFiles/cube_demo.dir/build.make
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/liburdf.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosconsole_bridge.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /home/mike/catkin_ws/devel/lib/libm5api.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libntcan.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/liblaser_geometry.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libmean.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libparams.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libincrement.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libmedian.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libtransfer_function.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libpcl_ros_filters.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libpcl_ros_io.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libpcl_ros_tf.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_common.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_octree.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_io.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_kdtree.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_search.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_sample_consensus.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_filters.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_features.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_keypoints.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_segmentation.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_visualization.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_outofcore.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_registration.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_recognition.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_surface.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_people.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_tracking.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libpcl_apps.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libOpenNI.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libvtkCommon.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libvtkRendering.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libvtkHybrid.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libvtkCharts.so.5.8.0
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libnodeletlib.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libbondcpp.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libclass_loader.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/libPocoFoundation.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libdl.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libroslib.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosbag.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosbag_storage.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libroslz4.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libtopic_tools.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libtf.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libtf2_ros.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libactionlib.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libmessage_filters.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libroscpp.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libtf2.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosconsole.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/liblog4cxx.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/librostime.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libcpp_common.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: /opt/ros/indigo/lib/libpcan.so
/home/mike/catkin_ws/devel/lib/wrock/cube_demo: wrock/CMakeFiles/cube_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mike/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/mike/catkin_ws/devel/lib/wrock/cube_demo"
	cd /home/mike/catkin_ws/build/wrock && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cube_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
wrock/CMakeFiles/cube_demo.dir/build: /home/mike/catkin_ws/devel/lib/wrock/cube_demo

.PHONY : wrock/CMakeFiles/cube_demo.dir/build

wrock/CMakeFiles/cube_demo.dir/requires: wrock/CMakeFiles/cube_demo.dir/src/cube_demo.cpp.o.requires

.PHONY : wrock/CMakeFiles/cube_demo.dir/requires

wrock/CMakeFiles/cube_demo.dir/clean:
	cd /home/mike/catkin_ws/build/wrock && $(CMAKE_COMMAND) -P CMakeFiles/cube_demo.dir/cmake_clean.cmake
.PHONY : wrock/CMakeFiles/cube_demo.dir/clean

wrock/CMakeFiles/cube_demo.dir/depend:
	cd /home/mike/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mike/catkin_ws/src /home/mike/catkin_ws/src/wrock /home/mike/catkin_ws/build /home/mike/catkin_ws/build/wrock /home/mike/catkin_ws/build/wrock/CMakeFiles/cube_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : wrock/CMakeFiles/cube_demo.dir/depend

