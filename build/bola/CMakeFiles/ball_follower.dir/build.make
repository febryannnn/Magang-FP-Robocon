# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/farelfebryan/finalproject_ws/src/bola

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/farelfebryan/finalproject_ws/build/bola

# Include any dependencies generated for this target.
include CMakeFiles/ball_follower.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ball_follower.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ball_follower.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ball_follower.dir/flags.make

CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o: CMakeFiles/ball_follower.dir/flags.make
CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o: /home/farelfebryan/finalproject_ws/src/bola/src/deteksibola.cpp
CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o: CMakeFiles/ball_follower.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/farelfebryan/finalproject_ws/build/bola/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o -MF CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o.d -o CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o -c /home/farelfebryan/finalproject_ws/src/bola/src/deteksibola.cpp

CMakeFiles/ball_follower.dir/src/deteksibola.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ball_follower.dir/src/deteksibola.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/farelfebryan/finalproject_ws/src/bola/src/deteksibola.cpp > CMakeFiles/ball_follower.dir/src/deteksibola.cpp.i

CMakeFiles/ball_follower.dir/src/deteksibola.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ball_follower.dir/src/deteksibola.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/farelfebryan/finalproject_ws/src/bola/src/deteksibola.cpp -o CMakeFiles/ball_follower.dir/src/deteksibola.cpp.s

# Object files for target ball_follower
ball_follower_OBJECTS = \
"CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o"

# External object files for target ball_follower
ball_follower_EXTERNAL_OBJECTS =

ball_follower: CMakeFiles/ball_follower.dir/src/deteksibola.cpp.o
ball_follower: CMakeFiles/ball_follower.dir/build.make
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
ball_follower: /opt/ros/humble/lib/librclcpp.so
ball_follower: /opt/ros/humble/lib/libcv_bridge.so
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
ball_follower: /opt/ros/humble/lib/liblibstatistics_collector.so
ball_follower: /opt/ros/humble/lib/librcl.so
ball_follower: /opt/ros/humble/lib/librmw_implementation.so
ball_follower: /opt/ros/humble/lib/libament_index_cpp.so
ball_follower: /opt/ros/humble/lib/librcl_logging_spdlog.so
ball_follower: /opt/ros/humble/lib/librcl_logging_interface.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/librcl_yaml_param_parser.so
ball_follower: /opt/ros/humble/lib/libyaml.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/libtracetools.so
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
ball_follower: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
ball_follower: /opt/ros/humble/lib/libfastcdr.so.1.0.24
ball_follower: /opt/ros/humble/lib/librmw.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
ball_follower: /usr/lib/x86_64-linux-gnu/libpython3.10.so
ball_follower: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
ball_follower: /opt/ros/humble/lib/librosidl_typesupport_c.so
ball_follower: /opt/ros/humble/lib/librosidl_runtime_c.so
ball_follower: /opt/ros/humble/lib/librcpputils.so
ball_follower: /opt/ros/humble/lib/librcutils.so
ball_follower: CMakeFiles/ball_follower.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/farelfebryan/finalproject_ws/build/bola/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ball_follower"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ball_follower.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ball_follower.dir/build: ball_follower
.PHONY : CMakeFiles/ball_follower.dir/build

CMakeFiles/ball_follower.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ball_follower.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ball_follower.dir/clean

CMakeFiles/ball_follower.dir/depend:
	cd /home/farelfebryan/finalproject_ws/build/bola && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/farelfebryan/finalproject_ws/src/bola /home/farelfebryan/finalproject_ws/src/bola /home/farelfebryan/finalproject_ws/build/bola /home/farelfebryan/finalproject_ws/build/bola /home/farelfebryan/finalproject_ws/build/bola/CMakeFiles/ball_follower.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ball_follower.dir/depend

