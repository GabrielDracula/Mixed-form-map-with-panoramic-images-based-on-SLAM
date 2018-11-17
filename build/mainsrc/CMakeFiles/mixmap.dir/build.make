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
CMAKE_SOURCE_DIR = /home/tangdi/MyFiles/GraduationProject/GPcode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tangdi/MyFiles/GraduationProject/GPcode/build

# Include any dependencies generated for this target.
include mainsrc/CMakeFiles/mixmap.dir/depend.make

# Include the progress variables for this target.
include mainsrc/CMakeFiles/mixmap.dir/progress.make

# Include the compile flags for this target's objects.
include mainsrc/CMakeFiles/mixmap.dir/flags.make

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o: mainsrc/CMakeFiles/mixmap.dir/flags.make
mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o: ../mainsrc/mixmap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tangdi/MyFiles/GraduationProject/GPcode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o"
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mixmap.dir/mixmap.cpp.o -c /home/tangdi/MyFiles/GraduationProject/GPcode/mainsrc/mixmap.cpp

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mixmap.dir/mixmap.cpp.i"
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tangdi/MyFiles/GraduationProject/GPcode/mainsrc/mixmap.cpp > CMakeFiles/mixmap.dir/mixmap.cpp.i

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mixmap.dir/mixmap.cpp.s"
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tangdi/MyFiles/GraduationProject/GPcode/mainsrc/mixmap.cpp -o CMakeFiles/mixmap.dir/mixmap.cpp.s

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.requires:

.PHONY : mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.requires

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.provides: mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.requires
	$(MAKE) -f mainsrc/CMakeFiles/mixmap.dir/build.make mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.provides.build
.PHONY : mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.provides

mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.provides.build: mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o


# Object files for target mixmap
mixmap_OBJECTS = \
"CMakeFiles/mixmap.dir/mixmap.cpp.o"

# External object files for target mixmap
mixmap_EXTERNAL_OBJECTS =

../bin/mixmap: mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o
../bin/mixmap: mainsrc/CMakeFiles/mixmap.dir/build.make
../bin/mixmap: vo_core/libvocore.so
../bin/mixmap: dense_mapper/libdensemapper.so
../bin/mixmap: mainsrc/libIMGReader.a
../bin/mixmap: map2d/libmap2d.so
../bin/mixmap: /usr/local/lib/libopencv_stitching.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_superres.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_videostab.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_photo.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_aruco.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_bgsegm.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_bioinspired.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_ccalib.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_dpm.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_face.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_freetype.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_fuzzy.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_hdf.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_img_hash.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_line_descriptor.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_optflow.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_reg.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_rgbd.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_saliency.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_sfm.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_stereo.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_structured_light.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_viz.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_phase_unwrapping.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_surface_matching.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_tracking.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_datasets.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_plot.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_text.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_dnn.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_xfeatures2d.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_ml.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_shape.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_video.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_ximgproc.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_calib3d.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_features2d.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_flann.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_highgui.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_videoio.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_xobjdetect.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_imgcodecs.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_objdetect.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_xphoto.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_imgproc.so.3.3.1
../bin/mixmap: /usr/local/lib/libopencv_core.so.3.3.1
../bin/mixmap: /home/tangdi/Sophus/build/libSophus.so
../bin/mixmap: /usr/local/lib/libvtkIOMINC-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkGeovisCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkproj4-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkDomainsChemistry-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersSelection-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersParallelImaging-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersHyperTree-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingImage-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkInteractionImage-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingQt-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersTexture-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOParallel-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersParallel-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIONetCDF-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkjsoncpp-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOExport-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingGL2PS-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkgl2ps-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkGUISupportQtOpenGL-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersGeneric-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOVideo-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingStatistics-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingLOD-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingLIC-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOPLY-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersFlowPaths-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersProgrammable-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingMorphological-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkGUISupportQtWebkit-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkViewsQt-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkViewsInfovis-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkInfovisLayout-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkChartsCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersImaging-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingLabel-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkGUISupportQt-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingOpenGL-6.3.so.1
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libXt.so
../bin/mixmap: /usr/local/lib/libvtkImagingStencil-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOImport-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOAMR-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersAMR-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOInfovis-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkInfovisCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtklibxml2-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOLSDyna-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkGUISupportQtSQL-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOSQL-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtksqlite-6.3.so.1
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
../bin/mixmap: /usr/local/lib/libvtkFiltersSMP-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersVerdict-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkverdict-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkViewsContext2D-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkViewsCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkInteractionWidgets-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersHybrid-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersModeling-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingHybrid-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOImage-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkDICOMParser-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkmetaio-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkpng-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtktiff-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkjpeg-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkInteractionStyle-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingGeneral-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingSources-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingAnnotation-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingColor-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingVolume-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingContext2D-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingFreeType-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkRenderingCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonColor-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersExtraction-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersStatistics-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingFourier-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkImagingCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkalglib-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersGeometry-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersSources-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkftgl-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkfreetype-6.3.so.1
../bin/mixmap: /usr/local/lib/libGL.so
../bin/mixmap: /usr/local/lib/libvtkImagingMath-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOExodus-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersGeneral-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkFiltersCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkexoIIc-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkNetCDF_cxx-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkNetCDF-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkhdf5_hl-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkhdf5-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOParallelXML-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOXML-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOGeometry-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOXMLParser-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkexpat-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkParallelCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOLegacy-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOEnSight-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOMovie-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkIOCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonExecutionModel-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonDataModel-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonSystem-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonTransforms-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtksys-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonMisc-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonMath-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkCommonCore-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkzlib-6.3.so.1
../bin/mixmap: /usr/local/lib/libvtkoggtheora-6.3.so.1
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/mixmap: /usr/local/lib/libpcl_common.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../bin/mixmap: /usr/local/lib/libpcl_kdtree.so
../bin/mixmap: /usr/local/lib/libpcl_octree.so
../bin/mixmap: /usr/local/lib/libpcl_search.so
../bin/mixmap: /usr/local/lib/libpcl_sample_consensus.so
../bin/mixmap: /usr/local/lib/libpcl_filters.so
../bin/mixmap: /usr/local/lib/libpcl_features.so
../bin/mixmap: /usr/local/lib/libpcl_ml.so
../bin/mixmap: /usr/local/lib/libpcl_segmentation.so
../bin/mixmap: /usr/lib/libOpenNI.so
../bin/mixmap: /usr/local/lib/libpcl_io.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/mixmap: /usr/local/lib/libpcl_surface.so
../bin/mixmap: /usr/local/lib/libpcl_registration.so
../bin/mixmap: /usr/local/lib/libpcl_recognition.so
../bin/mixmap: /usr/local/lib/libpcl_keypoints.so
../bin/mixmap: /usr/local/lib/libpcl_visualization.so
../bin/mixmap: /usr/local/lib/libpcl_stereo.so
../bin/mixmap: /usr/local/lib/libpcl_tracking.so
../bin/mixmap: /usr/local/lib/libpcl_outofcore.so
../bin/mixmap: /usr/local/lib/libpcl_people.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/mixmap: /usr/local/lib/libpcl_common.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
../bin/mixmap: /usr/local/lib/libpcl_kdtree.so
../bin/mixmap: /usr/local/lib/libpcl_octree.so
../bin/mixmap: /usr/local/lib/libpcl_search.so
../bin/mixmap: /usr/local/lib/libpcl_sample_consensus.so
../bin/mixmap: /usr/local/lib/libpcl_filters.so
../bin/mixmap: /usr/local/lib/libpcl_features.so
../bin/mixmap: /usr/local/lib/libpcl_ml.so
../bin/mixmap: /usr/local/lib/libpcl_segmentation.so
../bin/mixmap: /usr/lib/libOpenNI.so
../bin/mixmap: /usr/local/lib/libpcl_io.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/mixmap: /usr/local/lib/libpcl_surface.so
../bin/mixmap: /usr/local/lib/libpcl_registration.so
../bin/mixmap: /usr/local/lib/libpcl_recognition.so
../bin/mixmap: /usr/local/lib/libpcl_keypoints.so
../bin/mixmap: /usr/local/lib/libpcl_visualization.so
../bin/mixmap: /usr/local/lib/libpcl_stereo.so
../bin/mixmap: /usr/local/lib/libpcl_tracking.so
../bin/mixmap: /usr/local/lib/libpcl_outofcore.so
../bin/mixmap: /usr/local/lib/libpcl_people.so
../bin/mixmap: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/mixmap: mainsrc/CMakeFiles/mixmap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tangdi/MyFiles/GraduationProject/GPcode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/mixmap"
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mixmap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mainsrc/CMakeFiles/mixmap.dir/build: ../bin/mixmap

.PHONY : mainsrc/CMakeFiles/mixmap.dir/build

mainsrc/CMakeFiles/mixmap.dir/requires: mainsrc/CMakeFiles/mixmap.dir/mixmap.cpp.o.requires

.PHONY : mainsrc/CMakeFiles/mixmap.dir/requires

mainsrc/CMakeFiles/mixmap.dir/clean:
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc && $(CMAKE_COMMAND) -P CMakeFiles/mixmap.dir/cmake_clean.cmake
.PHONY : mainsrc/CMakeFiles/mixmap.dir/clean

mainsrc/CMakeFiles/mixmap.dir/depend:
	cd /home/tangdi/MyFiles/GraduationProject/GPcode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tangdi/MyFiles/GraduationProject/GPcode /home/tangdi/MyFiles/GraduationProject/GPcode/mainsrc /home/tangdi/MyFiles/GraduationProject/GPcode/build /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc /home/tangdi/MyFiles/GraduationProject/GPcode/build/mainsrc/CMakeFiles/mixmap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mainsrc/CMakeFiles/mixmap.dir/depend
