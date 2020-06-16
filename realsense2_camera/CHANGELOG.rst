^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.7 (2020-06-16)
------------------
* Merge branch 'increase-temp-diagnostics' into 'ferrum-devel'
  Increase temperature diagnostics so they don't appear as stale
  See merge request ros-overlays/realsense!21
* Increase temperature diagnostics so they don't appear as stale
* Contributors: Victor Lopez, procopiostein

3.1.6 (2020-05-29)
------------------
* Merge branch 'revert-upstream-update' into 'ferrum-devel'
  Revert "Merge branch 'update-upstream' into 'erbium-devel'"
  See merge request ros-overlays/realsense!20
* Revert "Merge branch 'update-upstream' into 'erbium-devel'"
  This reverts merge request !17
* Contributors: procopiostein

3.1.5 (2020-05-18)
------------------
* Merge branch 'update-upstream' into 'erbium-devel'
  Update upstream
  See merge request ros-overlays/realsense!17
* Merge pull request #1112 from doronhi/catch_device
  catch failure on device creation.
* Merge pull request #1095 from doronhi/join_query_thread
  Join query thread
* Merge branch 'support_L515' into development
* Merge branch 'optimize_pointcloud' into development
* Merge pull request #1084 from doronhi/release_build
  fix bug: enable building release version instead of debug.
* Merge pull request #1088 from doronhi/usb_port_id
  update parsing port_id
* Merge pull request #802 from ahoarau/patch-1
  add missing eigen dependency
* Merge branch 'tykurtz-rename-frames-urdf' into development
* Merge pull request #1010 from doronhi/fix_imu_copy
  fixed unite_imu_method copy to transmit at gyro's rate.
* fix point cloud test to work with outdorrs_1color.bag file.
* Merge pull request #993 from sahandy/bugfix/wrong-rgb-datatype-in-ros-msg
  Fix wrong PointField Type for Colored Point Cloud Messages
* Contributors: doronhi, procopiostein

3.1.4 (2020-03-25)
------------------
* Revert "compatible way of disabling pal flags"
  This reverts commit 4e17d45bb4c8dfb2ac1d7fad3beb481d5e74afd9.
* Contributors: Procópio Stein

3.1.3 (2020-03-25)
------------------
* compatible way of disabling pal flags
* hack to reduce pointcloud publishing frequency
* Contributors: Procópio Stein, federiconardi

3.1.2 (2020-02-18)
------------------

3.1.1 (2020-02-05)
------------------
* Merge branch 'torso-pointcloud-fix' into 'erbium-devel'
  Changed default value of allow_no_texture_points to true
  See merge request ros-overlays/realsense!11
* Changed default value of allow_no_texture_points to true
* Contributors: Sai Kishor Kothakota, Sara Cooper

3.1.0 (2020-01-30)
------------------

3.0.2 (2020-01-07)
------------------

3.0.1 (2019-12-16)
------------------
* Merge branch 't265_fisheyes_baseline' into 'erbium-devel'
  Adding the baseline translation in the projection matrix of the fisheye on the right, the fisheye2
  See merge request ros-overlays/realsense!8
* Adding the baseline translation in the projection matrix of the fisheye on the right, the fisheye2
* Contributors: Victor Lopez, alessandrodifava

3.0.0 (2019-12-09)
------------------
* Added missing eigen build dependency
* added disable_pal_flags() to CMakeLists.txt
* update package dependencies in package.xml
* add compilation fixes
* Merge pull request #973 from clearpathrobotics/fix-usb-port-regex
  Fix USB port regex to work with hub-connected cams.
* Fix USB port regex to work with hub-connected cams.
* upgrade version: 2.2.9
* Merge pull request #967 from doronhi/offline_serial_no
  set serial number in diagnostics when running from bag file
* set serial number in diagnostics when running from bag file
* Merge pull request #965 from aswinkumar2019/patch-1
  Update opensource_tracking.launch
* Merge pull request #966 from doronhi/rename_usb_port_id
  Rename usb port
* Update opensource_tracking.launch
  Fixed a wrong parameter in imufiltermadgwick
* renamed porn_no to usb_port_id
* Merge pull request #964 from doronhi/BriceRenaudeau-PortNumberConnection
  Allow to use usb port number to connect to devices (continue from Brice renaudeau)
* Add device_type option to choose the device type using regular expression.
* use regex to parse usb description string.
  handle cases where failed to extract usb bus and port.
* Allow to use usb port number to connect to devices
* Merge pull request #924 from doronhi/fix_W10
  force infrared stream to choose stable Y8 format.
* Merge branch 'Blindnology-dynamic_tf' into development
* add publish_tf and tf_publish_rate to README.md
  fixed print.
* force infrared stream to choose stable Y8 format.
* Merge pull request #804 from Auterion/enable-required-tag
  enable required tag when launching node
* Add dynamic transform support again.
  https://github.com/intel-ros/realsense/pull/169
  https://github.com/intel-ros/realsense/issues/660
* Add option to disable odom tf
* Merge pull request #890 from kamilritz/t265_publishOdomTf_option
  Add option to disable odom tf to be published
* Add option to disable odom tf to be published
  This is useful if you want to link your camera to your existing robot tf tree.
  tf can not handle two parents for the camera_pose_frame
* Merge pull request #888 from doronhi/fix_monitor_crash
  fix issue: node crashes when device disconnects.
* fix issue: node crash when device disconnects.
* upgrade version: 2.2.8
* Check temperature freqency (#884)
  Add the ability to set auto_exposure ROI.
  Available using dynamic reconfigure parameters or by setting the following to the launch file:
  <rosparam>
  /camera/rgb_camera/auto_exposure_roi/right: 250
  /camera/stereo_module/auto_exposure_roi/top: 50
  </rosparam>
  * update temperature checks to once every 10 seconds.
* set auto_exposure ROI. (#883)
  * Add the ability to set auto_exposure ROI.
  Available using dynamic reconfigure parameters or by setting the following to the launch file:
  <rosparam>
  /camera/rgb_camera/auto_exposure_roi/right: 250
  /camera/stereo_module/auto_exposure_roi/top: 50
  </rosparam>
* add asic temperature and projector temperature to diagnostics topic. (#878)
  * add asic temperature and projector temperature to diagnostics topic.
  * fix bug relating reading from file.
* fix bug: filters do not show in rqt_reconfigure. (#858)
* update version to 2.2.7
* Add support for SR305 (#844)
  fix bug in scaling depth.
* move ddynamic_reconfigure settings to before start streaming. (#843)
* publish imu_info (#809)
  * publish imu_info
  * update apt-key for ROS repos.
  * Add option to use external nodelet manager
  * Register dynamic options in parent node handle
  * Fix empty cmake variable and dependency on librealsense (#747)
  * add external_manager option to rs_camera.launch
  * clarify installation process.
  * publish gyro/imu_info and accel/imu_info only once.
  * cleaning
* fixed bug: advertise unsupported streams. (#811)
  * fixed bug: advertise unsupported streams.
  * constraints on accel_up test - loosen up a bit to compensate for different in frames arriving from file.
* cleaning
* Make T265 less noisy in the info log (#823)
* publish gyro/imu_info and accel/imu_info only once.
* add external_manager option to rs_camera.launch
* Fix empty cmake variable and dependency on librealsense (#747)
* Register dynamic options in parent node handle
  If running in an external nodelet manager this makes sure dynamic options are in the right namespace. This does not change existing behaviour.
* Add option to use external nodelet manager
  Right now a nodelet manager is always created when running the realsense
  node. This is not always a wanted behaviour as a nodelet manager might
  already be running somewhere else in the ROS system. This patch adds a
  parameter to disable launching a new nodelet manager. In this case the
  nodelet manager must be provided using the manager arg. The new
  parameter is set to false by default, so existing behaviour is not
  changed.
* enable reaquire tag whan launching node
* Merge branch 'stsundermann-development' into development
* add external_manager option to rs_camera.launch
* Merge branch 'development' of https://github.com/stsundermann/realsense into stsundermann-development
* Fix empty cmake variable and dependency on librealsense (#747)
* publish imu_info
* rename librealsense2.rdmanifest to librealsense2_xenial.rdmanifest
  add file: librealsense2_bionic.rdmanifest
* disable static_tf_1 test - not working on Travis.
  Modified points_cloud_1 test - make more robust
* add librealsense2 dependency
* update version to 2.2.6
* add librealsense2.rdmanifest file
* fix remarks to imu test
* Add accel test
  * add rs_rtabmap.launch
  * Add test for accel in d435i. Needs recorded file: 20190527_D435i.bag
  * add d435i accel unit-test
* update version to 2.2.5
* exit node if failed to initialize.
* Merge branch 'athackst-feature/realsense2_description' into development
* Merge branch 'feature/realsense2_description' of https://github.com/athackst/realsense into athackst-feature/realsense2_description
* Merge branch 'pal-robotics-forks-development' into development
* Migrate to https://github.com/pal-robotics/ddynamic_reconfigure
* fixed bug: wrong frame_id for imu frames. (#784)
* update version: 2.2.4
* add option: allow_no_texture_points
* add note to rs_rgbd.launch, reminding users to initially install ros package rgbd_launch.
* Merge branch 'fix_t265_coordinates' into development
* removed global variable _device, based on @akirayou at https://github.com/IntelRealSense/realsense-ros/issues/774#issuecomment-494236047
* Merge branch 'dense_pointcloud' into development
* Merge branch 'abhijitmajumdar-development' into development
* Merge branch 'development' of https://github.com/abhijitmajumdar/realsense into abhijitmajumdar-development
* fixed bug: imu and synced imu are now sent in original device coordinates frames - i.e. gyro_optical_frame, accel_optical_frame, imu_optical_frame. Fix issue for both t265 and d435i with different coordinate systems.
  fixed bug: sending united imu without images enabled.
  add imu_optical_frame_id to nodelet.launch.xml.
* camera_link for t265 is POSE instead of GYRO.
  fix is needed due to the availability of t265 extrinsics.
* fix inserted bug reading from file
* removed lock_guard.
  set_devices_changed_callback called AFTER getDevice()
  Keep checking for devices until device is found - for cases where T265 was momentarily taken by another node at the time of query.
  Add a 3rd, optional camera, to rs_multiple_devices.launch file.
* fix bug in pointcloud. Used to send points with Z=0.
  add feature: _allow_no_texture_points - if set to true, will send points with depth, both with and without texture.
* Merge pull request #752 from schmidtp1/sync-get-device
  sync get devices
* sync get devices
* add decimation filter at the front of the filter list, before the start of disparity filter
* Migrate to https://github.com/pal-robotics/ddynamic_reconfigure
* fix bug scaling depth. (#717)
* change frame_id for imu messages to camera_link's coordinates system, same as imu's sync messages.
* Add hole_filling filter.
  update version to 2.2.3
* update version to 2.2.2
  update README.md link to librealsense v2.19.2
* Merge branch 'disable_odom_tf' into development
* wheel_odometry (#691)
  * use wheel_Odometry
  Add parameters to launch files:
  * topic_odom_in - The topic on which wheel odometry arrives.
  * calib_odom_file - path to calibration.json file, of the librealsense format. i.e.: https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json
* Register dynamic options in parent node handle
  If running in an external nodelet manager this makes sure dynamic options are in the right namespace. This does not change existing behaviour.
* Add option to use external nodelet manager
  Right now a nodelet manager is always created when running the realsense
  node. This is not always a wanted behaviour as a nodelet manager might
  already be running somewhere else in the ROS system. This patch adds a
  parameter to disable launching a new nodelet manager. In this case the
  nodelet manager must be provided using the manager arg. The new
  parameter is set to false by default, so existing behaviour is not
  changed.
* add flag publish_odom_tf (default to true)
* fix compilation bug with OMP (#692)
* Merge branch 'efernandez-set_defaults_from_param_server' into development
* updated references to realsense2_description
* moved description files into realsense2_description package
* Cast min, step, step to int in get_enum_method
  so we don't loop with float values.
* Fix validation check for enum options
  by finding the ROS (static) param in the enum dictionary, which can take
  values in the min:step:max range (not just 0 <= value < enum_dict.size()).
  Also remove the check when the option is taken from the sensor, which
  should always be correct.
* Transform to lower in create_graph_resource_name
* Use std::replace_if in create_graph_resource_name
  with equivalent to ros::names::isValidCharInName(char c)
* Set config defaults from ROS param server
  fixes #609
  Only if the ROS params are set
* Fixed distortion coefficients bug. (#662)
* Add dependency on nav_msgs (#674)
  thanks.
* fix Twist in odometry problem (#676)
* fix PR#682 (#683)
  * add example for checking the depth at the center of the image.
  * fix bug: did not fix depth scale for single frame.
* fix depth scale (#682)
  * fix depth scale to always follow ROS convention of 1mm
  * incorporates PR#605
* Adjust unit of SR300's depth image into 0.001 meter (same as D435's). This unit adjustment is needed for rgbd_launch package and it's point cloud value.
* update version - 2.2.1
* Merge branch 'ibaranov-cp-development' into development
* Merge branch 'development' of https://github.com/ibaranov-cp/realsense into ibaranov-cp-development
* Add handling t265 coordinate system (#657)
  * fixed launch files (fisheye1,2)
  * renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* :
  [Problem]
  [Solution]
  [Test]
  [Links]
  https://issues.labcollab.net/browse/
* fixed launch files (fisheye1,2)
* Merge branch 'bfulkers-i-update-readme' into development
* add Notice to README.md and rs_t265.launch
* Merge branch 'doronhi-reconnect2' into development
* Merge branch 'reconnect2' of https://github.com/doronhi/realsense into doronhi-reconnect2
  # Conflicts:
  #	README.md
  #	realsense2_camera/launch/includes/nodelet.launch.xml
  #	realsense2_camera/launch/rs_camera.launch
  #	realsense2_camera/launch/rs_d400_and_t265.launch
  #	realsense2_camera/launch/rs_t265.launch
  #	realsense2_camera/scripts/rs2_test.py
  #	realsense2_camera/src/realsense_node_factory.cpp
* Fix version in package.xml (#625)
* Modified the CMake file so that URDF and mesh files will be installed (#615)
* Fix #628 - added guards around clang-specific pragmas (#630)
  Also added a guard around an OpenMP pragma
* fix rs_aligned_depth.launch
* increase rs2_test.py robustness for node failing to load.
* fix README.md and launch files.
* auto reset if need to.
* fix README.md and launch files.
* restore initial_reset option.
  Fix bug of locking tracking module (t265) by nodes that don't use it.
* modify behavior: if reconnect if camera disconnected.
  package.xml: upgrade package format
  removed initial_reset option - need to return.
* rename tm2 to t265
* fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
* delete topics of aligned depth to index 2 of other sensors. (#644)
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* delete topics of aligned depth to index 2 of other sensors.
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* rs_t265.launch: Add a disclaimer about wheel odometry
* renames and readme (#629)
  * fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
  * rename tm2 to t265
  * fix README.md
* check build with librealsense v2.18.1
* update version to 2.2.0
* Merge remote-tracking branch 'doronhi/tm2' into development
  # Conflicts:
  #	realsense2_camera/src/base_realsense_node.cpp
* use tf2 instead of tf for pose static transformation
* Fix pending messages variable name typo (#608)
* Replace spaces and hyphens in parameter names (#617)
* fix dependency between covariance values and confidence value.
  Added to README.md
* fix test. remove some log messages.
* fix frame_id for odom topic.
* TM265 - add odometry topic
  interface change: add parameter: enable_tm2 - cause the wrapper to wait on initialization while tm2 device sets its Unique USB ID
  use enable_gyro and enable_accel instead of enable_imu
  use infra_width, infra_fps instead of infra1_width, infra1_fps and infra2_width, infra2_fps
* add basic support for TM265. Fisheye, Gyro, Accel.
* code reorganization.
  fix bug of reinitializing align operator.
* add support for TM1 fisheye comes in RAW8 and Tm2's in Y8.
  moved enabling HID sensors to enable_devices()
* clean parameters reading.
* set base time on first message (image or imu originated)
  clean code.
* Remove gencfg dependency (#581)
  Now with ddynamic_reconfigure being the backend for dynamic reconfigurability, the ${PROJECT_NAME}_gencfg target doesn't exist anymore and this dependency can be removed.
* fix bug: "No stream match for pointcloud chosen texture" warning was meant to appear when unavailable texture is chosen. As it was, it appears every time a frame was dropped. (#591)
* Remove REQUIRED from find_package to show the correct error message (#592)
* Add filters argument to rs_rgbd.launch (#593)
* No depth required (#601)
  * add benchmark test for static_tf
  * enable running with depth disabled.
  rs2_test.py: Add message to results summery.
* fix bug: no default covariance for separate gyro and accel imu messages. (#600)
* update version to 2.1.4
* fix bug: update camera_info if image size changes. (#587)
* changed the default gyro_fps and accel_fps to match actual values (#560)
* Merge branch 'RhysMcK-development' into development
* Merge branch 'development' of https://github.com/RhysMcK/realsense into RhysMcK-development
* add initial_reset to camera2 in rs_multiple_devices.launch
* fixed urdf.rviz to look nicer.
* Merge branch 'atyshka-development' into development
* fix transform between urdf and driver
* correctred .stl filename
* added realsense D415 urdf
* Fixed d435 collision position
* add bottom_screw joint to _d435.urdf.xacro
* Merge branch 'development' of https://github.com/atyshka/realsense into atyshka-development
* add initial_reset option to rs_multiple_devices.launch
* Merge remote-tracking branch 'origin/development' into development
* fix bug in align depth to image. (#572)
  When publishFrame is called from publishAlignedDepthToOthers the format of the images is already set and is different from what is defined in _image_format for that stream type.
* close sensors when Ctrl-C signal is received. (#571)
  add test in makefile for librealsense version
* Fixed different transforms between xacro and driver
* update version number
* add linear interpolation union method for IMU (#558)
  Add linear interpolation method for union of IMU sensors. Thanks to Marius Fehr (@mfehr) for the idea.
  Set the initial behavior to sending IMU sensors separately, since this is the raw data. Enabling union with option unite_imu_method as demonstrated in the file opensource_tracking.launch.
  fix bug if initializing with unavailable imu profile.
* fix to work with librealsense v2.17.0 (#555)
  fixed to work with librealsense v2.17.0
* fix: wrong reference for the gmock dependency (#546)
  fix: typo on ddynamic_reconfigure
* Add notifications for hardware errors.
* add parameter "initial_reset" to reset the device on start up. Default is set to false.
* Merge branch 'yycho0108-development' into development
* Merge branch 'development' of https://github.com/yycho0108/realsense into yycho0108-development
* Fixed: invalid module name format for ROS (#537)
* use ddynamic_reconfigure and support D435i (#535)
  Add dynamic dynamic reconfigure. That means there are no longer differences in the code between D415, D430, SR300.
  Add dynamic options for filters
  Add support for camera D435i.
  Add clipping_disance option. enabled with parameter: clip_distance. units: meters. Default: no clipping.
  Add linear accel covariance - Default: 0.01
  Add option: unite_imu - send linear acceleration and radial velocity in the same Imu message. Default: True
  Add parameter: hold_back_imu_for_frames. If set to true, hold imu messages that arrived while manipulating frames, until frames are actually sent.
  Comply with librealsense v2.17.0
  Add opensource_tracking.launch - demo that runs realsense2_camera, imu_filter_madgwick, rtabmap and robot_localization to demonstrate Slam with realsense D435i
  Set accel_fps to 250 as this is the new maximal rate in librealsense v2.17.0
  * Add NOTICE file, to emphasize the contribution of the ddynamic_reconfigure project.
  Known Issue: Option for toggling sensor on and off while running is missing.
* Update constants.h
  update version to 2.1.2
* Potential Fix for librealsense2 v2.17.0 Compatilbility (#523)
  Fix to comply with librealsense v2.17.0.
  Thanks @m-price-softwearinc
* add log info - when dynamic reconfiguration is done.
* revert PR #490: rgbd_launch file is a running example for using the rgbd module. No need to add elements to installation for all users.
* add disparity processing.
  move colorizer to the back of the filters pipeline.
* add disparity processing
  moved colorizer filter to the end of filters pipeline.
* add decimation filter (#504)
  * add decimation filter. enable with filters:=decimation
  * fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* add decimation filter. enable with filters:=decimation
* update version to 2.1.1
* start working on decimation filter
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* add filters option to rs_aligned_depth.launch
* fix all sensors.
* fix bug: depth_auto_exposure was override in initialization by depth_exposure.
  fix bug: error in setting a parameter stop setting all other parameters.
* added missing dependencies: rgbd_launch (#490)
* Merge branch 'fork_development' into development
* fix bug: Initial dynamic configuration was stopped by starting an already started sensors. While this may not be the best practice, it's not doing any wrong and setting parameters to their default values should continue.
* fix issue: depth is being sent incorrectly if pointcloud is being sent. (#498)
  * add test for depth and aligned_depth_to_infra1.
  * fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
  * use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
  * add test depth_w_cloud_1 according to issue #491.
  * fix bug: depth_frame is not sent if pointcloud is on.
* fix bug: depth_frame is not sent if pointcloud is on.
* add test depth_w_cloud_1 according to issue #491. Fails.
* use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
* fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
* Merge branch 'development' with fix for aligned depth bug into fork_development with matching test.
* add test for depth and aligned_depth_to_infra1. The last one is knowingly fails.
* fix bug aligning depth to images
* Merge pull request #483 from shuntaraw/fix_tf_prefix
  Set tf_prefix in demo_pointcloud.launch
* Merge branch 'AndyZe-development' into development
* Merge branch 'development' of https://github.com/AndyZe/realsense into AndyZe-development
* base_realsense_node.cpp: fix typo.
* set_cams_transforms.py: fix bugs.
* add set_cams_transforms.py to add transformation between cameras.
* Pausing sensors with sens.stop(). Saves about 9% CPU load on useless processing.
* Merge remote-tracking branch 'intel/development' into development
* Adding a dynamic_reconfigure option to toggle ROS publication (issue #477).
* Set tf_prefix in demo_pointcloud.launch
* Merge pull request #482 from doronhi/development
  Add support for post processing filters
* Merge branch 'development' into development
* filters applied in given order.
  add spatial and temporal filters.
  pointcloud can be activated as a type of filter (also, still, with flag enable_pointcloud)
* fix build warning.
* modify test for pointcloud because of known bug in setting texture for pointcloud of 1st frame.
  New pointcloud does not put background color so values of test have changed.
* fix image size in pointcloud test.
* Merge branch 'baumanta-multi_cam' into development
* Change default names for frames to the same name specified for the camera topics
* new launch parameter for frame distinction in multi camera use
* enable filter colorizer.
  Issue: Can not send both pointcloud and colorized depth image at the same time.
* working pointcloud by filter. need to clean.
* Start adding filters.
  pointcloud is now implemented with filter.
  BUG: Not transmitting texture.
* add test for PointCloud2 in topic /camera/depth/color/points
* Start working on version 2.1.0 - enabling filters.
* Start working on version 2.1.0 - enabling filters.
* removed unnecessary device query (artifact from merge)
* fixed merge conflict while retaining hardware reset during initialization; added exec_depends to rgbd_launch
* Merge pull request #452 from doronhi/development
  build with librealsense 2.16
* create wrapper class PipelineSyncer to work around librealsense 2.16 feature, removing operator() from class asynchronous_syncer.
* Merge pull request #440 from doronhi/development
  merge PR regarding CMakefile and package.xml
* remove librealsense2 from catkin dependencies.
* Use find_package() variables.
* Merge pull request #439 from doronhi/development
  namespace argument renamed "camera".
* namespace argument renamed "camera".
* Merge branch 'MisoRobotics-fix-rotationMatrixToQuaternion-declaration' into development
* fix input for realsense2_camera::rotationMatrixToQuaternion from float[3] to float[9]
* line up <group ns> parameter in all launch files. (#438)
  fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge branch 'SteveMacenski-launch_name_configuration' into development
* Merge branch 'launch_name_configuration' of https://github.com/SteveMacenski/realsense into SteveMacenski-launch_name_configuration
* Travis CI build and test (#437)
  * fix issue #335 according to solution lsolanka as suggested in pull request #336.
  * moving all the properties and material definitions inside the macro as suggested by @felixvd
  * add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
  * add .travis.yml file
* remove parse_bag_file.py
* possible fix
* Merge remote-tracking branch 'reset_dev/reset_dev' into development
* use locations of realsense2
* TravisCI.yml: fix and add data downloading.
  rs2_test.py: fix test to match new bag file: outdoors.bag
* update .travis.yml
  make test expected to fail to display SUCCESS.
* moved .travis.yml to root
* add .travis.yml file
* Merge branch 'read_bg_file' into development
* Merge branch 'Origin->development' 'fork->development'
* add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
* making camera name configurable, necessity for launching multiple cameras
* Merge pull request #418 from yayaneath/alignment
  Fix the name of the alignment-related parameters when invoking the RealSenseNodeFactory.
* Fix the name of the alignment-related parameters when invoking the RealSenseNodeFactory.
* Merge pull request #417 from doronhi/fix_bug_pointer_out_of_bounds
  fix bug pointer out of bounds
* fix issue #335 according to solution lsolanka as suggested in pull request #336.
* Merge branch 'development' of https://github.com/doronhi/realsense into development
* moving all the properties and material definitions inside the macro as suggested by @felixvd
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge branch 'Affonso-Gui-add_d435_urdf' including some modifications into development
* fixed coordinate system for sensors in camera.
  renamed fisheye to color camera
* Merge branch 'add_d435_urdf' of https://github.com/Affonso-Gui/realsense into Affonso-Gui-add_d435_urdf
* Merge pull request #374 from scythe-robotics/development
  Fixes librealsense CMake vars.
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge pull request #367 from AlanBB277/development
  checked also with D415. Confirmed.
* fix issue #335 according to solution lsolanka as suggested in pull request #336.
* Merge pull request #383 from mikolajz/my-development
  Fix coordinate system transforms so that the pointcloud aligns with camera view
* Fixing the length of an array argument in rotationMatrixToQuaternion
* Add mesh and urdf for D435
* Also when align_depth is no, publish proper data on extrinsic topics.
  AFAIK there is no convention of what to publish on extrinsic topics, so you
  may choose to keep it as is, but I would say the current behavior can be
  surprising in a negative way.
* Fix the rotation quaternion in coordinate transforms.
  When going from one optical frame to another, the actual rotation we are
  performing is quaternion_optical.inverse() * Q * quaternion_optical, so we
  need to for the final rotation to be as specific in the extrinsics.
  The pointcloud is now properly aligned.
* Publish coordinate system transforms also when align depth is on.
  That fact that aligned_depth_to\_* is in color coordinates is already
  experessed by these cameras camera_info reporting the color frame. However,
  for the "depth", "infra1" etc. camera to be properly reported and for the
  pointcloud to have a change to align, we need to report the transformations.
* In coordinate system transforms, fix which extrincits we use and use matrix properly.
  Two bugs which cancel out each other for rotation, but not translation:
  - it seems that ROS and Realsense use different conventions of coordinate
  system transformations. In ROS, it is defined as a transformation of child
  fame coordinates to parent frame coordinates (see
  http://wiki.ros.org/tf/Overview/Transformations), while in RealSense
  it seems to be transformation of "from" frame coordinates to "to" frame
  coordinates. Thus, the order needs to be reversed.
  - the matrix in RealSense extrinsics is stored in column-major format, while
  Eigen::Matrix3f expects row-major, causing the matrix to be transposed.
  To see that this is a problem, one can open rviz and add the pointcloud and the
  color/image_raw camera. From the camera viewpoint, the images should align, but
  don't. This patch doesn't yet solve the whole problem, but makes it smaller.
* Fixes librealsense CMake vars.
* fix the aligned depth frame unit conversion issue
* Merge pull request #364 from lorenwel/fix/aligned_depth_cam_info
  aligned_depth_to\_... assign stream cam info instead of depth
* Assign stream cam info instead of depth
* Merge pull request #352 from ruvu/feature/diagnostics
  Feature/diagnostics
* Corrected diagnostics naming of aligned streams (comment @icarpis)
* correct pointer to expected frequency
* Revert "Use nodehandles from nodelet"
  This reverts commit 03b0114bdca04ac8752c760495981c349b7ae595.
* Use nodehandles from nodelet
* Some logging
* diagnostic updaters with frequency status for publishers
* Merge pull request #351 from icarpis/development
  Bump version
* Bump version
* Merge pull request #350 from icarpis/development
  Improve CPU utilization using rs_rgbd.launch
* Fixed SR300 depth scale issue
* Check for subscribers before publish aligned frames
* Merge pull request #324 from icarpis/development
  Renaming ROS package from realsense_ros_camera to realsense2_camera
* Fixed merge issue
* Renaming ROS package from realsense_ros_camera to realsense2_camera
* Contributors: Abhijit Majumdar, AlanBB277, Allison Thackston, AndyZe, Anthony Musco, Aswin Kumar, Brian Fulkerson, CameronDevine, David W, Enrique Fernandez, Enrique Fernández Perdomo, Florenz Graf, Guilherme de Campos Affonso, Harsh Pandya, Ian Zhang, Itay Carpis, Jack Morrison, Jamie Cho, Jarvis Schultz, Mike Purvis, Mikołaj Zalewski, Miles Price, Nick Giancola, Pavlo Kolomiiets, Phillip Schmidt, Rein Appeldoorn, RenaudeauBrice, RhysMcK, Robert Haschke, Ryan Sinnet, Sai Kishor Kothakota, Sergey Dorodnicov, Shuntaro Yamazaki, Stephan, Stephan Sundermann, Thiago de Freitas, Unknown, Victor Lopez, akira_you, baumanta, brayan, carlos, doronhi, icarpis, iliabara, kamilritz, lorenwel, oka, socrob, stevemacenski, vatbrain
