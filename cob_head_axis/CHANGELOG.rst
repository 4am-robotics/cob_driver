^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_head_axis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* cob_head_axis: tabs to spaces
* cob_head_axis: fix deprecated API in constructor
* Contributors: ipa-fxm, ipa-mig

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* cherry-pick
* removed a lot of code related to packages not available in hydro anymore
* output long unsigned variables correctly
* upstream changes
* Merge branch 'groovy_dev' of git://github.com/ipa320/cob_driver into groovy_dev
* fixed build errors for gcc version >= 4.7
* trying to fix quantal compilation
* merged with upstream
* Fixed diagnostic error for head_axis
* trying to get error state out of head axis
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* futher include and linkpath modifications
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* dirty hack for fixing joint direction
* fix finish bug in head axis
* fix finish bug in head axis
* added effort to joint states message
* remove deprecated link to urdf
* switched from pr2_controllers_msgs::JointTrajectoryAction to control_msgs::FollowJointTrajectory
* removed outdated file - not used anymore
* fix actionserver bug with early success
* beautyfing
* partial fix for head axis
* fix for crash if stop is executed before init
* add state topic for head_axis
* changes for fuerte compatibility
* remove deprecated tests
* delete old in iand yaml files
* added diagnotic topic for initialization states for sdh
* move to private namespace
* private nodehandle
* merge
* adde env ROBOT to test
* added roslaunch tests
* added cob3-4 configs
* removed compiler warnings
* cob_head_axis: turning to 0deg after homing
* cob_head_axis: removed homing-sleep
* added rostest
* modifications for fetch and carry
* update cob3-3
* dummy head axis
* adaptions for cob_head_axis on cob3-3, included some new parameters instead of hard-coded settings
* update for cob3-3
* rearranging cob_camera_sensors launch files
* update for cob3-3
* add services and return true for recover after init
* changed test duration to 10s
* camera settings added for head
* changed head params for cob3-2
* moved init test to cob_srvs
* wrong namespace in test file
* modified parameters
* modified parameters
* modified tests
* release update for cob3-1
* merge
* changed device for head axis
* adjust devices for cob3-1
* changed trigger service
* joint_state aggregator working on cob3-1, calibration script update
* cleanup in cob_driver
* Moved hard-coded lines for head_axis_homing from CanDriveHarmonica.cpp into ElmoCtrl.cpp. Removed debugger in base_drive_chain.launch and undercarriage_ctrl.launch
* Head axis working, tested on cob3-1 but adapted parameters (-files)  should work on both robots
* merge
* HeadAxis working
* bugfix
* Cleaned cob_head_axis yaml-files
* merge
* removed unused parameters
* cob_head_axis: set offset via urdf and chose can-device-path via ini-File
* cob_camera_axis tested, now also is able to be shut down
* cob_head_axis working
* cob_head_axis working
* cob_head_axis: correctly working, but front and back is switched
* renamed camera_axis to head_axis and platform to base
* Contributors: Alexander Bubeck, Felix Messmer, Frederik Hegger, Richard Bormann, abubeck, cpc-pk, fmw-jk, ipa-bnm, ipa-cob3-4, ipa-cob3-5, ipa-fmw, ipa-fxm, ipa-goa, ipa-mig, ipa-uhr
