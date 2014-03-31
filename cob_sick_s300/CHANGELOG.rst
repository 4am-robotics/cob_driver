^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_sick_s300
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* fix CMakeLists
* move unified scan publisher to cob_navigation
* fixed little issue and tested on raw3-1
* new parsing function
* Added compilation definitino for unified scan publisher and started transition from tf to tf2 (Added tf2 dependecies and included header files).
* trying to fix quantal compilation
* fix compile error for ubuntu > precise
* Installation stuff
* merged with upstream
* fix for oodl
* cleaned up CMakeLists and added install directives
* futher include and linkpath modifications
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* cob_sick_s300: changes from Jan Paulus, BRSU
* fixed parameter handling
* electric backport of sick driver
* changes for fuerte compatibility
* remove test for non existent launch files
* removed deprecated yaml and launch files
* cob_sick_s300: set range_min and range_max
* cob_scan_filter: setting filtered values to 0 to be skipped by ros::laser_projector
* sick_s300: corrected error from merge with ipa320 diagnostic implementation
* merge with ipa320
* cob_scan_gilter is working with multiple scan intervals -> tested
* scan_filtered: specifiying ranges from-PI to PI.
* scan_filter: before merge
* cob scan filter
* sick_s300: introduced scan_cycle_time, decrementing time_increments -> experiments look good
* sick_s300: now time_increment now negative, better overall results
* sick_s300: angle_increment negative to keep time diffs...
* added diagnostic messages to sick module
* sick_s300: changed laser_frequency to scan_duration
* sick_s300
* sick_s300: cleaned file, parameter loading
* cob_scan_filter: added additional handling for invalid intervals..
* cob_scan_filter: added funcionality to specify multiple intervals to be filtered out from any LaserScan topic
* sick_s300: debugged some type conversions, time sync now working, not tested on hw
* sick_s300: always using latest scan-message from buffer, sync stamps with ros-time
* sick_s300: scanning for data-sets backwards in stream->getting newer scans
* sick_tests
* Merge branch 'master' into sick_test
* tests on laser scanner/serial
* using private nodehandle
* merge
* additional config files for cob3-bosch
* added roslaunch tests
* added cob3-4 configs
* additional config files for cob3-bosch
* added rostest
* added node for unifying front and rear scanner in base_link frame in order to use the resulting 360 degree scan for gmapping
* icob changes
* adapted test files
* update cob3-3
* rearranging cob_camera_sensors launch files
* scanner config for icob
* config for cob3-3
* config for cob3-3
* changed test duration to 10s
* camera settings added for head
* correct rear scan launch
* added camera tests
* unified names
* modified parameters
* modified parameters
* modified tests
* modified sick driver to support efi configuration
* switched head modules
* cleanup in cob_driver
* merged rostest files
* included new rostest file rear.test
* included new rostest file front.test
* new rostest file
* deleted old restest file
* launch files for testing
* hztest for rear scanner
* added rostest tag
* parameter changes
* front scanner launch file and front scanner test file
* laser filter working on cob3-1
* test for scan front and scan front raw
* rostest for scan front
* added scanfilter for front scanner
* merge
* lbr working on cob
* modifications for cob3-1
* bringup for cob3-1
* update documentation and deleted tf broadcaster
* adaptions for cob3-2
* restructures launch files for sick
* testing navigation
* modified url
* cleanup in cob_simulation
* cleanup in cob_driver
* new files for navigation, e.g. maps and launch files
* cob_2dnav working
* JSF: Added intrinsics to topic
* adaptions to cob3-1
* added cob3-1 launch files
* improved navigation parameters
* separated launch files for cob3-2
* test of ROS navigation on cob
* adjusted file paths
* Merge branch 'fmw-hj' into review-bitbots
* renamed packages to cob_
* Contributors: Alexander Bubeck, COB3-Navigation, Denis Štogl, Richard Bormann, Your full name, abubeck, b-it-bots, cob, cpc-pk, fmw, fmw-jk, ipa, ipa-fmw, ipa-fxm, ipa-mig, ipa-uhr, raw3
