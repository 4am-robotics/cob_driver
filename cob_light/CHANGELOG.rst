^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_light
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2014-12-15)
------------------
* fix warning message
* Merge pull request `#188 <https://github.com/ipa320/cob_driver/issues/188>`_ from ipa-bnm/feature/newCircleMode
  feature/new-circle-mode and bugfixes
* Merge branch 'feature/newCircleMode' of https://github.com/ipa-bnm/cob_driver into indigo_new_structure
* change default frequency and fix breath mode
* Merge branch 'indigo_dev' of https://github.com/ipa-nhg/cob_driver into ipa-nhg-indigo_dev
* added circle color mode test script
* fixed circle color mode
* implemented new circular mode
* defined 1Hz startup frecuency
* frequency corresponds to choosen mode
* fix
* removed debug output
* fixes to stagedriver and some refactoring
* write stageprofi colors for all dmx channels within one command
* cleanup
* queue messages
* added concurrent queue
* merge conflict
* typo fix
* Tested on cob4-2 for all modes
* Changes for the LED driver without led numbers
* Tested on cob4-2
* Temporary commit for tests
* CHanges for array of leds
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_driver into indigo_dev
  Conflicts:
  cob_light/ros/src/ms35.cpp
* Removed unecessary debug
* fix minor compiler warning
* new line at end of file
* Changes formatting
* Support for the StageProfi board on cob_light
* Contributors: Benjamin Maidel, Florian Weisshardt, bnm, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg, thiagodefreitas

0.6.1 (2014-09-17)
------------------

0.6.0 (2014-09-09)
------------------

0.5.7 (2014-08-26)
------------------
* Merge pull request `#163 <https://github.com/ipa320/cob_driver/issues/163>`_ from ipa320/hydro_dev
  updates from hydro_dev
* 0.5.6
* update changelog
* added explicit default argument queue_size
* Cleaned up cob_driver with reduced deps to compile on indigo
* port settings fixed
* added light support for conrad ms-35 led controller
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, ipa-bnm

0.5.6 (2014-08-26)
------------------
* Merge pull request `#163 <https://github.com/ipa320/cob_driver/issues/163>`_ from ipa320/hydro_dev
  updates from hydro_dev
* added explicit default argument queue_size
* Cleaned up cob_driver with reduced deps to compile on indigo
* port settings fixed
* added light support for conrad ms-35 led controller
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, ipa-bnm

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* change Error to warning
* console feedback if mode finished
* bugfixed segfault if a freq with zero was set (default freq = 10Hz)
* changed timeout to si unit (ms -> s)
* cleaned up CMakeLists and added install directives
* merged with ipa320
* added missing message_gen deps
* merge ipa320
* futher include and linkpath modifications
* add message dependencies
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* added diagnostics to cob_light
* startup color and mode can now be defined by parameters
* worked on simulation mode
* fixed reading params from parameterserver
* some refactoring
* added some more source code description
* service got same return type as the action
* refactor
* modes reflect their name
* add __SIM__ ifdef to cob_light
* new cob light driver
* enable light marker by default
* updated cob_light
* added rosparam to set inversion mask instead of using environment variable
* cob_light re-inserted
* hwboard updated
* added check for existance of Robot environment variable, if not available default cob setting is used
* removed old python light node
* last modifications after testing
* some fixes to new cob_light node. tested and working
* turning off leds on init
* removed lightmode stuff (breathing, flasing...) from cob_light python driver
* moved light control (sound, breath, flash...) from cob_light driver into own package
* merged cob_lights
* merge
* testing cpp cob_light node
* some modifications for correct fft analysis
* callback func for soundcontroller
* added cob_light cpp node with soundanalyser capabilities over fft
* ported cob_light controller to cpp
* added led breath functionality and a service to change led mode
* add message output for device name
* fix for light on cob3-3
* remove deprecated Light message
* publish light marker continuously
* Change to ColorRGBA message the light test
* changed light to std_msgs/ColorRGBA message
* add simulation variant of light sensors and publish visualization marker
* Deleted launch tests in CMakeLists
* Moved light.launch to cob_bringup
* moved the light parameter configuration to cob_hardware_config
* moved the light parameter configuration to cob_hardware_config
* merge
* added roslaunch tests
* undo previous merge + commits
* merge with review-sven
* cameras working and calibrated
* update stacks
* moved light message to cob_light
* cleanup in cob_driver
* adapt light for cob3-2
* light device for cob3-2
* update documentation and deleted tf broadcaster
* modification for cob3-2
* adaption to light
* launch file and parameter check for cob_light
* changed light topic
* changes on light controller
* cib_light is working
* new package for lights, not working yet
* Contributors: Alexander Bubeck, abubeck, cob-hardware-test, cob3-1-pc2, cpc-pk, fmw, ipa-bnm, ipa-fmw, ipa-nhg, ipa-uhr-eh, ipa-uhr-fm
