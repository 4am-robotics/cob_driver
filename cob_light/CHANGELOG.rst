^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_light
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
