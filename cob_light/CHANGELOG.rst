^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_light
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2022-11-17)
-------------------

0.7.13 (2022-07-29)
-------------------
* Merge pull request `#433 <https://github.com/ipa320/cob_driver/issues/433>`_ from fmessmer/cob_light/num_led_info
  [cob_light] more info in error case
* more info in error case
* Contributors: Felix Messmer, fmessmer

0.7.12 (2022-03-15)
-------------------
* Merge pull request `#430 <https://github.com/ipa320/cob_driver/issues/430>`_ from benmaidel/feature/new_modes
  [cob_light] new features
* catkin lint
* updated license header
* add turn indicator mode
* decrease fade factor for kit
* fun kit mode
* Contributors: Benjamin Maidel, Felix Messmer

0.7.11 (2022-01-12)
-------------------

0.7.10 (2021-12-23)
-------------------
* Merge pull request `#428 <https://github.com/ipa320/cob_driver/issues/428>`_ from benmaidel/fix/debug_output
  removed console logging
* removed console logging
* Contributors: Benjamin Maidel

0.7.9 (2021-11-26)
------------------

0.7.8 (2021-10-19)
------------------

0.7.7 (2021-08-02)
------------------

0.7.6 (2021-05-10)
------------------

0.7.5 (2021-04-06)
------------------
* Merge pull request `#419 <https://github.com/ipa320/cob_driver/issues/419>`_ from benmaidel/fix/topic_multi_color
  [cob_light] fix topic multicolor support
* fix topic multicolor support
* Merge pull request `#418 <https://github.com/ipa320/cob_driver/issues/418>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Benjamin Maidel, Felix Messmer, fmessmer

0.7.4 (2020-10-14)
------------------
* Merge pull request `#417 <https://github.com/ipa320/cob_driver/issues/417>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.7.3 (2020-03-18)
------------------

0.7.2 (2020-03-18)
------------------
* Merge pull request `#409 <https://github.com/ipa320/cob_driver/issues/409>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* python3 compatibility via 2to3
* Merge pull request `#408 <https://github.com/ipa320/cob_driver/issues/408>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#406 <https://github.com/ipa320/cob_driver/issues/406>`_ from lopsided98/cob-light-boost-signals
  [cob_light] removed unused Boost signals dependency
* cob_light: removed unused Boost signals dependency
* Contributors: Ben Wolsieffer, Felix Messmer, Loy van Beek, fmessmer

0.7.1 (2019-11-07)
------------------

0.7.0 (2019-08-06)
------------------
* Merge pull request `#380 <https://github.com/ipa320/cob_driver/issues/380>`_ from ipa-jba/fix/boost_shared_ptr
  [Melodic] combined melodify pr
* more constexpr in cob_light
* c++11 for cob_light
* fixed boost colors signal type
* disable narrowing conversion warnings
* fix "constexpr needed" compilation error
* Merge pull request `#396 <https://github.com/ipa320/cob_driver/issues/396>`_ from HannesBachter/indigo_dev
  0.6.15
* Contributors: Benjamin Maidel, Felix Messmer, Jannik Abbenseth

0.6.15 (2019-07-17)
-------------------

0.6.14 (2019-06-07)
-------------------

0.6.13 (2019-03-14)
-------------------

0.6.12 (2018-07-21)
-------------------
* update maintainer
* Contributors: fmessmer

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#341 <https://github.com/ipa320/cob_driver/issues/341>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.10 (2017-07-24)
-------------------
* fixed function return value
* Contributors: Benjamin Maidel

0.6.9 (2017-07-18)
------------------
* added test script
  added test script
  added test script
  fix
  fix
  fix
  added additional mode to test scripts
  speed up test scripts
* protect callbacks with scoped locks
* void function
* manually fix changelog
* xmas mode corrections
* added new lightmode xmas
* Contributors: Benjamin Maidel, ipa-fxm

0.6.8 (2016-10-10)
------------------
* return true for stop mode service to avoid ugly error messages on client side
* fix tab vs spaces
* at static mode send color to driver on begin of update cylce
* set freq to default 1.0 if not set by requested mode
* Contributors: Benjamin Maidel, ipa-fmw, msh

0.6.7 (2016-04-02)
------------------

0.6.6 (2016-04-01)
------------------
* keep static mode running so they can be stopped, paused and resumed
* fixes
* comments
* param tuning
* implemented new mode glow
* implemented feature to resume modes if modes with higher prio ends + refactored code
* considered `#241 <https://github.com/ipa320/cob_driver/issues/241>`_ PR comments
* high distances visualized with a lighter green
* moved light dist approx function to cob_light driver
* wip light_approximation
* Merge branch 'feature/led_offset' into feature/light_approximation
* fix
* added node for scan approxiamtion led visualization
* added param to define led mounting offset
* fixed ms35 controller bug
* Merge branch 'indigo_dev' of github.com:ipa320/cob_driver into indigo_dev
  Conflicts:
  cob_light/CMakeLists.txt
* added headers to add_executable makro for qt_creator visiblity
* Contributors: Benjamin Maidel, bnm

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------
* boost revision
* do not install headers in executable-only packages
* explicit dependency to boost
* remove obsolete autogenerated mainpage.dox files
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------
* use component namespaces for light, mimic and say
* make visualization marker frame configurable
* publish contiuous diagnostics
* small fixes
* Merge branch 'feature/newCircleMode' into feature/newSweepMode
  Conflicts:
  cob_light/common/src/modeFactory.cpp
* added color array msg
* modified message description
* added new sweep color mode
* Merge pull request `#190 <https://github.com/ipa320/cob_driver/issues/190>`_ from ipa-bnm/bug/infinit-send-error
  bugfix infinit-send-error
* Merge branch 'bug/infinit-send-error' into indigo_dev
* fix stageprofi init
* check for acknowledge message from controller
* recover serial connection and driver if there was an error during sending
* Contributors: Benjamin Maidel, Florian Weisshardt, ipa-fmw

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
