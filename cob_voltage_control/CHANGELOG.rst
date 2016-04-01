^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_voltage_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.6 (2016-04-01)
------------------
* beautify
* Merge branch 'aggregated_power_message' into feature/raw_batterie_state
  Conflicts:
  cob_voltage_control/common/src/cob_voltage_control_common.cpp
  cob_voltage_control/ros/src/cob_voltage_control_ros.cpp
* voltage and current measurements for raw3-3
* use aggregated power state message
* changed name relayboard to powerboard
* removed debug outputs
* fixed topic names
* Contributors: Benjamin Maidel, ipa-bnm, ipa-fmw

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------
* explicit dependency to boost
* remove trailing whitespaces
* add_dependencies EXPORTED_TARGETS
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------

0.6.2 (2014-12-15)
------------------
* fix install tag
* Tested on cob4-2
* make record current and voltage generic
* Contributors: ipa-cob4-2

0.6.1 (2014-09-17)
------------------
* merge conflict
* Deleted CurrentMeasurement.msg
* Re-add time_volt
* reocord current script and launch file
* fix install tags
* Current measurement
* Removed emergency model and custom Relayboard
* Merge branch 'groovy_dev' of https://github.com/ipa320/cob_driver into groovy_dev
  Conflicts:
  cob_relayboard/ros/src/new_method.py
  cob_relayboard/ros/src/relayboard_sim.py
* Voltage filter now on the config folder
* Changes
* Adjusting path for the required files
* More thrash
* Removing some thrash
* License
* Modifications for the battery characterization on the robot
* Contributors: Thiago de Freitas, ipa-cob4-2, ipa-nhg, thiagodefreitas, thiagodefreitas@gmail.com

0.6.0 (2014-09-09)
------------------
* trying to fix cob_voltage_control buildfarm error
* missing dependency
* Contributors: Florian Weisshardt, ipa-fxm

0.5.7 (2014-08-26)
------------------
* Merge pull request `#163 <https://github.com/ipa320/cob_driver/issues/163>`_ from ipa320/hydro_dev
  updates from hydro_dev
* 0.5.6
* update changelog
* merge
* move EmergencyStopState.msg to cob_msgs + PowerBoardState works again
* changes due to introduction of cob_msgs
* added message to submit voltage data
* Cleaned up cob_driver with reduced deps to compile on indigo
* increased receive buffer
* Merge pull request `#136 <https://github.com/ipa320/cob_driver/issues/136>`_ from ipa-fmw/hydro_dev
  change maintainer and add missing dependency
* Update package.xml
* fix
* fix
* voltage ctrl over phidgets
* voltage info from phidgets
* work in progress voltagectrl with cob_phidget
  Merge branch 'groovy_dev' into feature/voltagectrl_newphidget
  Conflicts:
  cob_voltage_control/ros/src/cob_voltage_control_ros.cpp
* voltagectrl work in progress
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, ipa-bnm, ipa-fxm

0.5.6 (2014-08-26)
------------------
* Merge pull request `#163 <https://github.com/ipa320/cob_driver/issues/163>`_ from ipa320/hydro_dev
  updates from hydro_dev
* merge
* move EmergencyStopState.msg to cob_msgs + PowerBoardState works again
* changes due to introduction of cob_msgs
* added message to submit voltage data
* Cleaned up cob_driver with reduced deps to compile on indigo
* increased receive buffer
* Merge pull request `#136 <https://github.com/ipa320/cob_driver/issues/136>`_ from ipa-fmw/hydro_dev
  change maintainer and add missing dependency
* Update package.xml
* fix
* fix
* voltage ctrl over phidgets
* voltage info from phidgets
* work in progress voltagectrl with cob_phidget
  Merge branch 'groovy_dev' into feature/voltagectrl_newphidget
  Conflicts:
  cob_voltage_control/ros/src/cob_voltage_control_ros.cpp
* voltagectrl work in progress
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, ipa-bnm, ipa-fxm

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* some install tag updates
* removed phidget21.h include inside main source file due to missing header guard
* bug fix in voltage_control
* remove old rosbuild import
* merged with groovy_dev upstream
* No need for the stds, thrash from the voltage checks on robot-at-work
* Modification from RAW3-1
* Installation stuff
* fix for battery dashboard
* V instead of mV
* use v not mV
* Set parameter nedded for battery monitor
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* Removed csv files from the cob_voltage_control
* Changed launch file to be related to each robot
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* debugging voltage filter
* fix launch file
* nasty fix for rob3-6
* Changes the modes nomenclatures
* More organization to the voltage commit
* Organizing the voltage filter commit
* Reverting new method
* Method modification on the robot
* Reverting new method
* Merge branch 'groovy_dev' of github.com:thiagodefreitas/cob_driver into groovy_dev
  Conflicts:
  cob_voltage_control/ros/src/new_method.py
* Method
* Modifications to the modes
* Mods
* Launch files
* IPA PC
* Starting real time implementation
* Saved at home computer
* Mods
* More mods on the battery statistics
* Mods on the plots
* Starting the statistical analysis of the battery
* added topic for relayboard message
* changed mapping of em state topics to make sense for gui
* added parameters
* added simple voltage calculation
* changes with sensor attached
* initial version of cob_voltage_control
* Contributors: Alexander Bubeck, Frederik Hegger, Thiago de Freitas, Thiago de Freitas Oliveira Araujo, abubeck, ipa-bnm, ipa-cob3-4, ipa-cob3-6, ipa-cob3-7, ipa-fmw, robot, thiagodefreitas, thiagodefreitas@gmail.com
