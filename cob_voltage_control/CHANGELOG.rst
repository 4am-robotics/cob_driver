^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_voltage_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
