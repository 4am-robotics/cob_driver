^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_phidgets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* use local namespace
* remove launch and yaml configuration. this should be either in cob_robots or on the ROS wiki page
* removed some debug outputs
* fixed crash due to non existing phidget configuration for an attached board
* modified phidget config
* changed template config
* added topic to set digital outputs
* fixed include error
* debug infos
* example config changed
* changed service message description from index to uri
* fix include path for libphidgets
* added license headers
* bugfix
* moved services to boards individual namespace
* removed unused files
* cleaned up code
* new phidget config structure
* params reading fix
* implemented new sensor message layout
* added launch file
* publish on root namespace for all attached boards
* phidget config yaml prototype
* exposed settings to yaml param file
* exposed settings to yaml param file
* new message format + POLLING and EVENT based mode
* changed namespaces
* cleanup
* threading and memory-leak fixes
* correction for gcc4.6
* gcc 4.6 corrections
* new phidget driver
* testing
* fix
* flush input buffer
* renaming phidget labels
* changes
* display device name in list
* added phidget device name setter
* Installation stuff
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* eliminate global variable and fix indentation
* delete launch file, should be in cob_robots
* tray: fix
* tray: outputting correct range msgs + configurable
* renamed cob_tray_sensors to cob_phidgets and removed tray occupied check, now only publishing raw data
* Contributors: Alexander Bubeck, Joshua Hampp, abubeck, ipa-bnm, ipa-fmw, ipa-nhg
