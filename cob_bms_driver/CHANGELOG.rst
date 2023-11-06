^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_bms_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.15 (2023-11-06)
-------------------

0.7.14 (2022-11-17)
-------------------
* Merge pull request `#437 <https://github.com/ipa320/cob_driver/issues/437>`_ from Deleh/fix/power_consume
  Fix power consumption of fake BMS
* round remaining capacity for publisher and diagnostics
* remove round of remaining capacity
* Merge pull request `#436 <https://github.com/ipa320/cob_driver/issues/436>`_ from HannesBachter/audi_scenario
  configurable battery details
* get battery details from parameter server
* adjust battery capacity and discharging values to new diff base
* Contributors: Denis Lehmann, Felix Messmer, HannesBachter, fmessmer

0.7.13 (2022-07-29)
-------------------
* Merge pull request `#434 <https://github.com/ipa320/cob_driver/issues/434>`_ from floweisshardt/feature/power_state_connected
  add explicit power_state.connected
* remove unused subscriber
* add explicit power_state.connected
* Merge pull request `#432 <https://github.com/ipa320/cob_driver/issues/432>`_ from fmessmer/feature/publish_battery_state
  power_state_aggregator now also publishes sensor_msgs.BatteryState
* fix percentage value range
  Co-authored-by: Benjamin Maidel <benjamin.maidel@mojin-robotics.de>
* update current_buffer_size
* power_state_aggregator now also publishes sensor_msgs.BatteryState
* Contributors: Felix Messmer, floweisshardt, fmessmer

0.7.12 (2022-03-15)
-------------------

0.7.11 (2022-01-12)
-------------------
* Merge pull request `#429 <https://github.com/ipa320/cob_driver/issues/429>`_ from fmessmer/fix/invalid_value_warning
  catch invalid value warning
* use odd number for buffer_size to prevent 0.0 mean
* catch invalid value warning
* Contributors: Felix Messmer, fmessmer

0.7.10 (2021-12-23)
-------------------

0.7.9 (2021-11-26)
------------------
* Merge pull request `#427 <https://github.com/ipa320/cob_driver/issues/427>`_ from benmaidel/fix/fake_bms_service
  [BMS] fix fakebms set_relative_remaining_capacity service
* fix fakebms set_relative_remaining_capacity service
* Contributors: Benjamin Maidel, Felix Messmer

0.7.8 (2021-10-19)
------------------

0.7.7 (2021-08-02)
------------------

0.7.6 (2021-05-10)
------------------
* Merge pull request `#423 <https://github.com/ipa320/cob_driver/issues/423>`_ from mikaelarguedas/python2-deps
  ROS_PYTHON_VERSION conditional dependency for `python-tk` and `python-numpy`
* convter cob_bms_driver to package format 3
* ROS_PYTHON_VERSION conditional dependency for python-numpy
* Contributors: Felix Messmer, Mikael Arguedas

0.7.5 (2021-04-06)
------------------
* Merge pull request `#418 <https://github.com/ipa320/cob_driver/issues/418>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

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
* Merge pull request `#408 <https://github.com/ipa320/cob_driver/issues/408>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.7.1 (2019-11-07)
------------------

0.7.0 (2019-08-06)
------------------
* Merge pull request `#380 <https://github.com/ipa320/cob_driver/issues/380>`_ from ipa-jba/fix/boost_shared_ptr
  [Melodic] combined melodify pr
* melodify cob_bms_driver_node
* Merge pull request `#396 <https://github.com/ipa320/cob_driver/issues/396>`_ from HannesBachter/indigo_dev
  0.6.15
* Contributors: Felix Messmer, Jannik Abbenseth

0.6.15 (2019-07-17)
-----------
* 0.6.14
* update changelogs
* Merge pull request `#393 <https://github.com/ipa320/cob_driver/issues/393>`_ from fmessmer/add_int_bms_parameter
  add int bms parameter types
* get rid of c++11 compile options
* fix key name
* add int bms parameter types
* Merge pull request `#392 <https://github.com/ipa320/cob_driver/issues/392>`_ from fmessmer/max_time_remaining
  clamp time_remaining when current is zero
* clamp time_remaining when current is zero
* Contributors: Felix Messmer, fmessmer

0.6.14 (2019-06-07)
-------------------
* Merge pull request `#393 <https://github.com/ipa320/cob_driver/issues/393>`_ from fmessmer/add_int_bms_parameter
  add int bms parameter types
* get rid of c++11 compile options
* fix key name
* add int bms parameter types
* Merge pull request `#392 <https://github.com/ipa320/cob_driver/issues/392>`_ from fmessmer/max_time_remaining
  clamp time_remaining when current is zero
* clamp time_remaining when current is zero
* Contributors: Felix Messmer, fmessmer

0.6.13 (2019-03-14)
-------------------
* Merge pull request `#381 <https://github.com/ipa320/cob_driver/issues/381>`_ from pholthau/boost-format
  include boost/format.hpp
* include boost/format.hpp
* Contributors: Felix Messmer, Patrick Holthaus

0.6.12 (2018-07-21)
-------------------
* update maintainer
* Merge pull request `#374 <https://github.com/ipa320/cob_driver/issues/374>`_ from floweisshardt/feature/round_remaining_capacity
  round remaining_capacity
* adjust to real driver precision
* round remaining_capacity
* Contributors: Felix Messmer, fmessmer, ipa-fmw, ipa-fxm

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#364 <https://github.com/ipa320/cob_driver/issues/364>`_ from ipa-fxm/fake_bms_diagnostics
  use diagnostic updater in fake_bms
* use diagnostic updater in fake_bms
* Merge pull request `#361 <https://github.com/ipa320/cob_driver/issues/361>`_ from ipa-fxm/set_relative_remaining_capacity
  set relative remaining capacity
* set relative remaining capacity
* Merge pull request `#341 <https://github.com/ipa320/cob_driver/issues/341>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* change maintainer
* use license apache 2.0
* Contributors: Felix Messmer, Florian Weisshardt, ipa-fxm, ipa-uhr-mk

0.6.10 (2017-07-24)
-------------------

0.6.9 (2017-07-18)
------------------
* minor change for handling exception
* made changes which only sets the current that in turn is used by power_aggregator for relative_remaining_capacity calculation
* fix typo
* added emulation of realistic current value
* minor change for publishing a realistic voltage value
* Merge pull request `#310 <https://github.com/ipa320/cob_driver/issues/310>`_ from souravran/feature/fake_bms
  added a fake bms with set_charging and set_relative_remaining_capacity services
* finalize exception handling
* fake current
* consistent naming
* publish diagnostics in fake_bms
* harmonize namespaces of fake_bms
* uses the default parameter value
* poll frequency has been set from the parameter list
* made changes as per the review.
  power state elements being published at 20 Hz.
  removed junk rospy log and changed division_by_zero error message.
* fake_bms publishing all power_state entities.
  added exception handling in power_state_aggregator.
  added package dependency and install tags.
* added a fake bms with set_charging and set_relative_remaining_capacity services
* fix typo
* fix powerstate aggregator charging flag (bms is not delivering correct flag for full battery and docked)
* use bms flag for harging
* fix identation
* use spaces for indention in BMS driver
* updated authors
* added support for bit_mask'ed booleans
* make BmsParameter an abstract base class
* BMS driver clean-up
* switch from map of vectors to multimap in BMS driver
* simplified BMS publisher creation and polling list optimization
* simplified BMS config parsing
* manually fix changelog
* Contributors: Felix Messmer, Florian Weisshardt, Mathias Lüdtke, Nadia Hammoudeh García, fmw-ss, ipa-fxm, robot

0.6.8 (2016-10-10)
------------------
* restart CAN on failure
* move power_state_phidget node to new package
* invert current + round values
* fix typo
* corrected namespace
* added node to calculate powerstate from phidget board
* Contributors: Benjamin Maidel, Mathias Lüdtke

0.6.7 (2016-04-02)
------------------
* add missing dependencies
* Contributors: ipa-fxm

0.6.6 (2016-04-01)
------------------
* dependency and package cleanup
* remove config and launch as it is added to cob_robots
* adjust version
* move cob_bms_driver to cob_driver
* Contributors: ipa-fxm

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------

0.6.3 (2015-06-17)
------------------

0.6.2 (2014-12-15)
------------------

0.6.1 (2014-09-17)
------------------

0.6.0 (2014-09-09)
------------------

0.5.7 (2014-08-26 09:47)
------------------------

0.5.6 (2014-08-26 09:42)
------------------------

0.5.5 (2014-08-26 08:33)
------------------------

0.5.4 (2014-08-25)
------------------

0.5.3 (2014-03-31)
------------------

0.5.2 (2014-03-21)
------------------

0.5.1 (2014-03-20 10:54)
------------------------
