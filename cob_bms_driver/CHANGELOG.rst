^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_bms_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
