^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_sick_lms1xx
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.14 (2022-11-17)
-------------------

0.7.13 (2022-07-29)
-------------------

0.7.12 (2022-03-15)
-------------------

0.7.11 (2022-01-12)
-------------------

0.7.10 (2021-12-23)
-------------------

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
* Merge pull request `#405 <https://github.com/ipa320/cob_driver/issues/405>`_ from fmessmer/fix_warnings
  fix compile warnings
* fix -Wunused-result in cob_sick_lms1xx
* Contributors: Felix Messmer, fmessmer

0.7.1 (2019-11-07)
------------------

0.7.0 (2019-08-06)
------------------
* Merge pull request `#396 <https://github.com/ipa320/cob_driver/issues/396>`_ from HannesBachter/indigo_dev
  0.6.15
* Contributors: Felix Messmer

0.6.15 (2019-07-17)
-------------------

0.6.14 (2019-06-07)
-------------------

0.6.13 (2019-03-14)
-------------------

0.6.12 (2018-07-21)
-------------------

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#341 <https://github.com/ipa320/cob_driver/issues/341>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.10 (2017-07-24)
-------------------

0.6.9 (2017-07-18)
------------------
* manually fix changelog
* Contributors: ipa-fxm

0.6.8 (2016-10-10)
------------------
* cob_sick_lms1xx: add range configuration
* Contributors: Shin

0.6.7 (2016-04-02)
------------------
* add missing dependencies
* Contributors: ipa-fxm

0.6.6 (2016-04-01)
------------------
* Sick LMS1xx node now uses global NodeHandle
* Resolved problem with tabs and spaces.
* Clean trailing spaces. Convert default frame name to tf2 format (remove "/")
* Revert indentation style as it was
* Correction.
* Corrected revert.
* Revert indentation style as it was
* Node rewritten and publishing on diagnostics topic added.
* Contributors: Denis Štogl

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------
* install tags for libraries
* boost revision
* do not install headers in executable-only packages
* explicit dependency to boost
* catkin_package according to install tags
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

0.6.1 (2014-09-17)
------------------

0.6.0 (2014-09-09)
------------------

0.5.7 (2014-08-26)
------------------
* 0.5.6
* update changelog
* merge
* fix python3 ascii error while parsing "S"
* Corrected inversion for lms1xx
* Merge pull request `#136 <https://github.com/ipa320/cob_driver/issues/136>`_ from ipa-fmw/hydro_dev
  change maintainer and add missing dependency
* Update package.xml
* Contributors: Denis Štogl, Florian Weisshardt, Nadia Hammoudeh García, ipa-fxm

0.5.6 (2014-08-26)
------------------
* merge
* fix python3 ascii error while parsing "S"
* Corrected inversion for lms1xx
* Merge pull request `#136 <https://github.com/ipa320/cob_driver/issues/136>`_ from ipa-fmw/hydro_dev
  change maintainer and add missing dependency
* Update package.xml
* Contributors: Denis Štogl, Florian Weisshardt, Nadia Hammoudeh García, ipa-fxm

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* fixed missing include allowing sleep()
* Changed node name.
* New package with driver for Sick LMS1xx. Driver is taken from https://github.com/ipa320/RCPRG_laser_drivers.git.
* Contributors: Alexander Hagg, Denis Štogl, IPR-SR2
