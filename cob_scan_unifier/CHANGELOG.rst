^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_scan_unifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.11 (2022-01-12)
-------------------

0.7.10 (2021-12-23)
-------------------

0.7.9 (2021-11-26)
------------------

0.7.8 (2021-10-19)
------------------
* Merge pull request `#424 <https://github.com/ipa320/cob_driver/issues/424>`_ from lpk1950/publish_unified_points
  Publish unified points
* final touches
* rename point_cloud to pointcloud
* Changes based on the review
* Changes on review: Separate publisher
* Format
* Format
* check for the point_cloud param and set to false by default
* publish pointcloud only when the parameter is set
* change in topic name
* publish unified scan as unified pointcloud
* Contributors: Felix Messmer, fmessmer, karthik

0.7.7 (2021-08-02)
------------------

0.7.6 (2021-05-10)
------------------

0.7.5 (2021-04-06)
------------------
* Merge pull request `#420 <https://github.com/ipa320/cob_driver/issues/420>`_ from lindemeier/feature/improve_performance
  performance boost
* performance boost
* Merge pull request `#418 <https://github.com/ipa320/cob_driver/issues/418>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer, tsl

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
* Merge pull request `#396 <https://github.com/ipa320/cob_driver/issues/396>`_ from HannesBachter/indigo_dev
  0.6.15
* Contributors: Felix Messmer

0.6.15 (2019-07-17)
-------------------

0.6.14 (2019-06-07)
-------------------

0.6.13 (2019-03-14)
-------------------
* Merge pull request `#385 <https://github.com/ipa320/cob_driver/issues/385>`_ from fmessmer/scan_unifier_range_initialization
  initialize ranges with max_range rather than zero
* add explanatory comment
* initialize ranges with max_range rather than zero
* Contributors: Felix Messmer, fmessmer

0.6.12 (2018-07-21)
-------------------
* update maintainer
* Merge pull request `#366 <https://github.com/ipa320/cob_driver/issues/366>`_ from ipa-bnm/feature/scan_unifier
  merge up to 4 laserscans
* merge up to 4 laserscans
* Contributors: Benjamin Maidel, Richard Bormann, fmessmer

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#353 <https://github.com/ipa320/cob_driver/issues/353>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#341 <https://github.com/ipa320/cob_driver/issues/341>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.10 (2017-07-24)
-------------------

0.6.9 (2017-07-18)
------------------
* remove commented line
* Added sleep in constructor, new topic parameter parsing, better error handling.
* Some small fixes
* Cleanup
* Use message_filter::Synchronizer (there is still a bug)
* manually fix changelog
* Contributors: Elias Marks, Matthias Gruhler, ipa-fxm

0.6.8 (2016-10-10)
------------------

0.6.7 (2016-04-02)
------------------

0.6.6 (2016-04-01)
------------------
* changed scan_unifier maintainer
* filtered out scan unifier and moved to new directory
* Contributors: Benjamin Maidel

0.6.3 (2015-08-31)
------------------
* remove trailing whitespace
* migration to package format v2, indentation fixes
* Merge remote-tracking branch 'origin-ipa320/hydro_dev' into indigo_dev
* reduced MAGIC NUMBER
* check range values
* round index
* Contributors: ipa-josh, ipa-mig

0.6.2 (2015-06-17)
------------------
* cob_scan_unifier: get rid of exported but uninstalled include path
* cob_scan_unifier: fix include folder stuff
* Contributors: ipa-mig

0.6.1 (2014-09-18)
------------------

0.6.0 (2014-09-10)
------------------

0.5.2 (2014-08-28)
------------------
* add changelog
* cob_scan_unifier: fix laser projection. wrong parameter
* cob_scan_unifier: added intensities to unified scan and use the nearest range measurement from all incoming scan
* adjusted license header in cob_scan_unifier
* updated license tag in cob_scan_unifier
* another indentation-fix-attempt
* merge
* removed start_delay from scan-unifier configs and intendation-fix
* Update scan_unifier_node.h
  fixed intendation
* renamed ipa_navigation_scan_uniffier to cob_scan_unifier
* Contributors: Florian Mirus

0.5.1 (2014-03-24)
------------------
