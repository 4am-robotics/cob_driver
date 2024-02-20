^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_mimic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.16 (2024-02-20)
-------------------

0.7.15 (2023-11-06)
-------------------

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
* Merge pull request `#422 <https://github.com/ipa320/cob_driver/issues/422>`_ from fmessmer/fix/mimic_noetic
  use default libvlc configuration for noetic
* use default libvlc configuration for noetic
* Contributors: Felix Messmer, fmessmer

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
* Merge pull request `#413 <https://github.com/ipa320/cob_driver/issues/413>`_ from fmessmer/remove_mimic_python
  remove cob_mimic python driver
* remove cob_mimic python driver
* Contributors: Felix Messmer, fmessmer

0.7.2 (2020-03-18)
------------------
* Merge pull request `#410 <https://github.com/ipa320/cob_driver/issues/410>`_ from fmessmer/fix_mimic_preemption
  fix mimic preemption + add parameters
* fix preemption condition
* add random_mimics parameter
* add default_mimic parameter
* fix fullscreen
* add blocking parameter
* fix mimic preemption
* Merge pull request `#409 <https://github.com/ipa320/cob_driver/issues/409>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* fix pylint errors
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
* added missing boost format include
* Merge pull request `#396 <https://github.com/ipa320/cob_driver/issues/396>`_ from HannesBachter/indigo_dev
  0.6.15
* Merge pull request `#398 <https://github.com/ipa320/cob_driver/issues/398>`_ from LoyVanBeek/feature/reduce_framerate_and_resolution
  Reduce resolution and framerate in .mp4-files
* Reduce resolution and framerate in .mp4-files
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, Loy van Beek

0.6.15 (2019-07-17)
-------------------
* Merge pull request `#394 <https://github.com/ipa320/cob_driver/issues/394>`_ from HannesBachter/feature/diagnostics
  publish diagnostics
* use one instance and vout config
* change diagnostics info for active mimic
* use own thread for diagnostics
* use async spinner and not one-instance
* use variable for action active
* remove c++11 compile and order mutex unlock
* add compile option and cleanup
* publish diagnostics and use multithreaded spinner
* 0.6.14
* update changelogs
* Contributors: Benjamin Maidel, Hannes Bachter, fmessmer, hyb

0.6.14 (2019-06-07)
-------------------

0.6.13 (2019-03-14)
-------------------

0.6.12 (2018-07-21)
-------------------
* update maintainer
* Merge pull request `#375 <https://github.com/ipa320/cob_driver/issues/375>`_ from fmessmer/bulletproof_mimic
  bulletproof mimic
* bulletproof mimic
* Merge pull request `#371 <https://github.com/ipa320/cob_driver/issues/371>`_ from fmessmer/mimic_play_nondefault_mimics
  allow to play non-default mimics by specifying full filepath
* allow to play non-default mimics by specifying full filepath
* Contributors: Felix Messmer, fmessmer, ipa-fxm

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#359 <https://github.com/ipa320/cob_driver/issues/359>`_ from ipa-bnm/fix/mimic
  [cob_mimic] use glx/opengl output for mimic
* use glx/opengl output for mimic, fixes mimic issue for 6th and 7th gen nuc
* Merge pull request `#354 <https://github.com/ipa320/cob_driver/issues/354>`_ from ipa-bnm/feature/mimic
  [Mimic] improvements
* Merge pull request `#353 <https://github.com/ipa320/cob_driver/issues/353>`_ from ipa-fxm/update_maintainer
  update maintainer
* Merge pull request `#356 <https://github.com/ipa320/cob_driver/issues/356>`_ from ipa-nhg/MimicPy
  HotFix: readded python node for mimic
* remove duplicated test_mimic.py install tag
* use the old driver
* readded python node for mimic
* do not start blinking timer on sleeping or falling_asleep requests
* added random mimics
* double check username
* update maintainer
* Merge pull request `#341 <https://github.com/ipa320/cob_driver/issues/341>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* Merge pull request `#352 <https://github.com/ipa320/cob_driver/issues/352>`_ from ipa-bnm/feature/mimic_sim
  Do not run mimic in fullscreen if sim is enabled
* use license apache 2.0
* Merge branch 'indigo_dev' of github.com:ipa320/cob_driver into feature/mimic_sim
* no fullscreen if sim enabled
* Merge pull request `#345 <https://github.com/ipa320/cob_driver/issues/345>`_ from ipa-fxm/fix_mimic_permission
  guarantee unique copy destinations
* guarantee unique copy destinations
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, ipa-fxm, ipa-nhg, ipa-uhr-mk

0.6.10 (2017-07-24)
-------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* added apache header
* ported mimic from python to c++
* Contributors: Benjamin Maidel, flg-pb

0.6.9 (2017-07-18)
------------------
* update license
* Delete ___init_\_.py
* Update package.xml
* remove vlc.py and add it as rosdep dependency (PR to rosdistro is https://github.com/ros/rosdistro/pull/15366)
* cleanup mimic node
* - removed hardcoded sleep time between transition between emotions.
* fix for the flickering in playback, caused when an emotion is set.
* fix cpu usage of the mimic node
* manually fix changelog
* mimic support the rotation of the face
* Contributors: Felix Messmer, Florian Weisshardt, fmw-ss, ipa-cob4-5, ipa-fxm, ipa-nhg, souravran

0.6.8 (2016-10-10)
------------------
* vlc 2.2 version use by default the wrong video output
* Contributors: ipa-cob4-5, ipa-nhg

0.6.7 (2016-04-02)
------------------

0.6.6 (2016-04-01)
------------------
* re-add copying mimic files
* fix action name in test node
* fix mimic shutdown and cleanup
* Update CMakeLists.txt
* add rospy again
* merge
* missed dependencies
* Contributors: Florian Weisshardt, ipa-fmw, ipa-fxm, ipa-nhg

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------
* cleanup
* fixing dependencies
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------
* use component namespaces for light, mimic and say
* catkin_lint'ing
* Contributors: Florian Weisshardt, ipa-fmw

0.6.2 (2014-12-15)
------------------
* new names for mimic
* use wallpaper instead of fullscreen
* add tired mimic
* delete outdated bored mimic and add default
* final faces
* new mimic files
* add action for mimic node
* new faces
* update mimic videos
* delete outdated gifs
* install tags
* new faces
* fixed circle color mode
* the rate can  be a float
* tested on cob4-2
* redo cob_mimic
* removed pygame dependency
* updated cob_mimic
* rewrite script using os.system
* new package cob_mimic - First Version
* Contributors: Florian Weisshardt, bnm, ipa-cob4-2, ipa-fmw, ipa-nhg

* new names for mimic
* use wallpaper instead of fullscreen
* add tired mimic
* delete outdated bored mimic and add default
* final faces
* new mimic files
* add action for mimic node
* new faces
* update mimic videos
* delete outdated gifs
* install tags
* new faces
* fixed circle color mode
* the rate can  be a float
* tested on cob4-2
* redo cob_mimic
* removed pygame dependency
* updated cob_mimic
* rewrite script using os.system
* new package cob_mimic - First Version
* Contributors: Florian Weisshardt, bnm, ipa-cob4-2, ipa-fmw, ipa-nhg

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
