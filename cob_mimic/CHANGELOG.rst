^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_mimic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
