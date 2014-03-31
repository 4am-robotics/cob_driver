^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_base_velocity_smoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* add definitions to get rid of compiler warning
* cob_undercarriage_ctrl: expose param for watchdog timeout
* cob_base_velocity_smoother: add param to specify minimal rate which is expected for commands. At slower rates, start filling in zeros
* cleaned up CMakeLists and added install directives
* cob_base_velocity_smoother: make robot stop if a zero is commanded
* cob_base_velocity_smoother: stop publishing velocity commands when we don't receive any
* futher include and linkpath modifications
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* bugfix: added missing default value for parameter in velocity smoother
* the cob_base_velocity_smoother now has a loop rate and some additional updates
* cleaned up the code
* some minor modifications on cob_base_velocity_smoother, removed some unnecessary couts
* some minor modifications on cob_base_velocity_smoother
* some minor modifications on cob_base_velocity_smoother
* some further modifications on cob_base_velocity_smoother, to be (parameter-) tested on hw
* some modifications on cob_base_velocity_smoother, to be tested on hw
* some bugfixes on cob_base_velocity_smoother, to be tested on hw. still contains some couts to be removed
* some modifications/bugfixes in cob_base_velocity_smoother.cpp. to be tested, not finished yet
* integration of cob_base_velocity_smoother, moved here from cob_navigation
* Contributors: Alexander Bubeck, Frederik Hegger, abubeck, frm, ipa-frm, ipa-mig
