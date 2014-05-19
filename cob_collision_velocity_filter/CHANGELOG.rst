^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_collision_velocity_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* add definitions to get rid of compiler warning
* Corrected compile error. See pull-request `#80 <https://github.com/ipa320/cob_driver/issues/80>`_ in original repo.
* merge changes from frederikhegger, `#80 <https://github.com/ipa320/cob_driver/issues/80>`_
* change from gencpp (for msgs and srvs) to gencfg (for dyn recfg)
* changes for hydro
* Installation stuff
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* Color interpolation added to the velocity limited marker
* set max vel to 0.2
* set max velocity for marker to 0.3
* A new marker shown when rotation speed is limited
* remove debug output
* add markers to veclocity filter
* reduce max range
* fix divide by zero bug
* make cob_collision_velocity_filter dynamically reconfigureable
* move src files
* move launch and config files to cob_robots
* add laser_scan_top_clearing as additional observation_source
* add scan_top_clearing topic
* add ramp to limit acceleration
* change documentation
* merge launch files and move configs to same folder
* put GetFootprint service into footprint observer
  instead of SetFootprint service in collision velocity filter
  fix namespace problems
* only use laser data for velocity filter
* integrate safe velocity controller by default
* add missing dependency
* moved safe base movement to driver stack
* Contributors: Alexander Bubeck, Florian Weißhardt, Frederik Hegger, IPR-SR2, abubeck, but-spanel, ipa-fmw, ipa-mig, mig, srs
