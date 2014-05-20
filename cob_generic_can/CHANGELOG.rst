^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_generic_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* added missing install tags
* Some small dependency tweaks.
* fixed gcc4.7 build error (sleep and usleep undefined)
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* futher include and linkpath modifications
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* cob_generic_can: selectable __DEBUG__ output
* cob_generic_can: error message filtering
* cob_generic_can: restructured and added diagnostic outputs for CAN errors
* merge
* remove compiler error
* cob_generic_can: debug outputs filtered
* cob_generic_can: detecting heavy CAN-bus loads
* cob_generic_can: trying to detect BUSOFF of can and restart
* cob_generic_can: Starting debug on base crash error
* camera settings added for head
* cleanup in cob_driver
* added inifile strings for pcan devices
* cob_head_axis: correctly working, but front and back is switched
* update documentation and deleted tf broadcaster
* cleanup in stacks
* cleanup in cob_driver
* merge wit cpc
* After merging in review branch
* debugging cob_camera_axis; not yet running
* added windows.h; some modifications in ElmoCtrl -> not yet working
* added classes to implement ESD can-itf; incorporated ESD interface as an option in cob_base_drive_chain-node via CanCtrlPltfCOb3; added windows.h to cob_utilities package
* Updated Can Classes to new file structure; removed some leftovers; corrected comments at the beginning considering association to stacks and packages; moved Mutex.h to Utilities; - Debugged compiler error in cob_base_drive_chain
* renamed to cob_
* merged master
* renamed packages to cob_ convention
* Contributors: Alexander Bubeck, Christian Connette, Richard Bormann, abubeck, cob, cpc, cpc-pk, ipa-bnm, ipa-cpc, ipa-fmw
