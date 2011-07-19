//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_release_h_general General file information
    \author   Dirk Osswald
    \date     2006-11-30

  \brief

    This file contains nothing but C/C++ defines with the name of the
    project itself (\ref project_name_sdhlibrary_cpp "PROJECT_NAME") and the name of the release
    (\ref project_release_sdhlibrary_cpp "PROJECT_RELEASE") of the whole project.

    For a general description of the project see \ref sdhlibrary_cpp_dox_general "general project information".


  \section sdhlibrary_cpp_release_h_copyright Copyright

  Copyright (c) 2006 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_release_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-05-10 13:55:42 +0200 (Di, 10 Mai 2011) $
      \par SVN file revision:
        $Id: release.h 6819 2011-05-10 11:55:42Z Osswald2 $

  \subsection sdhlibrary_cpp_release_h_changelog Changelog of this file:
      \include release.h.log

 */
//----------------------------------------------------------------------

#ifndef _SDHLIBRARY_CPP_h_
#define _SDHLIBRARY_CPP_h_

//! Name of the software project.
/*!
    \anchor project_name_sdhlibrary_cpp
    The name of the "SDHLibrary-CPP" (C Library for accessing %SDH from a PC) project.

    \internal
    \remark
    - This name is extracted by the makefile and then used by doxygen:
      - As name of the project within the generated documentation.
      - As base name of the generated install directory and pdf files.
    - The name should \b NOT contain spaces!
 */
#define PROJECT_NAME "SDHLibrary-CPP"

//! Release name of the whole software project (a.k.a. as the \e "version" of the project).
/*!
    \anchor project_release_sdhlibrary_cpp
    The release name of the "SDHLibrary-CPP" project. The doxygen comment below
    contains the changelog of the project.

    A suffix of "-dev" indicates a work in progress, i.e. a not yet finished release.
    A suffix of "-a", "-b", ... indicates a bugfix release.

    From newest to oldest the releases have the following names and features:

    - \b 0.0.2.3:
      - working <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=1012">Bug 1012: Bug: cannot communicate with SDH via PEAK-CAN on Linux</a>
        => testing compliance of SDHLibrary with PEAK CAN driver v7.1 for Linux
        - cannot be confirmed as a bug
        - minor bug in cCANSerial_PEAK::Open() now only the read-ID is filtered instead of both write and read
        - slightly improved cancat
        - improved debug output cCANSerial_PEAK and cCANSeral_ESD while

    - \b 0.0.2.2: 2011-04-27
      - fixed <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=1011">Bug 1011: Bug: Binary communication does not work via RS232 on Linux</a>
        The ISTRIP (Strip parity bits) bit MUST NOT be set on linux.
        If set then the MSbit of received bytes is set to 0 always...
        (Thanks to the hints of M. Schoepfer from Uni Bielefeld!)
      - In distribution CAN bus and boost support is now disabled by default.
        (Can be enabled in Makefile-settings, see WITH_ESD_CAN, WITH_PEAK_CAN, HAVE_BOOST)

    - \b 0.0.2.1: 2011-03-09
      - fixed <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=844">Bug 844: Bug: Korean windows cannot handle specific unit characters like ° and ²</a>
        The non ASCII string litearals for degrees and squared were replaced by ASCII literals
      - added <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=996">Bug 996: Task: Version numbering of DSACON32m firmware has changed since 2011-02-15</a>
        software version numbers for DSACON32m are reported correctly for the new firmwares
      - fixed <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=703">Bug 703: Bug: Tactile sensor frames cannot be read reliably in single frame mode</a>
        - Some firmware versions of the DSACON32m are not able to do single frame acquisition and enter push-mode. This might fill up the RS232 input buffer and leads to problems like reading of outdated frames or frame loss.
        - Added workaround to stop push mode immediately after it was entered unintentionally
        - made debug output of communication functions (RS232 and CAN and TCP) more concise
      - Enhancement <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=985">Bug 985: Task: Get Visual Studio to compile SDHLibrary-c++ as DLL</a>
        - The Visual Studio projects now generate and use a DLL version of the SDHLibrary

    - \b 0.0.2.0: 2011-02-08
      - added new documentation file SDH2_exchanging_tactile_sensor_controller
      - added support for communication via TCP (requires at least SDH firmware 0.0.3.0)
        - enhancement <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=874">Bug 874: Task: Enable TCP communication in SDHLibrary</a>
        - refactoring <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=918">Bug 918: Task: Refactor generation of error messages</a>
      - added properties for "Release" configuration in VCC project files
      - bugifx <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=959">Bug 959: Bug: SDHLibrary does not compile in new cygwin-1.7 environment</a>
        - refactored device driver specific CAN stuff from headers into code files
      - added demo-benchmark for measuring communication performance <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=963">Bug 963: Task: implement benchmark for measuring communication rate</a>
      - added support for fast binary communication with CRC protection <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=964">Bug 964: Task: implement new binary communication in SDHLibrary</a>
      - added commands to set target angles/velocities and get actual angles/velocities in one command,
        see cSDH::SetAxisTargetGetAxisActualAngle() and cSDH::SetAxisTargetGetAxisActualVelocity()
      - <b>These last two points along with the accompanying changes in the firmware boost the communication speed significantly!</b>
        - For RS232 the communication cycles per second rise from ca. 30 to 75 Hz (according to demo-benchmark)
        - For CAN the improvement goes even higher to ca. 160 Hz (according to demo-benchmark)
      - Doxygen documentation is now generated with doxygen-1.7.3 with included javascript search engine

    - \b 0.0.1.18:
      - added bug description for <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=822"> Bug 822: Online help of C++ demo programs compiled with VCC misses description for CAN parameters</a>
      - added new documentation file SDH2_FingerDimensions

    - \b 0.0.1.17: (2010-05-11)
      - enhancement <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=706"> Task: Reorganise Makefiles to simplify making of user provided demo-programs</a>
        extracted common Makefile settings to Makefile-settings for better maintenance and extensibility,
      - Enhancement made acquiring of single tactile sensor frames available

    - \b 0.0.1.16: (2010-04-12)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=680"> Bug 680: cDSA fails to communicate with Release 276 of DSACON32m Firmware</a>

    - \b 0.0.1.15: (2010-03-05)
       - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=482"> Bug 482: add support for adjusting sensitivity and threshold in SDHLibrary-CPP</a>
         - added cDSA.SetMatrixSensitivity() cDSA.GetMatrixSensitivity() cDSA.SetMatrixThreshold() cDSA.GetMatrixThreshold()
         - the sensitivity and the threshold can now be changed and saved temporarily or persistently to configuration memory of the DSACON32m
         - added command line options to demo-dsa to adjust matrix sensitivity and threshold (needs DSACON32m Firmware >= R268)

    - \b 0.0.1.14: (2010-02-19)
      - added doucmentation file SDH2_exchanging_tactile_sensors.pdf to distribution
      - added doucmentation file SDH2_wiring_test_board.pdf to distribution
      - added doucmentation file SDH training.pdf to distribution
      - updated doucmentation file SDH2_configuration-and-update.pdf

    - \b 0.0.1.13: (2010-02-02)
      - bugfix (firmware 0.0.2.10): <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=630">Bug 630: Bug: setting of pid parameters does not work</a>

    - \b 0.0.1.12: (2009-12-04)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=575">Bug 575: grip related commands do not work properly</a>
        - the actual answers for selgrip/grip from the SDH were different from the expected ones in sdhserial
        - there was a problem with the read ahead data in readline in serialbase
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=470">Bug 470: made demo-contact-grasping.cpp actually work</a>

    - \b 0.0.1.11: (2009-12-01)
      - added support for CAN devices from PEAK, kindly provided by Steffen Ruehl and Zhixing Xue
        - this caused some changes and additions in the command line option handling
        - made the "Linux-only" PEAK port work on Windows as well
      - fixed bug in both ESD and PEAK CAN support that might prevent proper access to several
        SDHs on same CAN interface from the same process. (Problem was static data in communication classes)
      - added online help of demonstration programs to doxygen documentation
      - changed Makefile to use 'uname' instead of unreliable OSTYPE to determine OSNAME
      - added demo-contact-grasping to show how to do very basic reactive grasping
        (using boost::threads to evaluate tactile sensor data concurrently)
      - added demo-mimic to show how to access 2 SDHs, one mimicing the movements of the other
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=561">Bug 561: SDHLibray does not compile with VCC</a>
        - the pragma pack was not used correctly, i.e. packing was not reset to normal after the structs to pack tightly
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=568"> Bug 568: option -M causes segfault on demo-dsa</a>
      - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=480">Bug 480: Include DSACON32 firmware within distribution</a>

    - \b 0.0.1.10: (unofficial release 2009-10-05)
      - added troubleshooting section to the CD contents in troubleshooting.html
      - corrected checking of environment variable "OS" for the automatic disabling
        of the use of color in output. OS is "WINNT" on German windows XP,
        but "Windows_NT" on US-english Windows XP... Phhh
      - while fixing <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=452">Bug 452: current demo-gui.py fails when connected to an old SDH v0.0.2.0</a>
        added firmware version specific code to make cSDH::GetAxisLimitVelocity() and cSDH::GetAxisLimitAcceleration() work on older firmwares
      - made all operator<< receive a const reference as parameter. This should improve performance especially for the cDBG::operator<<
      - bugfix: trying to connect to the tactile sensors of an SDH that is missing or switched off caused an infinite retry loop
      - Bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=351">Bug 351: make does not work recursively</a>
        reopend and fixed again since still had problems on Linux where the shell command "test" or "[]" is somewhat picky about comparing strings ("==" is not understood)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=471">Bug 471: Incorporate feedback for 64 bit compatibility</a>
        changes to make compilation work on 64 bit systems according to feedback from Niklas Bergstroem.
      - enhancement: Added possibility to set the RS232 device name format string (e.g. to use /dev/ttyUSB%d like devices)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=469">Bug 469: make clean funktioniert unter Linux nicht</a>
      - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=472"> Bug 472: Objects of type cDSA should be able to recover from power cycling</a>
        There is now a cDSA::Open() member to reopen the connection to the DSA controller after a failure
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=478>Bug 478: Online help of C++ demo programs is incorrect</a>
      - modified Makefile-doc, Makefile, Doxyfile to be able to use target specific variables to exclude/include files from documentation depending on whether internal or external docu is generated


    - \b 0.0.1.9: 2009-06-17
      - added missing includes of cstdio for gcc-4.4 as reported by Hannes Saal.
      - while adding webcheck to check the generated html files:
        - corrected settings in Doxyfile so that tagfiles not available for customer are no longer used in distribution
        - corrected broken/missing links in the distribution html files according to webcheck
      - added forgotten demo-velocity-acceleration project for VCC
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=433"> Bug 433: Invalid negative velocities remain set when switching from speed based controllers back to pose controller</a>
        Adjusted documentation for SetController() accordingly
      - adjusted Library for new behaviour of firmware 0.0.2.7 in eCT_VELOCITY_ACCELERATION controller type:
        - acceleration must no longer be given with correct sign. The sign of the acceleration
          is now determined automatically from the signs and magniturdes of the current reference velocity and the target velocity
        - Adjusted WaitAxis() since the state is now reported correctly by the firmware, even if in a speed based controller mode
        - adjusted doxygen documentation and demo-velocity-acceleration.cpp
        - current controller_type is now cached in cSDH object
        - Now using the same acceleration limits as the firmware
      - Date of library is now reported as well for option -v in the demo programs
      - corrected doxygen description of GetTemperature()
      - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=442">Bug 442: acceleration limits cannot be queried from the firmware</a>
        - added new commands to read acceleration limits from firmware
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=432">Bug 432: temperature output does not respect the -F Fahrenheit switch from the comand line</a>
      - enhancement: updated / corrected doxygen comments
        - updated known bugs
        - guarded text "SDH" with "%SDH" in doxygen comments to prevent doxygen from auto-linking to SDH namespaceup

    - \b 0.0.1.8: 2009-05-18
      - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=263">Enhancement 263: Provide access to speed controller of %SDH joints</a>
        - provide access to the 2 additional controller types eCT_VELOCITY and eCT_VELOCITY_ACCELERATION which are provided by the firmware 0.0.2.6
        - added new command cSDH.GetAxisReferenceVelocity() to access the internal reference velocity of the eCT_VELOCITY_ACCELERATION controller type
        - added new demonstration script demo-velocity-acceleration.cpp
        - the allowed lower limits for velocity and acceleration now have to be adjusted when the controller type changes.
      - enhancement <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=384"> Enhancement 384: Make doxygen documentation match for SDHLibrary-CPP and SDHLibrary-python</a>
        - added group with all demonstration programs
        - included description of all known bugs

    - \b 0.0.1.7: 2009-05-05
      - Enhancement: added CAD Datafiles to distribution CD
      - Bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=341">Bug 341: new gcc-4.x capable Makefile does not work in Linux</a>
        Linux gcc does not know option --enable-auto-import, added cygwin specific code to Makefile
      - Bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=351">Bug 351: make does not work recursively</a>
        building the library did not work in one go since make did not work recursively due to errors in Makefile-subdir
        PIPESTATUS was used which is an automatic variable of the bash shell.
        So if another shell is used then PIPESTATUS cannot be used to determine if the
        recursively called make succeded. Therefore we now export PIPESTATUS="0" (which
        means 'true' for shells) to keep make going on non bash shells.
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=342"> Bug 342: RS232 communication does not work any more when compiled with MS-Visual Studio on Windows</a>
        Resolved, parameters like bits per byte were not set explicitly, so the code worked with some interfaces only, depending on the default settings of the interface
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=361"> Bug 361: No proper RS232 communication in C++ version of SDHLibrary</a>
        Resolved, timeout handling improved
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=364"> Bug 364: Remove ancient, no current stuff from the distribution; Separate build target into build_lib build_demo (and build_test)</a>
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=366"> Bug 366: demo-dsa.exe does not respect --debuglog</a>
      - bugfix: corrected output of internal datatypes memebers of type UInt8. These are now printed as hex instead of as char
      - enhancement: debug outputs of low level communication in rs232-cygwin.cpp and rs232-vcc.cpp can be
        included / excluded at compile time using the SDH_RS232_CYGWIN_DEBUG / SDH_RS232_VCC_DEBUG macros.
        If included the debug messages can be enabled / disabled at runtime with the usual -d LEVEL parameter. (Level 4 is needed for these in the demos)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=372"> Bug 372: Disabled colored debug output on windows consoles</a> for better readability
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=370"> Bug 370: First call to demo-dsa after powering SDH fails </a>
        The problem is related to the tactile sensor controller DSACON32m within the
        %SDH. This controller needs approximately 8 seconds to "boot" up, and during
        that time it will not answer to requests from the SDHLibrary. Unfortunately the
        timeout mechanism of the SDHLibrary for Windows also had a bug which
        then made it wait forever for answers that would never come.
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=373">Bug 373: EmergencyStop did not work</a>
        Fixed (copy & paste error while porting from python to C++)
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=379">Bug 379: invalid command line parameters cause segv in demo-dsa</a>
        Fixed, missing ':' in option definition string
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=386">Bug 386: demo-simple2.cpp does not behave as expected</a>
        Fixed, some code was commented out
      - velocity and acceleration for virtual axis is no longer limited to [0..0], since that makes SetAxisTargetVelocity( All, x ) and SetAxisTargetAcceleration( All, x ) invalid for all x != 0.0

    - \b 0.0.1.6:
      - extracted generation of distribution stuff from Makefile to Makefile-dist (since not needed by customer)
      - corrected use of CC and CPPC variables in Makefiles. Now these can be set from the environment. Needed to test compilation with alternative compilers like gcc-4.x instead of std gcc-3.x
      - corrected copy & paste errors in doxygen comments of cSDH::GetAxisMaxVelocity()
      - Bugfix <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=333">Bug 333: Invalid default parameters and documentation for OpenESD_CAN</a>
        Mix-up of hex and dec representation of the default IDs used for CAN communication
      - made compilation work without errors and warnings with gcc-4.3.2:
        - added additional includes like "cstring"
        - changed many char* to char const* to get rid of deprecation warnings
      - changed generation of distribution:
        - modified Doxyfile is now included so that user can generate documentation by himself
        - doc target is no longer a subtarget of all target in distributed Makefile (user will most likely not want to regenerate the docu)
      - updated links to misc packages and Weiss documentation in index-overview.html in distribution

    - \b 0.0.1.5: 2009-02-11
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=322">Bug 322: AxisTargetVelocity cannot be set higher than 100 deg/s</a>
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=323">Bug 323: SDHLibrary exceptions not deleted correctly when caught and handled</a>
      - enhancement: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=315">Enhancement 315: Add documentation files to distribution</a>

    - \b 0.0.1.4:
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=267">Bug 267: SDHlib cannot be compiled on Linux</a>

    - \b 0.0.1.3: 2008-10-16
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=266">Bug 266: demo-dsa does not work in the VCC version</a>

    - \b 0.0.1.2: 2008-10-14
      - bugfix: target cancat is now only added in the Makefile if WITH_ESD_CAN is 1
      - bugfix: corrected copy/paste error: removed class qualifiers from member declarations since
                newer gccs do not accept fully qualified member names within class declarations (like aClass::aMember())
      - bugfix: corrected parameter checking of cSDHSerial::vp()
      - bugfix: corrected error in apply() in util.h (correct result was calculated only locally)
      - made generation of Doxygen-doku work with Doxygen v1.5.5
      - fixed bug in option detection
      - made output of version info more verbose (with SOC and dates)
      - implemented cRS232::Read()
      - made dsa.cpp/h work with VCC
      - corrected TCP calculation: corrected limb lengths and changed coordinate system from left to right handed
      - added cSDH::GetInfo
      - added cSDHSerial::vlim() and cSDH::GetAxisLimitVelocity() to read velocity limits
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=134">Bug 134: cannot generate doxygen documentation if SDH namespace is used</a>
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=261">Bug 261: Header file problems with SDHLibrary-CPP on Linux </a>
      - bugfix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=260">Bug 260: CAN access problems with SDHLibrary-CPP on Linux </a>
      - enhancement: Enable debugging to logfile in SDHLibrary-cpp
      - enhancement: reduced overhead in cDBG::operator<<, no more searching for color strings on each call
      - added demo-simple-withtiming to perform some simple OS-level time measurement
      - change: changed parent class of cSerialBaseException to cSDHErrorCommunication to make the hierarchy more consistent

    - \b 0.0.1.1:
      - added cancat program for sending inaccessible commands like change_rs232 via CAN
      - added info commands corresponding to new info commands in firmware:
        - in sdhserial:
          - soc - to read the SoC ID
          - soc_date : to read the date string of the SoC
          - ver_date : to read the release date of the firmware
        - in sdh:
          - enhanced GetInfo to read all the above also
        - in the option parser in auxilliary all the info is now printed if "-v" is given
      - added GetDuration command: returns the calculate duration of the currently configured movement
        (target angle, velocity, acceleration, velocity profile) but does not execute the movement.
        This simplifies scripts like sdhrecord.py a great deal.
      - made py.test test_sdh work again.
      - while working on <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=224">Bug 224: Positioning delay of 5s when using SDHLibrary-CPP</a>
        - removed call to SleepSec() in loop in serial for VCC, needed to get rid of delays
        - implemented cSimpleTime in VCC version
      - added support for new firmware v0.0.2.0 commands
        - soc, soc_date, ver_date
        - GetDuration

    - \b 0.0.1.0: 2008-06-13
      - added basic support for the tactile sensors,
        - new library classes cDSA, cCRC
        - new demo program demo-dsa

    - \b 0.0.0.9: 2008-06-06
      - added missing files for vcc compilation in distribution
      - removed error in overloaded Sleep function. Internal version renamed to SleepSec.
      - made demo-GetAxisActualAngle and demo-temperature work in periodic mode in Visual Studio
      - untabified all source and header files for use with Visual Studio
      - autostart feature for distribution CD

    - \b 0.0.0.8-a:
      - added project files for the demo-* programs to the Visual Studio solutions file to make the
        demo programs available under VCC too
      - added forgotten WITH_ESD_CAN=1 to Visual Studio project file for SDHLibrary
      - workaround for accessing RS232 from VCC (WriteFile() does not return number of bytes sent)
      - With VCC the communication via RS232 still has some bugs: long pauses and timeouts,
      - With VCC the "-t" parameter for periodic replies in demos does not work yet

    - \b 0.0.0.8:  2008-05-26
      - added compatibility code for MS Visual C++ Compiler (VCC)
        - sdhlibrary_defines with compatibility macros
        - pragmas for VCC
        - _attribute_ are switched off for VCC
        - all %SDH specific classes can be put in a namespace called "SDH"
      - added index-overview.html with overview of distributed files
      - index.html files in distribution are parsed for ${PROJECT_*}

    - \b 0.0.0.7-b: 2008-05-21
      - CAN timeout is now correctly set
      - fixed bug in cpp/Makefile: OSNAME_LINUX=1 was always appended to EXTRACPPFLAGS no matter what OS was used
      - renamed interace member to comm_interface

    - \b 0.0.0.7-a: 2008-05-17
      - bug fix: minor changes to make compilation work on Linux. (ESDs ntcan.h for Windows is different from that for Linux)
      - still contains a bug that can be fixed without recompilation:
        - The Linux version of ntcan canOpen does not accept timeout values < 0
        - Workaround for demo programs: Use an additional "-T 0.0" command line parameter

    - \b 0.0.0.7: 2008-05-16
      - added C++ support for CAN using ESD cards
        - restructured low level communication:
          - new base class cSerialBase (+new exception classes cSerialBaseException, cCANSerialESDException)
      - added support for variable baudrate for RS232 communication
      - made command line option handling in c++ demo programs more generic
      - corrected a bug in SDHLibrary-CPP that caused a SEGV (empty throw statement outside of a catch block)

    - \b 0.0.0.6: 2007-12-27
      - release for RoboCluster, Denmark
        - added ref command and demo-ref program (needed for SDH-003)
        - corrected minor errors to make the above work

    - \b 0.0.0.5-a: 2007-06-06
      - Included bugfixes from release 0.0.0.3-a (bugfix release for
        Uni Wales, see below) into release for care-o-bot

    - \b 0.0.0.5: 2007-05-24
      - Release for care-o-bot (IPA, Stuttgart), mai 2007
      - Restructured files: library stuff into sdh/ and demo programs in demo/ to ease installation on user platform
      - Added library support for the new firmware features:
        - cSDHSerial: a() vp(), vel()
        - cSDH: Get/SetAxisAcceleration(), GetAxisMaxAcceleration(), Get/SetVelocityProfile(), GetAxisCurrentVelocity()
      - while preparing release for IPA care-o-bot:
        - since line endings are corrected in firmware now removed the special EOL treatmnt in readline
        - enhanced generation of distribution
        - extended README files
        - added demo-simple3 in cpp and python
        - made compilation work on linux without warnings (SuSE 8.1 and Knoppix_v5.1.1)
        - added requested functions GetAxisActualState() and WaitAxis() in cpp and python library
        - added eAxisState enums from firmware
        - corrected some yet undetected errors
        - corrected / enhanced some doxygen comments
        - tried to find bug:
          - firmware not moving from 5,-5,0,0,0,0,0 to 20,0,0,0,0,0,0:
          - axis 1 is stuck at 1.4...
          - bug could not be resolved (does not happen for for larger movements)

    - \b 0.0.0.4: 2007-03-19
      - Release for demo at NASA, march 2007
        - adjusted expected lines for "m" command (it now prints one line debug output for every axis)

    - \b 0.0.0.3-a: 2007-06-05
      - Release modified according to bug report from Martin Huelse
        - A cSDH object could be opened successfully even if no %SDH was
          connected or was connected but not powered:
          - added demo-test program to verify erroneous / repaired behaviour
          - added SetTimeout and GetTimeout to cRS232 class
          - made the code to verify proper connection to %SDH work
        - Exceptions could not be caught properly:
          - corrected some real printf-style format string related
            problems in creations of exceptions
          - added gcc style printf-style format string checking (to
            enable the compiler to detect errors like the above at
            compile time)
          - The following piece of information from the C++
            Annotations was not considered properly.
            See <a href="http://www.icce.rug.nl/documents/cplusplus/">http://www.icce.rug.nl/documents/cplusplus/</a>:
            "A function for which a function throw list is specified
            may not throw other types of exceptions. A run-time
            error occurs if it tries to throw other types of
            exceptions than those mentioned in the function throw
            list."
          - corrected the function throw lists/exception
            specification lists so that the most generic type thrown
            was listed. Thus all the user-level functions now just
            mention cSDHLibraryException* in their function throw
            list as all thrown exceptions are derived from it.
      - Further changes
        - added further function throw lists/exception specification lists
        - merged in some changes from newer releases (up to 0.0.0.5):
          - retrying of sending in case of transmission errors
          - naming of sequential / non sequential (formerly called synchronous/asynchronous)
          - fixed many typos in doxygen comments

    - \b 0.0.0.3: 2007-03-09
      - Release modified at visit Uni-Wales
        - Changes to make everything work on Ubuntu-Linux
        - Enhanced Makefile a little bit to be more comfortable for the end user

    - \b 0.0.0.2: 2007-03-07
      - release, for Uni-Wales

    - \b 0.0.0.1
      - initial release, works for the first time, but not reliably

    \internal
    \remark
    - This \b is the name/number of a release of the software project i.e.
      the thing that is shipped to end-users and customers. So this is
      the name that these people later refer to when they complain about
      bugs or missing features.
    - This is \b NOT the subversion release number of a specific file.
    - There \b should be a corresponding TAG name of the project in the
      subversion repository.
    - This release name/number is extracted by the makefile and then used
      by doxygen:
      - As release name/number of the project within the generated documentation.
      - As part of the name of the generated install directory and pdf files.
 */
#define PROJECT_RELEASE "0.0.2.3"

//! Date of the release of the software project.
/*!
    \anchor project_date_sdhlibrary_cpp
    The date of the release of the project.
*/
#define PROJECT_DATE "2011-05-11"

#define PROJECT_COPYRIGHT "(c) SCHUNK GmbH & Co. KG, 2007-2011"

#endif
//======================================================================
