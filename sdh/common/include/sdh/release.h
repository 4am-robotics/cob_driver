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
      $LastChangedDate: 2008-10-17 10:56:55 +0200 (Fr, 17 Okt 2008) $
      \par SVN file revision:
        $Id: release.h 3725 2008-10-17 08:56:55Z Osswald2 $

  \subsection sdhlibrary_cpp_release_h_changelog Changelog of this file:
      \include release.h.log

 */
//----------------------------------------------------------------------

#ifndef _SDHLIBRARY_CPP_h_
#define _SDHLIBRARY_CPP_h_

//! Name of the software project.
/*!
    \anchor project_name_sdhlibrary_cpp
    The name of the "SDHLibrary-CPP" (C Library for accessing SDH from a PC) project.

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
    The release name of the "SDHLibrary-CPP" project.

    A suffix of "-dev" indicates a work in progress, i.e. a not yet finished release.
    A suffix of "-a", "-b", ... indicates a bugfix release.

    From newest to oldest the releases have the following names and features:

    - \b 0.0.1.4:
      - bufix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=267">Bug 267: SDHlib cannot be compiled on Linux</a>

    - \b 0.0.1.3: 2008-10-16
      - bufix: <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=266">Bug 266: demo-dsa does not work in the VCC version</a>

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
        - all SDH specific classes can be put in a namespace called "SDH"
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
        - A cSDH object could be opened successfully even if no SDH was
          connected or was connected but not powered:
          - added demo-test program to verify erroneous / repaired behaviour
          - added SetTimeout and GetTimeout to cRS232 class
          - made the code to verify proper connection to SDH work
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
      - Release modified at visit Uni-Whales
        - Changes to make everything work on Ubuntu-Linux
        - Enhanced Makefile a little bit to be more comfortable for the end user

    - \b 0.0.0.2: 2007-03-07
      - release, for Uni-Wales

    - \b 0.0.0.1
      - initial release, works for the first time, but not relyably

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
#define PROJECT_RELEASE "0.0.1.3"

//! Date of the release of the software project.
/*!
    \anchor project_date_sdhlibrary_cpp
    The date of the release of the project.
*/
#define PROJECT_DATE "2008-10-16"

#define PROJECT_COPYRIGHT "(c) SCHUNK GmbH & Co. KG, 2007"

#endif
//======================================================================
