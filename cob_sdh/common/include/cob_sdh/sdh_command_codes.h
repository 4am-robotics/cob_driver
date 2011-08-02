//======================================================================
/*!
  \file
  \section sdh_components_sdh_command_codes_h_general General file information
    \author   Ilshat Mamaev, Dirk Osswald
    \date     2011-02-01


  \brief
    This file contains the binary command codes of the SDH.
    To use this from a non gcc compiler you might have to
    define SDH__attribute__ to nothing and SDH_USE_VCC to 1.


  \section sdh_components_sdh_command_codes_h_copyright Copyright

  - Copyright (c) 2003 IPR, University of Karlsruhe (TH), all rights reserved
  - Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdh_components_sdh_command_codes_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-02-04 19:26:19 +0100 (Fr, 04 Feb 2011) $
      \par SVN file revision:
        $Id: sdh_command_codes.h 6420 2011-02-04 18:26:19Z Osswald2 $

  \subsection sdh_components_sdh_command_codes_h_changelog Changelog of this file:
      \include sch_command_codes.h.log
*/
//======================================================================


#ifndef _SDH_COMMAND_CODES
#define _SDH_COMMAND_CODES

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

#define USE_CMD_TEST 0


// since this file is included by other compilers which dont understand
// attributes we can only use it if SDH__attribute__ is not yet defined
#ifndef SDH__attribute__
# define SDH__attribute__( ... ) __attribute__(__VA_ARGS__)
#endif

#ifndef SDH_USE_VCC
# define SDH_USE_VCC 0
#endif


#if SDH_USE_VCC
#pragma pack(push,1)  // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif

/*!
    Packed (1 Byte) enum with binary command codes used to indicate the
    command sent in the binary communiation request or response.

    \remark
      - Not all commands are implemented as binary commands
      - To make this definition work with both C (nios-gcc) and C++ (std gcc/VCC)
        we have to separate the enum definition from the typedef.
      - You may use the corresponding typedef eCommandCode
*/
enum eCommandCodeEnum
{
    // Movement commands:
    CMDC_V = 128,   // assign a value so that non ASCII codes are used

    CMDC_VEL,
    //130:
    CMDC_RVEL,
    CMDC_POS,
    CMDC_STATE,
    CMDC_P,
    CMDC_A,
    CMDC_M,
    CMDC_STOP,
    CMDC_VP,
    CMDC_CON,
    CMDC_TPAP,
    //140:
    CMDC_TVAV,

    // Diagnostic and identification commands:
    CMDC_VLIM,
    CMDC_ALIM,
    CMDC_POS_SAVE,
    CMDC_REF,
    CMDC_TEMP,
    CMDC_ID,
    CMDC_SN,
    CMDC_VER,
    CMDC_VER_DATE,
    //150:
    CMDC_SOC,
    CMDC_SOC_DATE,
    CMDC_NUMAXIS,
    CMDC_P_MIN,
    CMDC_P_MAX,
    CMDC_P_OFFSET,
    CMDC_GET_DURATION,

    // Grip commands:
    CMDC_IGRIP,
    CMDC_IHOLD,
    CMDC_SELGRIP,
    //160:
    CMDC_GRIP,

    // Setup and configuration commands:
    CMDC_PID,
    CMDC_KV,
    CMDC_ILIM,
    CMDC_POWER,

    // Misc. commands:
    CMDC_DEMO,
    CMDC_USER_ERRORS,
    CMDC_TERMINAL,
    CMDC_DEBUG,
    CMDC_USE_FIXED_LENGTH,
    //170:
    CMDC_CHANGE_RS232,
    CMDC_CHANGE_CHANNEL,

#if USE_CMD_TEST

    CMDC_TEST,
#endif
} SDH__attribute__((packed));

//! typedef for eCommandCodeEnum, see there
typedef enum eCommandCodeEnum eCommandCode;

#if SDH_USE_VCC
#pragma pack(pop)   // for VCC (MS Visual Studio) restore normal packing
#endif

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External functions (function declarations)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


#endif


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
*/
//======================================================================
