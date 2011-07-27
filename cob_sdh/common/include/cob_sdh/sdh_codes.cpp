//======================================================================
/*!
\file
\section sdhlibrary_cpp_sdh_codes_cpp_general General file information
\author   Dirk Osswald
\date     2011-02-04


\brief
This file contains function to convert the binary command and return codes of the SDH to strings.

\section sdh_components_sdh_codes_cpp_copyright Copyright

- Copyright (c) 2011 SCHUNK GmbH & Co. KG

<HR>
\internal

\subsection sdh_components_sdh_codes_cpp_details SVN related, detailed file specific information:
$LastChangedBy: Osswald2 $
$LastChangedDate: 2011-02-08 14:31:14 +0100 (Di, 08 Feb 2011) $
\par SVN file revision:
$Id: sdh_codes.cpp 6431 2011-02-08 13:31:14Z Osswald2 $

\subsection sdh_components_sdh_codes_cpp_changelog Changelog of this file:
\include sdh_codes.cpp.log
*/
//======================================================================


//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdh_codes.h"
#include "util.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

char const* NS_SDH SDHCommandCodeToString( eCommandCode cc )
{
    switch (cc)
    {
    DEFINE_TO_CASECOMMAND( CMDC_V );
    DEFINE_TO_CASECOMMAND( CMDC_VEL );
    DEFINE_TO_CASECOMMAND( CMDC_RVEL );
    DEFINE_TO_CASECOMMAND( CMDC_POS );
    DEFINE_TO_CASECOMMAND( CMDC_STATE );
    DEFINE_TO_CASECOMMAND( CMDC_P );
    DEFINE_TO_CASECOMMAND( CMDC_A );
    DEFINE_TO_CASECOMMAND( CMDC_M );
    DEFINE_TO_CASECOMMAND( CMDC_STOP );
    DEFINE_TO_CASECOMMAND( CMDC_VP );
    DEFINE_TO_CASECOMMAND( CMDC_CON );
    DEFINE_TO_CASECOMMAND( CMDC_TPAP );
    DEFINE_TO_CASECOMMAND( CMDC_TVAV );
    DEFINE_TO_CASECOMMAND( CMDC_VLIM );
    DEFINE_TO_CASECOMMAND( CMDC_ALIM );
    DEFINE_TO_CASECOMMAND( CMDC_POS_SAVE );
    DEFINE_TO_CASECOMMAND( CMDC_REF );
    DEFINE_TO_CASECOMMAND( CMDC_TEMP );
    DEFINE_TO_CASECOMMAND( CMDC_ID );
    DEFINE_TO_CASECOMMAND( CMDC_SN );
    DEFINE_TO_CASECOMMAND( CMDC_VER );
    DEFINE_TO_CASECOMMAND( CMDC_VER_DATE );
    DEFINE_TO_CASECOMMAND( CMDC_SOC );
    DEFINE_TO_CASECOMMAND( CMDC_SOC_DATE );
    DEFINE_TO_CASECOMMAND( CMDC_NUMAXIS );
    DEFINE_TO_CASECOMMAND( CMDC_P_MIN );
    DEFINE_TO_CASECOMMAND( CMDC_P_MAX );
    DEFINE_TO_CASECOMMAND( CMDC_P_OFFSET );
    DEFINE_TO_CASECOMMAND( CMDC_GET_DURATION );
    DEFINE_TO_CASECOMMAND( CMDC_IGRIP );
    DEFINE_TO_CASECOMMAND( CMDC_IHOLD );
    DEFINE_TO_CASECOMMAND( CMDC_SELGRIP );
    DEFINE_TO_CASECOMMAND( CMDC_GRIP );
    DEFINE_TO_CASECOMMAND( CMDC_PID );
    DEFINE_TO_CASECOMMAND( CMDC_KV );
    DEFINE_TO_CASECOMMAND( CMDC_ILIM );
    DEFINE_TO_CASECOMMAND( CMDC_POWER );
    DEFINE_TO_CASECOMMAND( CMDC_DEMO );
    DEFINE_TO_CASECOMMAND( CMDC_USER_ERRORS );
    DEFINE_TO_CASECOMMAND( CMDC_TERMINAL );
    DEFINE_TO_CASECOMMAND( CMDC_DEBUG );
    DEFINE_TO_CASECOMMAND( CMDC_USE_FIXED_LENGTH );
    DEFINE_TO_CASECOMMAND( CMDC_CHANGE_RS232 );
    DEFINE_TO_CASECOMMAND( CMDC_CHANGE_CHANNEL );

#if USE_CMD_TEST

    DEFINE_TO_CASECOMMAND( CMDC_TEST );
#endif

    // no default here to make compiler warn about unhandled cases
    }
    return "Unknown SDH command code!";
}
//----------------------------------------------------------------------

char const* NS_SDH SDHReturnCodeToString( eReturnCode rc )
{
    switch (rc)
    {
    DEFINE_TO_CASECOMMAND( RC_OK );
    DEFINE_TO_CASECOMMAND( RC_NOT_AVAILABLE );
    DEFINE_TO_CASECOMMAND( RC_NOT_INITIALIZED );
    DEFINE_TO_CASECOMMAND( RC_ALREADY_RUNNING );
    DEFINE_TO_CASECOMMAND( RC_FEATURE_NOT_SUPPORTED );
    DEFINE_TO_CASECOMMAND( RC_INCONSISTENT_DATA );
    DEFINE_TO_CASECOMMAND( RC_TIMEOUT );
    DEFINE_TO_CASECOMMAND( RC_READ_ERROR );
    DEFINE_TO_CASECOMMAND( RC_WRITE_ERROR );
    DEFINE_TO_CASECOMMAND( RC_INSUFFICIENT_RESOURCES );
    DEFINE_TO_CASECOMMAND( RC_CHECKSUM_ERROR );
    DEFINE_TO_CASECOMMAND( RC_NOT_ENOUGH_PARAMS );
    DEFINE_TO_CASECOMMAND( RC_NO_PARAMS_EXPECTED );
    DEFINE_TO_CASECOMMAND( RC_CMD_UNKNOWN );
    DEFINE_TO_CASECOMMAND( RC_CMD_FORMAT_ERROR );
    DEFINE_TO_CASECOMMAND( RC_ACCESS_DENIED );
    DEFINE_TO_CASECOMMAND( RC_ALREADY_OPEN );
    DEFINE_TO_CASECOMMAND( RC_CMD_FAILED );
    DEFINE_TO_CASECOMMAND( RC_CMD_ABORTED );
    DEFINE_TO_CASECOMMAND( RC_INVALID_HANDLE );
    DEFINE_TO_CASECOMMAND( RC_DEVICE_NOT_FOUND );
    DEFINE_TO_CASECOMMAND( RC_DEVICE_NOT_OPENED );
    DEFINE_TO_CASECOMMAND( RC_IO_ERROR );
    DEFINE_TO_CASECOMMAND( RC_INVALID_PARAMETER );
    DEFINE_TO_CASECOMMAND( RC_RANGE_ERROR );
    DEFINE_TO_CASECOMMAND( RC_NO_DATAPIPE );
    DEFINE_TO_CASECOMMAND( RC_INDEX_OUT_OF_BOUNDS );
    DEFINE_TO_CASECOMMAND( RC_HOMING_ERROR );
    DEFINE_TO_CASECOMMAND( RC_AXIS_DISABLED );
    DEFINE_TO_CASECOMMAND( RC_OVER_TEMPERATURE );
    DEFINE_TO_CASECOMMAND( RC_MAX_COMMANDS_EXCEEDED );
    DEFINE_TO_CASECOMMAND( RC_INVALID_PASSWORD );
    DEFINE_TO_CASECOMMAND( RC_MAX_COMMANDLINE_EXCEEDED );
    DEFINE_TO_CASECOMMAND( RC_CRC_ERROR );
    DEFINE_TO_CASECOMMAND( RC_NO_COMMAND );
    DEFINE_TO_CASECOMMAND( RC_INTERNAL );
    DEFINE_TO_CASECOMMAND( RC_UNKNOWN_ERROR );
    DEFINE_TO_CASECOMMAND( RC_DIMENSION );

    // no default here to make compiler warn about unhandled cases
    }
    return "Unknown SDH return code!";
}
//----------------------------------------------------------------------


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
