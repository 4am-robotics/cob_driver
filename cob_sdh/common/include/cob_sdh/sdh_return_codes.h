//======================================================================
/*!
  \file
  \section sdh_components_sdh_return_codes_h_general General file information
    \author   Dirk Osswald
    \date     2011-02-04


  \brief
    This file contains a typedef for a common Return Codes enum.


  \section sdh_components_sdh_return_codes_h_copyright Copyright

  Copyright (c) 2011 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdh_components_sdh_return_codes_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-02-08 14:53:00 +0100 (Di, 08 Feb 2011) $
      \par SVN file revision:
        $Id: sdh_return_codes.h 6432 2011-02-08 13:53:00Z Osswald2 $

  \subsection sdh_components_sdh_return_codes_h_changelog Changelog of this file:
      \include sdh_return_codes.h.log
*/
//======================================================================

#ifndef SDH_COMPONENTS_SDH_RETURN_CODES_h_
#define SDH_COMPONENTS_SDH_RETURN_CODES_h_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

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
    Packed (1 Byte) enum with binary return codes used to indicate the
    status of the SDH sent in the binary communiation request or response.

    \remark
      - To make this definition work with both C (nios-gcc) and C++ (std gcc/VCC)
        we have to separate the enum definition from the typedef.
      - You may use the corresponding typedef eReturnCode
*/
enum eReturnCodeEnum
{
    RC_OK = 0,                 //!< Success, no error
    RC_NOT_AVAILABLE,          //!< Error: An accessed ressource is not available
    RC_NOT_INITIALIZED,        //!< Error: An accessed ressource has not been initialized
    RC_ALREADY_RUNNING,        //!< Error: Data acquisition: the acquisition loop is already running
    RC_FEATURE_NOT_SUPPORTED,
    RC_INCONSISTENT_DATA,
    RC_TIMEOUT,                //!< Error: timeout occured
    RC_READ_ERROR,             //!< Error: could not read
    RC_WRITE_ERROR,            //!< Error: could not write
    RC_INSUFFICIENT_RESOURCES, //!< Error: Insufficient ressources
    RC_CHECKSUM_ERROR,
    RC_NOT_ENOUGH_PARAMS,      //!< Error: not enough parameters on command line
    RC_NO_PARAMS_EXPECTED,
    RC_CMD_UNKNOWN,            //!< Error: unknown command on command line
    RC_CMD_FORMAT_ERROR,       //!< Error: invalid format of command line parameters
    RC_ACCESS_DENIED,
    RC_ALREADY_OPEN,
    RC_CMD_FAILED,
    RC_CMD_ABORTED,
    RC_INVALID_HANDLE,
    RC_DEVICE_NOT_FOUND,
    RC_DEVICE_NOT_OPENED,
    RC_IO_ERROR,               //!< Error: Input/Output error like bus-off detected
    RC_INVALID_PARAMETER,      //!< Error: invalid parameter on command line
    RC_RANGE_ERROR,
    RC_NO_DATAPIPE,
    RC_INDEX_OUT_OF_BOUNDS,    //!< Error: A given index parameter is invalid
    RC_HOMING_ERROR,
    RC_AXIS_DISABLED,
    RC_OVER_TEMPERATURE,
    RC_MAX_COMMANDS_EXCEEDED,  //!< Error: cannot add more than CI_MAX_COMMANDS to interpreter / POSCON_MAX_OSCILLOSCOPE parameters to oscilloscope
    RC_INVALID_PASSWORD,       //!< Error: invalid password given for change user command
    RC_MAX_COMMANDLINE_EXCEEDED, //!< Error: the command line given is too long
    RC_CRC_ERROR,              //!< Cyclic Redundancy Code error while receiving binary input
    RC_NO_COMMAND,             //!< Not really an error: reading input did not yield a new command

    RC_INTERNAL,               //!< Error: callback function reports internal error
    RC_UNKNOWN_ERROR,          //!< Error: unknown error

    // Dont forget to add new enums to ReturnCodeToString() too.
    RC_DIMENSION          //!< End marker and dimension
} SDH__attribute__((__packed__));

//! typedef for eCommandCodeEnum, see there
typedef enum eReturnCodeEnum eReturnCode;

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
