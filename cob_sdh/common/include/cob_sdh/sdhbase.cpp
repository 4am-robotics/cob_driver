//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhbase_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of class #SDH::cSDHBase.

  \section sdhlibrary_cpp_sdhbase_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhbase_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-02-08 14:53:00 +0100 (Di, 08 Feb 2011) $
      \par SVN file revision:
        $Id: sdhbase.cpp 6432 2011-02-08 13:53:00Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhbase_cpp_changelog Changelog of this file:
      \include sdhbase.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhbase.h"
#include "util.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

NAMESPACE_SDH_START

std::ostream* g_sdh_debug_log = &std::cerr;

NAMESPACE_SDH_END

//----------------------------------------------------------------------
// Function implementation (function definitions)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class member definitions
//----------------------------------------------------------------------

char const* cSDHBase::firmware_error_codes[] =
    {
        // Order DOES matter here. The order must match that of the eErrorCode enums.
        "eEC_SUCCESS: No error",
        "eEC_NOT_AVAILABLE: Service or data is not available",
        "eEC_NOT_INITIALIZED: The device is not initialized",
        "eEC_ALREADY_RUNNING: Data acquisition: the acquisition loop is already running",
        "eEC_FEATUReEC_NOT_SUPPORTED: The asked feature is not supported",
        "eEC_INCONSISTENT_DATA: One or more dependent parameters mismatch",
        "eEC_TIMEOUT: Timeout error",
        "eEC_READ_ERROR: Error while reading from a device",
        "eEC_WRITE_ERROR: Error while writing to a device",
        "eEC_INSUFFICIENT_RESOURCES: No memory available",
        "eEC_CHECKSUM_ERROR: Checksum error",
        "eEC_NOT_ENOUGH_PARAMS: Not enough parameters",
        "eEC_NO_PARAMS_EXPECTED: No parameters expected",
        "eEC_CMD_UNKNOWN: Unknown command",
        "eEC_CMD_FORMAT_ERROR: Command format error",
        "eEC_ACCESS_DENIED: Access denied",
        "eEC_ALREADY_OPEN: Interface already open",
        "eEC_CMD_FAILED: Command failed",
        "eEC_CMD_ABORTED: Command aborted",
        "eEC_INVALID_HANDLE: Invalid handle",
        "eEC_DEVICE_NOT_FOUND: Device not found",
        "eEC_DEVICE_NOT_OPENED: Device not open",
        "eEC_IO_ERROR: General I/O-Error",
        "eEC_INVALID_PARAMETER: Invalid parameter",
        "eEC_RANGE_ERROR: Range error",
        "eEC_NO_DATAPIPE: No datapipe was found to open the specified device path",
        "eEC_INDEX_OUT_OF_BOUNDS: The passed index is out of bounds",
        "eEC_HOMING_ERROR: Error while homing",
        "eEC_AXIS_DISABLED: The selected axis is disabled",
        "eEC_OVER_TEMPERATURE: Over-temperature",
        "eEC_MAX_COMMANDS_EXCEEDED: E_MAX_COMMANDS_EXCEEDED: cannot add more than CI_MAX_COMMANDS to interpreter / POSCON_MAX_OSCILLOSCOPE parameters to oscilloscope",
        "eEC_INVALID_PASSWORD: E_INVALID_PASSWORD: invalid password given for change user command",
        "eEC_MAX_COMMANDLINE_EXCEEDEDE_COMMANDLINE_EXCEEDED: the command line given is too long",
        "eEC_CRC_ERROR: Cyclic Redundancy Code error",
        "eEC_NO_COMMAND: No command available",

        "eEC_INTERNAL: internal error",
        "eEC_UNKNOWN_ERROR: unknown error",

        "eEC_DIMENSION: Number of error codes"
    };

char const* cSDHBase::grasp_id_name[] =
    {
        // Order DOES matter here. The order must match that of the eGraspId enums.
        "eGID_CENTRICAL: centrical grasp",
        "eGID_PARALLEL: parallel grasp",
        "eGID_CYLINDRICAL: cylindrical grasp",
        "eGID_SPHERICAL: spherecial grasp",

        "eGID_DIMENSION: number of predefined grasp ids"
    };


char const* cSDHBase::controller_type_name[] =
    {
        // Order DOES matter here. The order must match that of the eErrorCode enums.
        "eCT_POSE: position/pose controller coordinated",
        "eCT_VELOCITY: velocity controller",
        "eCT_VELOCITY_ACCELERATION: velocity with acceleration ramp controller",

        //"eCT_POSITION: position controller, all axes independent",

        "eCT_DIMENSION: number of controller types"
    };
//-----------------------------------------------------------------

cSDHBase::cSDHBase( int _debug_level ) :

    // init members:
    cdbg( (_debug_level > 0), "magenta", g_sdh_debug_log ),
    debug_level(_debug_level),
    NUMBER_OF_AXES( 7 ),
    NUMBER_OF_FINGERS( 3 ),
    NUMBER_OF_TEMPERATURE_SENSORS( 9 )
{
    cdbg << "Constructing cSDHBASE object\n";


    all_axes_used = (1<<NUMBER_OF_AXES)-1;

    firmware_state = eEC_SUCCESS;

    eps = 0.5; // !!!

    for ( int i=0; i < NUMBER_OF_AXES; i++ )
    {
        eps_v[i] = eps;

        //zeros_v[i] = 0.0;
        //ones_v[i]  = 1.0;

        min_angle_v[i] = (i==0)? 0.0 : -90.0;
        max_angle_v[i] = 90.0;
    }
}
//-----------------------------------------------------------------


void cSDHBase::CheckIndex( int index, int maxindex, char const* name )
    throw (cSDHErrorInvalidParameter*)
{
    if (index < 0  ||  maxindex <= index)
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid %s index %d (not in range [0..%d[)", name, index, maxindex ) );
}
//-----------------------------------------------------------------


void cSDHBase::CheckRange( double value, double minvalue, double maxvalue, char const* name )
    throw (cSDHErrorInvalidParameter*)
{
    if (! InRange(value, minvalue, maxvalue) )
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid %s value (%f not in range [%f..%f])", name, value, minvalue, maxvalue ) );
}
//-----------------------------------------------------------------


void cSDHBase::CheckRange( double* values, double* minvalues, double* maxvalues, char const* name )
    throw (cSDHErrorInvalidParameter*)
{
    for ( int i=0; i < NUMBER_OF_AXES; i++ )
    {
        if (! InRange(values[i],minvalues[i],maxvalues[i]) )
            throw new cSDHErrorInvalidParameter( cMsg( "Invalid %s value in vector (values[%d]=%f not in range [%f..%f])", name, i, values[i], minvalues[i], maxvalues[i] ) );
    }
}
//-----------------------------------------------------------------


char const* cSDHBase::GetStringFromErrorCode( eErrorCode error_code )
{
    if ( 0 <= error_code  &&  error_code < eEC_DIMENSION )
        return firmware_error_codes[ error_code ];
    else
        return "invalid error code";
}
//----------------------------------------------------------------------


char const* cSDHBase::GetStringFromGraspId( eGraspId grasp_id )
{
    if ( eGID_INVALID < grasp_id  &&  grasp_id < eGID_DIMENSION )
        return grasp_id_name[ grasp_id ];
    else
        return "invalid gasp id";
}
//----------------------------------------------------------------------


char const* cSDHBase::GetStringFromControllerType( eControllerType controller_type )
{
    if ( eCT_INVALID < controller_type  &&  controller_type < eCT_DIMENSION )
        return controller_type_name[ controller_type ];
    else
        return "invalid controller type";
}
//----------------------------------------------------------------------


int cSDHBase::GetNumberOfAxes( void )
{
    return NUMBER_OF_AXES;
}
//----------------------------------------------------------------------


int cSDHBase::GetNumberOfFingers( void )
{
    return NUMBER_OF_FINGERS;
}
//----------------------------------------------------------------------


int cSDHBase::GetNumberOfTemperatureSensors( void )
{
    return NUMBER_OF_TEMPERATURE_SENSORS;
}
//----------------------------------------------------------------------


cSDHBase::eErrorCode cSDHBase::GetFirmwareState( void )
{
    return firmware_state;
}
//----------------------------------------------------------------------


double cSDHBase::GetEps( void )
{
    return eps;
}
//----------------------------------------------------------------------


cSimpleVector const & cSDHBase::GetEpsVector( void )
{
    return eps_v;
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================
