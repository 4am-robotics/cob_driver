//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhbase_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSDHBase.

  \section sdhlibrary_cpp_sdhbase_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhbase_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
      \par SVN file revision:
        $Id: sdhbase.h 3686 2008-10-13 15:07:24Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhbase_h_changelog Changelog of this file:
      \include sdhbase.h.log
*/
//======================================================================

#ifndef SDHBASE_H_
#define SDHBASE_H_

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4290)
#endif

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include "iostream"

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "sdhexception.h"
#include "simplevector.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

extern std::ostream* g_sdh_debug_log;

//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

/*!
   \brief Derived exception class for exceptions related to invalid parameters.
*/
class cSDHErrorInvalidParameter: public cSDHLibraryException
{
public:
  cSDHErrorInvalidParameter( cMsg const & _msg )
    : cSDHLibraryException( "cSDHErrorInvalidParameter", _msg )
  {}
};
//======================================================================

/*!
  \brief The base class to control the SCHUNK Dexterous Hand.

   End-Users should \b NOT use this class directly, as it only provides
   some common settings and no function interface.
   End users should use the class cSDH instead, as it provides the
   end-user functions to control the SDH.

   <hr>
*/
class cSDHBase
{
public:

    //! Anonymous enum (instead of define like macros)
    enum {
        All = -1 //!< A meta-value that means "access all possible values"
    };


    /*!
      The error codes of the SDH firmware
       \internal
        In the SDH firmware these enums are of type DSA_STAT
    */
    enum eErrorCode
    {
        eEC_SUCCESS = 0,
        eEC_NOT_AVAILABLE = 1,
        eEC_NO_SENSOR = 2,
        eEC_NOT_INITIALIZED = 3,
        eEC_ALREADY_RUNNING = 4,
        eEC_FEATURE_NOT_SUPPORTED = 5,
        eEC_INCONSISTENT_DATA = 6,
        eEC_TIMEOUT = 7,
        eEC_READ_ERROR = 8,
        eEC_WRITE_ERROR = 9,
        eEC_INSUFFICIENT_RESOURCES = 10,
        eEC_CHECKSUM_ERROR = 11,
        eEC_NOT_ENOUGH_PARAMS = 12 ,
        eEC_NO_PARAMS_EXPECTED = 13,
        eEC_CMD_UNKNOWN = 14,
        eEC_CMD_FORMAT_ERROR = 15,
        eEC_ACCESS_DENIED = 16,
        eEC_ALREADY_OPEN = 17,
        eEC_CMD_FAILED = 18,
        eEC_CMD_ABORTED = 19,
        eEC_INVALID_HANDLE = 20,
        eEC_DEVICE_NOT_FOUND = 21,
        eEC_DEVICE_NOT_OPENED = 22,
        eEC_IO_ERROR = 23,
        eEC_INVALID_PARAMETER = 24,
        eEC_RANGE_ERROR = 25,
        eEC_NO_DATAPIPE = 26,
        eEC_INDEX_OUT_OF_BOUNDS = 27,
        eEC_HOMING_ERROR = 28,
        eEC_AXIS_DISABLED = 29,
        eEC_OVER_TEMPERATURE = 30,

        eEC_DIMENSION         //!< Endmarker and dimension
    };


    /*!
      \brief The enum values of the known grasps
       \internal
         In the SDH firmware these enums are of type TGRIP
    */
    enum eGraspId
    {
        // ??? fehlt e vor enums:
        eGID_INVALID = -1,    //!< invalid grasp id
        eGID_CENTRICAL = 0,   //!< centrical grasp:   ???
        eGID_PARALLEL = 1,    //!< parallel grasp:    ???
        eGID_CYLINDRICAL = 2, //!< cylindrical grasp: ???
        eGID_SPHERICAL = 3,   //!< spherecial grasp:  ???

        eGID_DIMENSION        //!< Endmarker and dimension
    };


    //! An enum for all possible SDH internal controller types
    enum eControllerType
    {
        eCT_POSE = 0,     //!< position controller (position per axis => "pose controller")

        eCT_DIMENSION         //!< Endmarker and dimension
    };


    //! An enum for all possible SDH internal velocity profile types
    enum eVelocityProfile
    {
        eVP_INVALID = -1,  //!< not a valid velocity profile, used to make #SDH::cSDHSerial::vp() read the velocity profile from the firmware
        eVP_SIN_SQUARE,    //!< sin square velocity profile
        eVP_RAMP,          //!< ramp velocity profile

        eVP_DIMENSION      //!< endmarker and dimension
    };

    //-----------------------------------------------------------------
    /*!
      Constructor of cSDHBase class, initilize internal variables and settings

       \param debug_level : debug level of the created object.
                            If the \a debug_level of an object is > 0 then
                            it will output debug messages.
       - (Subclasses of cSDHBase like cSDH or cSDHSerial use additional settings, see there.)

    */
    cSDHBase( int debug_level );


    //-----------------------------------------------------------------
    /*!
      virtual destructor to make compiler happy
    */
    virtual ~cSDHBase()
    {}


    //! Check if \a index is in [0 .. \a maxindex-1] or All. Throw a cSDHErrorInvalidParameter exception if not.
    void CheckIndex( int index, int maxindex, char const* name="" )
        throw (cSDHErrorInvalidParameter*);


    //! Check if \a value is in [\a minvalue .. \a maxvalue]. Throw a cSDHErrorInvalidParameter exception if not.
    void CheckRange( double value, double minvalue, double maxvalue, char const* name="" )
        throw (cSDHErrorInvalidParameter*);


    //! Check if any value[i] in array \a values is in [\a minvalue[i] .. \a maxvalue[i]]. Throw a cSDHErrorInvalidParameter exception if not.
    void CheckRange( double* values, double* minvalues, double* maxvalues, char const* name="" )
        throw (cSDHErrorInvalidParameter*);


    //! Return a ptr to a (static) string describing error code \a error_code
    static char const* GetStringFromErrorCode( eErrorCode error_code );


    //! Return a ptr to a (static) string describing grasp id \a grasp_id
    static char const* GetStringFromGraspId( eGraspId grasp_id );


    //! Return a ptr to a (static) string describing controller type \a controller_Type
    static char const* GetStringFromControllerType( eControllerType controller_type );


    //! Return the number of axes of the SDH
    int GetNumberOfAxes( void );


    //! Return the number of fingers of the SDH
    int GetNumberOfFingers( void );


    //! Return the number of temperature sensors of the SDH
    int GetNumberOfTemperatureSensors( void );


    //! Return the last known state of the SDH firmware
    eErrorCode GetFirmwareState( void );


    //! Return the #eps value
    double GetEps( void );


    //! Return simple vector of number of axes epsilon values
    cSimpleVector const & GetEpsVector( void );


    //! Return true if connection to SDH firmware/hardware is open
    virtual bool IsOpen( void ) = 0;

    //! change the stream to use for debug messages
    virtual void SetDebugOutput( std::ostream* debuglog )
    {
        cdbg.SetOutput( debuglog );
    }
protected:
    //! debug stream to print colored debug messages
    cDBG cdbg;


    //! A mapping from #eErrorCode error code enums to strings with human readable error messages
    static char const* firmware_error_codes[];


    //! A mapping from #eGraspId grasp id enums to strings with human readable grasp id names
    static char const* grasp_id_name[];


    //! A mapping from #eControllerType controller type enums to strings with human readable controller type names
    static char const* controller_type_name[];
    //---------------------

    //! The number of axes
    int NUMBER_OF_AXES;

    //! The number of fingers
    int NUMBER_OF_FINGERS;

    //! The number of temperature sensors
    int NUMBER_OF_TEMPERATURE_SENSORS;

    //! Bit field with the bits for all axes set
    int all_axes_used;

    //! the last known state of the SDH firmware
    eErrorCode firmware_state;

    /*!
      \brief epsilon value (max absolute deviation of reported values from actual hardware values)
      (needed since firmware limits number of digits reported)
    */
    double eps;

    //! simple vector of 7 epsilon values
    cSimpleVector eps_v;

    //! simple vector of 7 0 values ???
    //cSimpleVector zeros_v;

    //! simple vector of 7 1 values ???
    //cSimpleVector ones_v;

    //! Minimum allowed axis angles (in internal units (degrees))
    cSimpleVector min_angle_v;

    //! Maximum allowed axis angles (in internal units (degrees))
    cSimpleVector max_angle_v;

};

// end of class cSDHBase
//=====================================================================

NAMESPACE_SDH_END

#endif


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
