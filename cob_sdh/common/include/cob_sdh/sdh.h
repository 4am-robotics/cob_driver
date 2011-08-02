//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdh_h_general General file information
    \author   Dirk Osswald
    \date     2007-02-20


  \brief
    This file contains the interface to class #SDH::cSDH, the end user
    class to access the %SDH from a PC.


  \section sdhlibrary_cpp_sdh_h_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdh_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-04-26 17:40:10 +0200 (Di, 26 Apr 2011) $
      \par SVN file revision:
        $Id: sdh.h 6744 2011-04-26 15:40:10Z Osswald2 $

  \subsection sdhlibrary_cpp_sdh_h_changelog Changelog of this file:
      \include sdh.h.log
*/
//======================================================================

#ifndef SDH_h_
#define SDH_h_

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4996)
#else
// must be included first on some linuxes
extern "C" {
# include <stdint.h>
}
#endif

#include "basisdef.h"

  //----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <vector>
#include <string>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhbase.h"
#include "sdhserial.h"
#include "unit_converter.h"
#include "serialbase.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

#if SDH_USE_NAMESPACE
/*!
 * \namespace SDH
 * A namespace for all classes and functions in the SDHLibrary.
 *
 * The use of the namespace can be disabled at compile time of the
 * library by setting #SDH_USE_NAMESPACE to 0.
 */
#endif

NAMESPACE_SDH_START

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions and classes (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

#if SDH_USE_VCC
// these are needed by VCC for template instantiation into a DLL,
// see http://support.microsoft.com/default.aspx?scid=kb;EN-US;q168958
// and http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
//
VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::allocator<int>;
VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::vector<int,std::allocator<int> >;

VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::allocator<std::vector<int> >;
VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::vector<std::vector<int>,std::allocator<std::vector<int> > >;

VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::allocator<double>;
VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::vector<double,std::allocator<double> >;

VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::allocator<std::vector<double> >;
VCC_EXPORT_TEMPLATE template class VCC_EXPORT std::vector<std::vector<double>,std::allocator<std::vector<double> > >;
#endif

//======================================================================

/*!
  \brief #SDH::cSDH is the end user interface class to control a %SDH (SCHUNK Dexterous Hand).

   A general overview of the structure and architecture used is given \ref
   sdhlibrary_cpp_architecture_dox_sdhpackage_overview "here".

   \remark
   \anchor sdhlibrary_cpp_sdh_h_csdh_axis_vs_fingers
   - The cSDH class provides methods to access the 7 axes of the %SDH
     individually as well as on a finger level.
     - When accessing the axes individually then the following axis
       indices must be used to address an axis / some axes:
       - 0 : common base axis of finger 0 and 2
       - 1 : proximal axis of finger 0
       - 2 : distal axis of finger 0
       - 3 : proximal axis of finger 1
       - 4 : distal axis of finger 1
       - 5 : proximal axis of finger 2
       - 6 : distal axis of finger 2
     - When accessing the axes on finger level then every finger has 3
       axes for a uniform interface of the access methods. Her the
       following finger axis indices must be used:
       - 0 : base axis of finger (for finger 1 this is a "virtual" axis with min angle = max angle = 0.0)
       - 1 : proximal axis of finger
       - 2 : distal axis of finger
   \anchor sdhlibrary_cpp_sdh_h_csdh_vector
   - Vector-like parmeters: The interface functions defined here make
     full use of the flexibility provided by the STL vector<T>
     type. I.E. for parameters of functions like axis indices or axis
     angles not only single numerical values can be given, but also
     vectors of \c int or \c double values. This way the same (overloaded)
     interface function can address a single axis individually or
     multiple axes in a call, as required by the application. Such
     parameters are herein refered to as "vectors".
   - Parameters for methods are checked for validity. In case an
     invalid parameter is given the method throws a
     cSDHErrorInvalidParameter exception.
   \anchor sdhlibrary_cpp_sdh_h_csdh_unit
   - The underlying physical unit system of parameters that do have a
     unit (like angles, velocities or temperatures) can be adapted to
     the users or the applications need. See also \ref
     sdhlibrary_cpp_sdh_h_unit_conversion_objects "unit conversion
     objects". The default converter objects are set as the uc_*
     member variables (#uc_angle, #uc_angular_velocity, #uc_angular_acceleration, #uc_time,
     #uc_temperature, #uc_position). The units are changed in the
     communication between user application and cSDH object instance
     only (USERAPP and SDHLibrary-CPP in the \ref
     sdhlibrary_cpp_architecture_dox_sdhpackage_overview "overview
     figure"). For now the %SDH firmware knows only about its internal unit
     system.

   <hr>
*/
class VCC_EXPORT cSDH : public cSDHBase
{
public:

    //! the motor current can be set specifically for these modes:
    enum eMotorCurrentMode
    {
        eMCM_MOVE=0, //!< The motor currents used while "moving" with a MoveHand() or MoveFinger() command
        eMCM_GRIP=1, //!< The motor currents used while "gripping" with a GripHand() command
        eMCM_HOLD=2, //!< The motor currents used after "gripping" with a GripHand() command (i.e. "holding")

        eMCM_DIMENSION //!< Endmarker and Dimension
    };


    //! The state of an axis (see TPOSCON_STATE in global.h of the %SDH firmware)
    enum eAxisState
    {
        eAS_IDLE = 0,            //!< axis is idle
        eAS_POSITIONING,         //!< the goal position has not been reached yet
        eAS_SPEED_MODE,          //!< axis is in speed mode
        eAS_NOT_INITIALIZED,     //!< axis is not initialized or doesn't exist
        eAS_CW_BLOCKED,          //!< axis is blocked in counterwise direction
        eAS_CCW_BLOCKED,         //!< axis is blocked is blocked in against counterwise direction
        eAS_DISABLED,            //!< axis is disabled
        eAS_LIMITS_REACHED,      //!< position limits reached and axis stopped

        eAS_DIMENSION //!< Endmarker and Dimension
    };


    //#######################################################################
    /*!
        \anchor sdhlibrary_cpp_sdh_h_unit_conversion_objects
         \name   Predefined unit conversion objecs

         Some predefined cUnitConverter unit conversion objects to
         convert values between different unit systems. These are static members since the converter objects do not
         depend on the individiual cSDH object.

         For every physical unit used in the cSDH class there is at
         least one (most of the time more than one) predefined unit
         converter. For example for angles there are \e radians and \e degrees.

         @{
    */

    //! Default converter for angles (internal unit == external unit): degrees
    static cUnitConverter const uc_angle_degrees;

    //! Converter for angles: external unit = radians
    static cUnitConverter const uc_angle_radians;

    //! Default converter for times (internal unit == external unit): seconds
    static cUnitConverter const uc_time_seconds;

    //! Converter for times: external unit = milliseconds
    static cUnitConverter const uc_time_milliseconds;

    //! Default converter for temparatures (internal unit == external unit): degrees celsius
    static cUnitConverter const uc_temperature_celsius;

    //! Converter for temperatures: external unit = degrees fahrenheit
    static cUnitConverter const uc_temperature_fahrenheit;

    //! Default converter for angular velocities (internal unit == external unit): degrees / second
    static cUnitConverter const uc_angular_velocity_degrees_per_second;

    //! Converter for angular velocieties: external unit = radians/second
    static cUnitConverter const uc_angular_velocity_radians_per_second;

    //! Default converter for angular accelerations (internal unit == external unit): degrees / second
    static cUnitConverter const uc_angular_acceleration_degrees_per_second_squared;

    //! Converter for angular velocieties: external unit = radians/second
    static cUnitConverter const uc_angular_acceleration_radians_per_second_squared;

    //! Default converter for motor current (internal unit == external unit): Ampere
    static cUnitConverter const uc_motor_current_ampere;

    //! Converter for motor current: external unit = milli Ampere
    static cUnitConverter const uc_motor_current_milliampere;

    //! Default converter for position (internal unit == external unit): millimeter
    static cUnitConverter const uc_position_millimeter;

    //! Converter for position: external unit = meter
    static cUnitConverter const uc_position_meter;


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_unit_conversion_objects
    //! @}

protected:
    //---------------------
    // Misc member variables

    //! The number of axis per finger (for finger 1 this includes the "virtual" base axis)
    int NUMBER_OF_AXES_PER_FINGER;


    //! The number of virtual axes
    int NUMBER_OF_VIRTUAL_AXES;

    //! The number of all axes including virtual axes
    int nb_all_axes;

    //! Mapping of finger index to number of real axes of fingers:
    std::vector<int> finger_number_of_axes;


    //! Mapping of finger index, finger axis index to axis index:
    std::vector<std::vector<int> > finger_axis_index;

    //! Vector of 3 epsilon values
    //f_eps_v;

    //! Vector of 3 0.0 values
    std::vector<double> f_zeros_v;

    //! Vector of 3 1.0 values
    std::vector<double> f_ones_v;


    //! Vector of nb_all_axes 0.0 values
    std::vector<double> zeros_v;

    //! Vector of nb_all_axes 1.0 values
    std::vector<double> ones_v;


    //! Minimum allowed motor currents (in internal units (Ampere)), including the virtual axis
    std::vector<double> f_min_motor_current_v;

    //! Maximum allowed motor currents (in internal units (Ampere)), including the virtual axis
    std::vector<double> f_max_motor_current_v;



    //! Minimum allowed axis angles (in internal units (degrees)), including the virtual axis
    std::vector<double> f_min_angle_v;

    //! Maximum allowed axis angles (in internal units (degrees)), including the virtual axis
    std::vector<double> f_max_angle_v;

    //! Minimum allowed axis velocity (in internal units (degrees/second)), including the virtual axis
    std::vector<double> f_min_velocity_v;

    //! Maximum allowed axis velocity (in internal units (degrees/second)), including the virtual axis
    std::vector<double> f_max_velocity_v;

    //! Minimum allowed axis acceleration (in internal units (degrees/(second * second))), including the virtual axis
    std::vector<double> f_min_acceleration_v;

    //! Maximum allowed axis acceleration (in internal units (degrees/(second * second))), including the virtual axis
    std::vector<double> f_max_acceleration_v;

    //! Maximum allowed grip velocity (in internal units (degrees/second))
    double grip_max_velocity;

    /*!
      \anchor sdhlibrary_cpp_sdh_h_kinematic_vars
       \name   Kinematic parameters of the Hand

       @{
    */

    //! length of limb 1 (proximal joint to distal joint) in mm
    double l1;

    //! length of limb 2 (distal joint to fingertip) in mm
    double l2;

    // distance between center points of base joints f0<->f1, f1<->f2, f0<->f2
    double d;

    // height of center of base joints above finger base plate
    double h;

    /*!
        list of xyz-vectors for all fingers with offset from (0,0,0) of proximal joint in mm

    */
    std::vector<std::vector<double> > offset;

    cSerialBase* com;

    // !!! make this public for now (to access comm_interface->ref() / comm_interface->pos_save() for hands with missing absolute encoders)
public:
    //! The object to interface with the %SDH attached via serial RS232 or CAN or TCP.
    cSDHSerial comm_interface;

    //! change the stream to use for debug messages
    virtual void SetDebugOutput( std::ostream* debuglog )
    {
        cSDHBase::SetDebugOutput( debuglog );
        comm_interface.SetDebugOutput( debuglog );
    }

protected:

    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_internal
       \name   Internal helper methods

       @{
    */

    //----------------------------------------------------------------------
    /*!
        Generic set function: set some given axes to given values

        \param axes       - a vector of axis indices
        \param values     - a vector of values
        \param ll_set     - a pointer to the low level set function to use
        \param ll_get     - a pointer to the low level get function to use (for those axes where the given value is NaN)
        \param uc         - a pointer to the unit converter object to use before sending values to \a ll_set
        \param min_values - a vector with the minimum allowed values
        \param max_values - a vector with the maximum allowed values
        \param name       - a string with the name of the values (for constructing error message)

        \return the values returned from the SDH will be returned
           (for most commands ll_set/ll_get functions this will be the \a values, except
            for the cSDHSerial::tvav and cSDHSerial::tpap functions)

        \remark
        - The length of the \a axis and \a values vector must match.
        - The indices can be given in any order, but the order of the
          elements of \a axes and \a values must
          match too. I.e. \c values[i] will be applied
          to axis \c axes[i] (not axis \c i)
        - The indices are checked if they are valid axis indices.
        - The values are checked if they are in the allowed range
          [\a min_values .. \a f_max_values], i.e. it is checked that \c
          value[i], converted to the internal unit system by \a uc->ToInternal(),
          is in [\a min_values[axes[i]] .. \a max_values[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified values is
          sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter* exception is thrown.
    */
    std::vector<double> SetAxisValueVector( std::vector<int> const& axes,
                                            std::vector<double> const& values,
                                            pSetFunction ll_set,
                                            pGetFunction ll_get,
                                            cUnitConverter const* uc,
                                            std::vector<double> const& min_values,
                                            std::vector<double> const& max_values,
                                            char const* name )
                                            throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Generic get function: get some given axes values

        \param axes   - a vector of axis indices
        \param ll_get - a pointer to the low level get function to use
        \param uc     - a pointer to the unit converter object to apply before returning values
        \param name   - a string with the name of the values (for constructing error message)

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the addressed values for the selected axes.
        - The values are converted to external unit system using the
          \a uc unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).
    */
    std::vector<double> GetAxisValueVector( std::vector<int> const& axes,
                                            pGetFunction ll_get,
                                            cUnitConverter const* uc,
                                            char const* name )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Internal helper function: return a vector of checked indices according to index.

        \param index           - The index to vectorize or #All
        \param all_replacement - a vector to return if \a index is #All
        \param maxindex        - the \a index is checked if in [0..\ maxindex[ (i.e. not including \a maxindex)
        \param name            - A name for the things index, used to report out of bounds errors

        \return
        - If \a index is #All then \a all_replacement is returned.
        - If \a index is a single number >= 0 then it is checked if in
          [0..\ maxindex[ and a vector of length 1 is returned containing
          only \a index.
        - In case \a index exceeds \a maxindex a (cSDHErrorInvalidParameter*) exception is thrown.
    */
    std::vector<int> ToIndexVector( int index, std::vector<int>& all_replacement, int maxindex, char const* name )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Internal helper function: return the get/set function of the
        comm_interface object that is responsible for setting/getting
        motor currents in \a mode.
    */
    pSetFunction GetMotorCurrentModeFunction( eMotorCurrentMode mode )
        throw(cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        return cartesian [x,y,z] position in mm of fingertip for finger fi at angles r_angles (rad)
    */
    std::vector<double> _GetFingerXYZ( int fi, std::vector<double> r_angles )
        throw(cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_internal
    //! @}
    //#####################################################################


public:
    //----------------------------------------------------------------------
    /*!
        \anchor sdhlibrary_cpp_sdh_h_index_vectors
        \name   Predefined index vector objects

         @{
    */

    //! A vector with indices of all axes (in natural order), including the virtual axis.
    std::vector<int> all_axes;

    //! A vector with indices of all real axes (in natural order), excluding the virtual axis.
    std::vector<int> all_real_axes;

    //! A vector with indices of all fingers (in natural order)
    std::vector<int> all_fingers;

    //! A vector with indices of all temperature sensors
    std::vector<int> all_temperature_sensors;

    //  end of sdhlibrary_cpp_sdh_h_index_vectors
    //! @}
    //----------------------------------------------------------------------


    //----------------------------------------------------------------------
    /*!
        \anchor sdhlibrary_cpp_sdh_h_unit_conversion_ptrs
        \name   Predefined unit conversion objects

         Pointers to the unit converter objects used by this cSDH object.

         The refered objects convert values between different unit
         systems. Example: convert angle values between \e degrees and
         \e radians, temperatures between \e degrees \e celsius and \e degrees
         \e fahrenheit or the like.

         A cSDH object uses these converter objects to convert between external (user)
         and internal (%SDH) units. The user can easily change the converter object
         that is used for a certain kind of unit. This way a cSDH object
         can easily report and accept parameters in the user or application
         specific unit system.

         Additionally, users can easily add conversion objects for their own,
         even more user- or application-specific unit systems.

         @{
    */

    //! unit convert for (axis) angles: default = #SDH::cSDH::uc_angle_degrees
    const cUnitConverter* uc_angle;


    //! unit convert for (axis) angular velocities: default = #SDH::cSDH::uc_angular_velocity_degrees_per_second
    const cUnitConverter* uc_angular_velocity;


    //! unit convert for (axis) angular accelerations: default = #SDH::cSDH::uc_angular_acceleration_degrees_per_second_squared
    const cUnitConverter* uc_angular_acceleration;


    //! unit convert for times: default = uc_time_seconds
    const cUnitConverter* uc_time;


    //! unit convert for temperatures: default = #SDH::cSDH::uc_temperature_celsius
    const cUnitConverter* uc_temperature;


    //! unit converter for motor curent: default = #SDH::cSDH::uc_motor_current_ampere
    const cUnitConverter* uc_motor_current;


    //! unit converter for position: default = #SDH::cSDH::uc_position_millimeter
    const cUnitConverter* uc_position;
    //---------------------

    // end of sdhlibrary_cpp_sdh_h_unit_conversion_ptrs
    //! @}
    //----------------------------------------------------------------------


    //-----------------------------------------------------------------
    /*!
      \brief Constructor of cSDH class.

       Creates an new object of type cSDH. One such object is needed
       for each %SDH that you want to control.  The constructor
       initializes internal data structures. A connection the %SDH is
       \b not yet established, see #OpenRS232() on how to do that.

       After an object is created the user can adjust the unit systems
       used to set/report parameters to/from %SDH. This is shown in the
       example code below. The default units used (if not overwritten
       by constructor parameters) are:
       - \c degrees [deg] for (axis) angles
       - \c degrees \c per \c second [deg/s] for (axis) angular velocities
       - \c seconds [s] for times
       - \c degrees \c celsius [deg C] for temperatures

       \param _use_radians    : Flag, if true then use radians and radians/second
                                to set/report (axis) angles and angular velocities
                                instead of default degrees and degrees/s.
       \param _use_fahrenheit : Flag, if true then use degrees fahrenheit
                                to report temperatures instead of default degrees celsius.
       \param _debug_level    : The level of debug messages to print
                                - 0: (default) no messages
                                - 1: messages of this cSDH instance
                                - 2: like 1 plus messages of the inner cSDHSerial instance

       \par Examples:

       Common use:
       \code
         // Include the cSDH interface
         #include <sdh.h>

         // Create a cSDH object 'hand'.
         cSDH hand();
       \endcode

       The mentioned change of a unit system can be done like this:
       \code
         // Assuming 'hand' is a cSDH object ...

         // override default unit converter for (axis) angles:
         hand.uc_angle = &cSDH::uc_angle_radians;

         // override default unit converter for (axis) angular velocities:
         hand.uc_angular_velocity = &cSDH::uc_angular_velocity_radians_per_second;

         // override default unit converter for (axis) angular accelerations:
         hand.uc_angular_acceleration = &cSDH::uc_angular_acceleration_radians_per_second_squared;

         // instead of the last 3 calls the following shortcut could be used:
         hand.UseRadians();

         // override default unit converter for times:
         hand.uc_time  = &cSDH::uc_time_milliseconds;

         // override default unit converter for temperatures:
         hand.uc_temperature = &cSDH::uc_temperature_fahrenheit;

         // override default unit converter for positions:
         hand.uc_position = &cSDH::uc_position_meter;

       \endcode


       For convenience the most common settings can be specified as
       bool parameters for the constructor, like in:

       \code
         // Include the cSDH interface
         #include <sdh.h>

         // Create a cSDH object 'hand' that uses
         // - the non default radians and radians/s units,
         // - the default temperature in degrees celsius,
         // - A debug level of 2
         cSDH hand( true, false, 2 );

       \endcode

       <hr>
    */
    cSDH( bool _use_radians=false, bool _use_fahrenheit=false, int _debug_level=0 );


    //----------------------------------------------------------------------
    /*!
        Virtual destructor to make compiler happy

        If the connection to the %SDH hardware/firmware is still open
        then the connection is closed, which will stop the axis
        controllers (and thus prevent overheating).
    */
    virtual ~cSDH();


    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_misc
       \name   Miscellaneous methods

       @{
    */

    //----------------------------------------------------------------------
    //! Return \c true if index \a iAxis refers to a virtual axis
    bool IsVirtualAxis( int iAxis )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
      Shortcut to set the unit system to radians.

       After calling this axis angles are set/reported in radians and
       angular velocities are set/reported in radians/second

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // make hand object use radians and radians/second for angles and angular velocities
         hand.UseRadians();

       \endcode

       <hr>
    */
    void UseRadians( void );


    //-----------------------------------------------------------------
    /*!
      Shortcut to set the unit system to degrees.

       After calling this (axis) angles are set/reported in degrees and
       angular velocities are set/reported in degrees/second

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // make hand object use degrees and degrees/second for angles and angular velocities
         hand.UseDegrees();
         // as degrees, degrees/second are the default this is needed only if the
         // unit system was changed before

       \endcode

       <hr>
    */
    void UseDegrees( void );


    //-----------------------------------------------------------------
    /*!
      Return the number of real axes of finger with index \a iFinger.

       \param iFinger - index of finger in range [0..NUMBER_OF_FINGERS-1]

       \return
       - Number of real axes of finger with index \a iFinger
       - If \a iFinger is invalid a (cSDHErrorInvalidParameter*) exception is thrown.

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         cout << "The finger 0 has " << hand.GetFingerNumberOfAxes( 0 ) << " real axes\n";

       \endcode

       <hr>
    */
    int GetFingerNumberOfAxes( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
      Return axis index of \a iFingerAxis axis of finger with index iFinger

       For \a iFinger=2, iFingerAxis=0 this will return the index of
       the virtual base axis of the finger

       \param iFinger     - index of finger in range [0..NUMBER_OF_FINGERS-1]
       \param iFingerAxis - index of finger axis in range [0..NUMBER_OF_AXES_PER_FINGER-1]

       \return
       - Axis index of \a iFingerAxis-th axis of finger with index \a iFinger
       - If \a iFinger or \a iFingerAxis is invalid a (cSDHErrorInvalidParameter*) exception is thrown.

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         cout << "The 1st axis of finger 2 has real axis index " << hand.GetFingerNumberOfAxes( 2, 0 ) << "\n";

       \endcode

       <hr>
    */
    int GetFingerAxisIndex( int iFinger, int iFingerAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
      Return the release name of the library (not the firmware of the %SDH) as string.


       \par Examples:
       \code
         // static member functon, so no cSDH object is needed for access:

         cout << "The SDHLibrary reports release name " << cSDH::GetReleaseLibrary() << "\n";

       \endcode

       <hr>
    */
    static char const* GetLibraryRelease( void );


    //-----------------------------------------------------------------
    /*!
      Return the name of the library as string.


       \par Examples:
       \code
         // static member functon, so no cSDH object is needed for access:

         cout << "The SDHLibrary reports name " << cSDH::GetLibraryName() << "\n";

       \endcode

       <hr>
    */
    static char const* GetLibraryName( void );


    //-----------------------------------------------------------------
    /*!
      Return the release name of the firmware of the %SDH (not the library) as string.

      This will throw a (cSDHErrorCommunication*) exception if the
      connection to the %SDH is not yet opened.

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         cout << "The SDH firmware reports release " << hand.GetFirmwareRelease() << "\n";

       \endcode

       <hr>
    */
    char const* GetFirmwareRelease( void )
        throw (cSDHLibraryException*);

    /*! Return info according to \a what
    #
    #  The following values are valid for \a what:
    #  - "date-library"     : date of the SDHLibrary-python release
    #  - "release-library"  : release name of the sdh.py python module
    #  - "release-firmware" : release name of the %SDH firmware (requires
    #                         an opened communication to the %SDH)
    #  - "date-firmware"    : date of the %SDH firmware (requires
    #                         an opened communication to the %SDH)
    #  - "release-soc"      : release name of the %SDH SoC (requires
    #                         an opened communication to the %SDH)
    #  - "date-soc"         : date of the %SDH SoC (requires
    #                         an opened communication to the %SDH)
    #  - "id-sdh"           : ID of %SDH
    #  - "sn-sdh"           : Serial number of %SDH
    #
    #  \par Examples:
    #  \code
    #    # Assuming 'hand' is a sdh.cSDH object ...
    #
    #    print "The SDH firmware reports release %s" % ( hand.GetInfo( "release-firmware" ) )
    #
    #  \endcode
    #
    #  <hr>
    */
    char const* GetInfo( char const* what )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return temperature(s) measured within the %SDH.

        \param sensors - A vector of indices of temperature sensors to access.
                       - index 0 is sensor near motor of axis 0 (root)
                       - index 1 is sensor near motor of axis 1 (proximal finger 1)
                       - index 2 is sensor near motor of axis 2 (distal finger 1)
                       - index 3 is sensor near motor of axis 3 (proximal finger 2)
                       - index 4 is sensor near motor of axis 4 (distal finger 2)
                       - index 5 is sensor near motor of axis 5 (proximal finger 3)
                       - index 6 is sensor near motor of axis 6 (distal finger 3)
                       - index 7 is FPGA temperature (controller chip)
                       - index 8 is PCB temperature (Printed Circuit Board)

        \remark
        - The indices in \a sensors are checked if they are valid sensor indices.
        - If \b any sensor index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        To access a single temperature sensor use #GetTemperature(int), see there.

        \return
          The temperatures of the selected sensors are returned as
          std::vector<double> in the configured temperature unit system
          #uc_temperature.


        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // Get measured values of all sensors
          std::vector<double> temps = hand.GetTemperature( hand.all_temperature_sensors );
          // Now temps is something like { 38.500,37.250,35.750,37.250,33.500,36.500,32.250,59.625,52.500 }

          // Get controller temperature only:
          double temp_controller = hand.GetTemperature( 0 );
          // Now temp_controller is something like 40.5

          // If we - for some obscure islandish reason - would want
          // temperatures reported in degrees fahrenheit, the unit
          // converter can be changed:
          hand.uc_temperature = &cSDH::uc_temperature_fahrenheit;

          // Get all temperaturs again:
          temps = hand.GetTemperature( hand.all_temperature_sensors );
          // Now temps is something like {100.0, 96.8, 92.3, 97.7, 91.8, 96.8, 90.1,  137.5,  125.2}

        \endcode

        <hr>
    */
    std::vector<double> GetTemperature( std::vector<int> const& sensors )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetTemperature(std::vector<int>const&), just for one sensor
        \a iSensor and returning a single temperature as double.
    */
    double GetTemperature( int iSensor )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_common
    //! @}
    //#####################################################################


    //######################################################################
    /*!
       \anchor sdhlibrary_cpp_sdh_h_csdh_communication
       \name   Communication methods

       @{
    */

    //-----------------------------------------------------------------
    /*!
      Open connection to %SDH via RS232.

       \param _port    : The number of the serial port to use.
                         The default value port=0 refers to 'COM1' in Windows and
                         to the corresponding '/dev/ttyS0' in Linux.
       \param _baudrate: the baudrate in bit/s, the default is 115200 which happens
                         to be the default for the %SDH too
       \param _timeout : The timeout to use:
                         - -1 : wait forever
                         - T  : wait for T seconds
       \param _device_format_string : a format string (C string) for generating the device name, like "/dev/ttyS%d" (default) or "/dev/ttyUSB%d".
                                     Must contain a %d where the port number should be inserted.
                                     This char array is duplicated on construction
                                     When compiled with VCC (MS-Visual C++) then this is not used.
       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // Open connection to SDH via default port:
         hand.OpenRS232();

         // Use a different port 2 == COM3 == /dev/ttyS2 for a second hand "hand2":
         cSDH hand2();
         hand2.OpenRS232( 2 );

         // Linux only: Use a different USB to RS232 device on port 3 /dev/ttyUSB3 for a third hand "hand3":
         cSDH hand3();
         hand2.OpenRS232( 3, 115200, -1, "/dev/ttyUSB%d" );

       \endcode

       <hr>
    */
    void OpenRS232(  int _port=0, unsigned long _baudrate = 115200, double _timeout=-1, char const* _device_format_string="/dev/ttyS%d" )
        throw (cSDHLibraryException*);


    /*!
      Open connection to %SDH via CAN using an ESD CAN card.
      If the library was compiled without ESD CAN support then this will just throw an
      exception. See setting for WITH_ESD_CAN in the top level makefile.

       \param _net     : The ESD CAN net number of the CAN port to use. (default: 0)
       \param _baudrate : the CAN baudrate in bit/s. Only some bitrates are valid: (1000000 (default),800000,500000,250000,125000,100000,50000,20000,10000)
       \param _timeout : The timeout to use:
                         - <= 0 : wait forever (default)
                         - T  : wait for T seconds
       \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID, default=43=0x02b)
       \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID, default=42=0x02a)

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // use default parameters for net, baudrate, timeout and IDs
         hand.OpenCAN_ESD( );

         // use non default settings:
         // net=1, baudrate=500000, timeout=1.0, id_read=0x143, id_write=0x142
         hand.OpenCAN_ESD( 1, 500000, 1.0, 0x143, 0x142 );
       \endcode

       <hr>
    */
    void OpenCAN_ESD(  int _net=0,  unsigned long _baudrate=1000000, double _timeout=0.0, Int32 _id_read=43, Int32 _id_write=42 )
        throw (cSDHLibraryException*);


    /*!
      Open connection to %SDH via CAN using an ESD CAN card using an existing handle.
      If the library was compiled without ESD CAN support then this will just throw an
      exception. See setting for WITH_ESD_CAN in the top level makefile.

       \param _ntcan_handle : the ESD CAN handle to reuse (please cast your NTCAN_HANDLE to tDeviceHandle! It is save to do that!)
       \param _timeout : The timeout to use:
                         - <= 0 : wait forever (default)
                         - T  : wait for T seconds
       \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID, default=43=0x2a)
       \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID, default=42=0x2a)

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...
         // and 'handle' is a valid NTCAN_HANDLE for the ESD device

         // use default parameters for timeout and IDs
         hand.OpenCAN_ESD( tDeviceHandle(handle) );

         // or use non default settings:
         // timeout=1.0, id_read=0x143, id_write=0x142
         hand.OpenCAN_ESD( handle, 1.0, 0x143, 0x142 );

       \endcode

       <hr>
    */
    void OpenCAN_ESD(  tDeviceHandle _ntcan_handle, double _timeout=0.0, Int32 _id_read=43, Int32 _id_write=42 )
        throw (cSDHLibraryException*);


    /*!
      Open connection to %SDH via CAN using an PEAK CAN card.
      If the library was compiled without PEAK CAN support then this will just throw an
      exception. See setting for WITH_PEAK_CAN in the top level makefile.

       \param _baudrate : the CAN baudrate in bit/s. Only some bitrates are valid: (1000000 (default),800000,500000,250000,125000,100000,50000,20000,10000)
       \param _timeout : The timeout to use:
                         - <= 0 : wait forever (default)
                         - T  : wait for T seconds
       \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID, default=43=0x02b)
       \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID, default=42=0x02a)
       \param _device   - the PEAK device name. Used for the Linux char dev driver only. default="/dev/pcanusb0"

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // use default parameters for baudrate, timeout, IDs and device
         hand.OpenCAN_PEAK( );

         // use non default settings:
         // baudrate=500000, timeout=1.0, id_read=0x143, id_write=0x142, , const char *device="/dev/pcanusb1"
         hand.OpenCAN_PEAK( 500000, 1.0, 0x143, 0x142, "/dev/pcanusb1" );
       \endcode

       <hr>
    */
    void OpenCAN_PEAK( unsigned long _baudrate=1000000, double _timeout=0.0, Int32 _id_read=43, Int32 _id_write=42, const char *_device="/dev/pcanusb0" )
        throw (cSDHLibraryException*);


    /*!
      Open connection to %SDH via CAN using an PEAK CAN card using an existing handle.
      If the library was compiled without PEAK CAN support then this will just throw an
      exception. See setting for WITH_PEAK_CAN in the top level makefile.

       \param _handle : The PEAK CAN handle to reuse to connect to the PEAK CAN driver (please cast your PEAK_HANDLE to tDeviceHandle! It is save to do that!)
       \param _timeout : The timeout to use:
                         - <= 0 : wait forever (default)
                         - T  : wait for T seconds
       \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID, default=43=0x2a)
       \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID, default=42=0x2a)

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...
         // and 'handle' is a valid HANDLE for the PEAK device

         // use default parameters for timeout and IDs
         hand.OpenCAN_PEAK( tDeviceHandle(handle) );

         // or use non default settings:
         // timeout=1.0, id_read=0x143, id_write=0x142
         hand.OpenCAN_PEAK( handle, 1.0, 0x143, 0x142 );

       \endcode

       <hr>
    */
    void OpenCAN_PEAK(  tDeviceHandle _handle, double _timeout=0.0, Int32 _id_read=43, Int32 _id_write=42 )
        throw (cSDHLibraryException*);

    /*!
      Open connection to %SDH via TCP using TCP/IP address \a _tcp_adr and \a _tcp_port.

       \param _tcp_adr : The tcp host address of the SDH. Either a numeric IP as string or a hostname
       \param _tcp_port : the tcp port number on the SDH to connect to
       \param _timeout : The timeout to use:
                         - <= 0 : wait forever (default)
                         - T  : wait for T seconds

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // IP-address 192.168.1.1, port 23, timeout=0.0
         hand.OpenTCP( "192.168.1.1.", 23, 0.0 );
       \endcode

       <hr>
    */
    void OpenTCP( char const* _tcp_adr="192.168.1.1", int _tcp_port=23, double _timeout=0.0 )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
       Close connection to %SDH.

       The default behaviour is to \b not leave the controllers of the
       %SDH enabled (to prevent overheating). To keep the controllers
       enabled (e.g. to keep the finger axes actively in position) set
       \a leave_enabled to \c true. Only already enabled axes will be
       left enabled.

       \param leave_enabled - Flag: true to leave the controllers on,
                              false (default) to disable the
                              controllers (switch powerless)

       This throws a (cSDHErrorCommunication*) exception if the
       connection was not opened before.

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // Close connection to SDH, power off controllers:
         hand.Close();

         // To leave the already enabled controllers enabled:
         hand.Close( true );

       \endcode

       <hr>
    */
    void Close( bool leave_enabled=false )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return true if connection to %SDH firmware/hardware is open.
    */
    virtual bool IsOpen( void )
        throw ();


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_communication
    //! @}
    //#####################################################################


    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_auxilliary
       \name   Auxiliary movement methods

       @{
    */

    //-----------------------------------------------------------------
    /*!
      Stop movement of all axes of the %SDH and switch off the controllers

       This command will always be executed sequentially: it will return
       only after the %SDH has confirmed the emergency stop.

       \bug
       For now this will \b NOT work while a GripHand() command is
       executing, even if that was initiated non-sequentially!

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // Perform an emergency stop:
         hand.EmergencyStop();

       \endcode

       <hr>
    */
    void EmergencyStop( void )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
      Stop movement of all axes but keep controllers on

       This command will always be executed sequentially: it will return
       only after the %SDH has confirmed the stop

       \bug
       For now this will \b NOT work while a GripHand() command is
       executing, even if that was initiated non-sequentially!

       \bug
       With %SDH firmware < 0.0.2.7 this made the axis jerk in eCT_POSE controller type.
       This is resolved in %SDH firmware 0.0.2.7 for the eCT_POSE controller type with
       velocity profile eVP_RAMP. For the eCT_POSE controller type with velocity profile
       eVP_SIN_SQUARE changing target points/ velocities while moving will still make
       the axes jerk.
       <br><b>=> Partly resolved in %SDH firmware 0.0.2.7</b>


       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // Perform a stop:
         hand.Stop();

       \endcode

       <hr>
    */
    void Stop( void )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    //-----------------------------------------------------------------
    // unimplemented from SAH:
    // def GetEmergencyStop( int* piBrakeState)

    //-----------------------------------------------------------------
    /*!
       Set the type of axis controller to be used in the %SDH

       With %SDH firmware >= 0.0.2.7 this will automatically set valid
       default values for all target velocities, accelerations and positions
       in the %SDH firmware, according to the \a controller type:
       - eCT_POSE:
         - target velocities will be set to default (40 deg/s)
         - target accelerations will be set to default (100 deg/(s*s))
         - target positions will be set to default (0.0 deg)
       - eCT_VELOCITY:
         - target velocities will be set to default (0 deg/s)
       - eCT_VELOCITY_ACCELERATION:
         - target velocities will be set to default (0 deg/s)
         - target accelerations will be set to default (100 deg/(s*s))

       This will also adjust the lower limits of the allowed velocities
       here in the SDHLibrary, since the eCT_POSE controller allows only
       positive velocities while the eCT_VELOCITY and
       eCT_VELOCITY_ACCELERATION controllers require also negative
       velocities.

       \attention The availability of a controller type depends on the
         %SDH firmware of the attached %SDH and is checked here.
         - firmware <= 0.0.2.5: only eCT_POSE
         - firmware >= 0.0.2.6: eCT_POSE, eCT_VELOCITY, eCT_VELOCITY_ACCELERATION

       \param controller - identifier of controller to set. Valid
                           values are defined in eControllerType

       \par Examples:
       \code
         // Assuming 'hand' is a cSDH object ...

         // Set the pose controller in the SDH
         // (see e.g. demo-simple.cpp, demo-simple2.cpp, demo-simple3.cpp for further examples)
         hand.SetController( hand.eCT_POSE );

         // Set the simple velocity controller in the SDH:
         hand.SetController( hand.eCT_VELOCITY );

         // Set the velocity with acceleration ramp controller in the SDH:
         // (see e.g. demo-velocity-acceleration.cpp for further examples)
         hand.SetController( hand.eCT_VELOCITY_ACCELERATION );

       \endcode

       <hr>
    */
    void SetController( cSDHBase::eControllerType controller )
        throw( cSDHLibraryException* );

    //-----------------------------------------------------------------
    /*!
      Get the type of axis controller used in the %SDH

      The currently set controller type will be queried and returned
      (One of eControllerType)

      \par Examples:
      \code
        // Assuming 'hand' is a sdh.cSDH object ...

        // Get the controller type of the attached SDH:
        ct = hand.GetController();

        // Print result, numerically and symbolically
        std::cout << "Currently the axis controller type is set to " << ct;
        std::cout << "(" << GetStringFromControllerType(ct) << ")\n";
      \endcode

      <hr>

    */
    eControllerType GetController( void  )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
     Set the type of velocity profile to be used in the %SDH

      \param velocity_profile - Name or number of velocity profile to set. Valid
                                values are defined in eVelocityProfileType


      \par Examples:
      \code
        // Assuming 'hand' is a cSDH object ...

        // Set the sin square velocity profile in the SDH:
        hand.SetVelocityProfile( hand.eVP_SIN_SQUARE );

        // Or else set the ramp velocity profile in the SDH:
        hand.SetVelocityProfile( hand.eVP_RAMP )
      \endcode

      <hr>
    */
    void SetVelocityProfile( eVelocityProfile velocity_profile )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
     Get the type of velocity profile used in the %SDH

      \return the currently set velocity profile as integer, see eVelocityProfileType


      \par Examples:
      \code
        // Assuming 'hand' is a cSDH object ...

        // Get the velocity profile from the SDH:
        velocity_profile = hand.GetVelocityProfile();
        // now velocity_profile is something like eVP_SIN_SQUARE or eVP_RAMP

      \endcode

      <hr>
    */
    eVelocityProfile GetVelocityProfile( void )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_auxilliary
    //! @}
    //#####################################################################


    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_axis
       \name   Methods to access %SDH on axis-level

       @{
    */

    //-----------------------------------------------------------------
    /*!
        Set the maximum allowed motor current(s) for axes.

        The maximum allowed motor currents are sent to the %SDH.
        The motor currents can be stored:
        - axis specific
        - mode specific (see #eMotorCurrentMode)

        \param axes           - A vector of axis indices to access.
        \param motor_currents - A vector of motor currents to set. If any of the numbers in the vector is \c NaN (Not a Number) then
                                the currently set axis motor current will be kept for the corresponding axis.
                                The value(s) are expected in the configured motor current unit system #uc_motor_current.
        \param mode           - the mode to set the maximum motor current for. One of the #eMotorCurrentMode modes.

        \remark
        - The lengths of the \a axes and \a motor_currents vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c motor_currents[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The motor currents are checked if they are in the allowed range
          [0 .. #f_max_motor_current_v], i.e. it is checked that \c motor_currents[i],
          converted to internal units, is in \c [0 .. \c f_max_motor_currents_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter* exception
          is thrown.

        See also #SetAxisMotorCurrent(int,double,eMotorCurrentMode)
        for an overloaded variant to set a single axis motor current
        or to set the same motor current for all axes.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set maximum allowed motor current of all axes to the given values in mode "eMCM_MOVE"::
          std::vector<double> all_motor_currents;
          all_motor_currents.push_back( 0.0 );
          all_motor_currents.push_back( 0.1 );
          all_motor_currents.push_back( 0.2 );
          all_motor_currents.push_back( 0.3 );
          all_motor_currents.push_back( 0.4 );
          all_motor_currents.push_back( 0.5 );
          all_motor_currents.push_back( 0.6 );

          hand.SetAxisMotorCurrent( hand.all_axes, all_motor_currents );


          // Set maximum allowed motor current of all axes to 0.1 A in mode "eMCM_HOLD":
          hand.SetAxisMotorCurrent( hand.All, 1.0, eMCM_HOLD );

          // Set maximum allowed motor current of axis 3 to 0.75 A in mode "eMCM_MOVE":
          hand.SetAxisMotorCurrent( 3, 0.75, eMCM_MOVE );

          // Set maximum allowed motor current of for axis 0, 4 and 2 to 0.0 A,
          // 0.4 A and 0.2 A respectively in mode "eMCM_GRIP"
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> motor_currents042;
          motor_currents042.push_back( 0.0 );
          motor_currents042.push_back( 0.4 );
          motor_currents042.push_back( 0.2 );

          hand.SetAxisMotorCurrent( axes042, states042, eMCM_GRIP );

        \endcode

        <hr>
    */
    void SetAxisMotorCurrent( std::vector<int> const& axes, std::vector<double> const& motor_currents, eMotorCurrentMode mode=eMCM_MOVE )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisMotorCurrent(std::vector<int>const&,std::vector<double>const&,eMotorCurrentMode),
        just for a single axis \a iAxis and a single motor current \a motor_current, see there.

        If \a iAxis is #All then \a motor_current is set for all axes.
    */
    void SetAxisMotorCurrent( int iAxis, double motor_current, eMotorCurrentMode mode=eMCM_MOVE )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the maximum allowed motor current(s) of axis(axes).

        The maximum allowed motor currents are read from the %SDH.
        The motor currents are stored:
        - axis specific
        - mode specific (see eMotorCurrentMode)

        \param axes           - A vector of axis indices to access.
        \param mode           - the mode to set the maximum motor current for. One of the #eMotorCurrentMode modes.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the motor currents of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_motor_current unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisMotorCurrent(int,eMotorCurrentMode) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get maximum allowed motor currents of all axes
          std::vector<double> v = hand.GetAxisMotorCurrent( hand.all_axes );
          // now v is something like {0.1, 0.2, 0.3, 0.4, 0.5, 0,6, 0.7}

          // Get maximum allowed motor current of axis 3 in mode "eMCM_MOVE"
          double mc3 = hand.GetAxisMotorCurrent( 3, eMCM_MOVE );
          // mc3 is now something like 0.75

          // Get maximum allowed motor current of axis 3 and 5 in mode "eMCM_GRIP"
          std::vector<int> axes35;
          axes35.push_back( 3 );
          axes35.push_back( 5 );

          v = hand.GetAxisMotorCurrent( axes35, eMCM_GRIP );
          // now v is something like {0.5,0.5};

        \endcode

        <hr>
    */
    std::vector<double> GetAxisMotorCurrent( std::vector<int> const& axes, eMotorCurrentMode mode=eMCM_MOVE )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisMotorCurrent(std::vector<int>const&,eMotorCurrentMode),
        just for a single axis, see there for details and examples.
    */
    double GetAxisMotorCurrent( int iAxis, eMotorCurrentMode mode=eMCM_MOVE )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Set enabled/disabled state of axis controller(s).

        The controllers of the selected axes are enabled/disabled in
        the %SDH. Disabled axes are not powered and thus might not
        remain in their current pose due to gravity, inertia or
        other external influences. But to prevent overheating the axis
        controllers should be switched of when not needed.

        \param axes   - A vector of axis indices to access.
        \param states - A vector of enabled states (0 = disabled, !=0 = enabled) to set.
                        If any of the numbers in the vector is \c NaN (Not a
                        Number) then the currently set enabled state will be
                        kept for the corresponding axis.

        \remark
        - The lengths of the \a axes and \a states vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c state[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - If \b any index is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        See also #SetAxisEnable(int,double), #SetAxisEnable(int,bool)
        for overloaded variants to set a single axis enabled/disabled
        or to set the same state for all axes.  See further
        #SetAxisEnable(std::vector<int>const&,std::vector<bool>const&) for a
        variant that accepts a \c bool vector for the states to set.


        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // Enable all axes:
          hand.SetAxisEnable( hand.all_axes, hand.ones_v );

          // Disable all axes:
          hand.SetAxisEnable( All, 0 );

          // Enable axis 0 and 2 while disabling axis 4:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );

          std::vector<double> states042;
          states042.push_back( 1.0 );
          states042.push_back( 0.0 );
          states042.push_back( 1.0 );

          hand.SetAxisEnable( axes042, states042 );


          // Disable axis 2
          hand.SetAxisEnable( 2, false );
        \endcode

        <hr>
    */
    void SetAxisEnable( std::vector<int> const& axes, std::vector<double> const& states )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisEnable(std::vector<int>const&,std::vector<double>const&),
        just for a single axis \a iAxis and a single axis state \a state, see there.

        If \a iAxis is #All then \a state is applied to all axes.
    */
    void SetAxisEnable( int iAxis=All, double state=1.0 )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisEnable(std::vector<int>const&,std::vector<double>const&),
        just accepting a vector of \c bool values as states, see there.
    */
    void SetAxisEnable( std::vector<int> const& axes, std::vector<bool> const& states )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisEnable(std::vector<int>const&,std::vector<double>const&),
        just for a single axis \a iAxis and a single axis state \a state, see there.

        If \a iAxis is #All then \a state is applied to all axes.
    */
    void SetAxisEnable( int iAxis=All, bool state=true )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get enabled/disabled state of axis controller(s).

        The enabled/disabled state of the controllers of the selected
        axes is read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of enabled/disabled states as doubles (0=disabled,
          1.0=enabled) of the selected axes.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisEnable(int) for an overloaded variant to
        access a single axis.


        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // Get enabled state of all axes:
          std::vector<double> v = hand.GetAxisEnable( hand.all_axes );
          // now v is something like {0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0}

          // Get enabled state of axis 3 and 5
          std::vector<int> axes35;
          axes35.push_back( 3 );
          axes35.push_back( 5 );

          v = hand.GetAxisEnable( axes35 );
          // now v is something like {1.0, 0.0}


          // Get enabled state of axis 3
          double v3 = hand.GetAxisEnable( 3 );
          // now v3 is something like 1.0

        \endcode

        <hr>
    */
    std::vector<double> GetAxisEnable( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisEnable(std::vector<int>const&), just for a single
        axis \a iAxis, see there for details and examples.
    */
    double GetAxisEnable( int iAxis )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
       Get the current actual state(s) of axis(axes).

       The actual axis states are read from the %SDH.

       \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the actual states of the selected axes.
        - The values are given as #eAxisState enum values
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisActualState(int) for an
        overloaded variant to access a single axis.

        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get actual axis state of all axes
          std::vector<eAxisState> v = hand.GetAxisActualState( hand.all_axes )
          // now v is something like {eAS_IDLE, eAS_POSITIONING, eAS_IDLE, eAS_IDLE, eAS_IDLE, eAS_DISABLED, eAS_IDLE}

          // Get actual axis state of axis 3
          eAxisState v3 = hand.GetAxisActualState( 3 );
          // v3 is now something like eAS_IDLE


          // Get actual state of axis 2 and 5
          std::vector<int> axes25;
          axes25.push_back( 2 );
          axes25.push_back( 5 );

          v = hand.GetAxisActualState( axes25 );
          // now v is something like {eAS_IDLE, eAS_DISABLED}
      \endcode

      <hr>
    */
    std::vector<eAxisState> GetAxisActualState( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisActualState(std::vector<int>const&),
        just for a single axis \a iAxis, see there for details and examples.
    */
    eAxisState GetAxisActualState( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Wait until the movement(s) of of axis(axes) has finished

        The state of the given axis(axes) is(are) queried until all
        axes are no longer moving.

        \param axes    - A vector of axis indices to access.
        \param timeout - a timeout in seconds or -1.0 (default) to wait indefinetly.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.
          - If \a timeout < 0 then this function will wait arbitrarily long
          - If a \a timeout is given then this function will throw a
            cSDHErrorCommunication exception if the given axes are still
            moving after \a timeout many seconds

        See also #WaitAxis(int,double) for an overloaded variant to
        wait for a single axis or all axes.

        \bug Due to a bug in %SDH firmwares prior to 0.0.2.6 the WaitAxis() command
             was somewhat unreliable there. When called immediately after a movement command like MoveHand(),
             then the WaitAxis() command returned immediately without waiting for the end of the movement.
             With %SDH firmwares 0.0.2.6 and newer this is no longer problematic and WaitAxis() works as expected.
             <br><b>=> Resolved in %SDH firmware 0.0.2.6</b>

        \bug With %SDH firmware 0.0.2.6 WaitAxis() did not work if one of the new
             velocity based controllers (eCT_VELOCITY, eCT_VELOCITY_ACCELERATION)
             was used. With %SDH firmwares 0.0.2.7 and newer this now works. Here
             the WaitAxis() waits until the selected axes come to velocity 0.0
             <br><b>=> Resolved in %SDH firmware 0.0.2.7</b>

        \par Examples:
        Example 1, WaitAxis and eCT_POSE controller, see also the demo program demo-simple3:
        \code
          // Assuming "hand" is a cSDH object ...

          hand.SetController( eCT_POSE );

          // Set a new target pose for axis 1,2 and 3
          std::vector<int> axes123;
          axes123.push_back( 1 );
          axes123.push_back( 2 );
          axes123.push_back( 3 );

          std::vector<double> angles123;
          angles123.push_back( -20.0 );
          angles123.push_back( -30.0 );
          angles123.push_back( -40.0 );


          hand.SetAxisTargetAngle( axes123, angles123 );

          // Move axes there non sequentially:
          hand.MoveAxis( axes123, false );

          // The last call returned immediately so we now have time to
          // do something else while the hand is moving:

          // ... insert any calculation here ...

          // Before doing something else with the hand make sure the
          // selected axes have finished the last movement:
          hand.WaitAxis( axes123 );


          // go back home (all angles to 0.0):
          hand.SetAxisTargetAngle( hand.All, 0.0 );

          // Move all axes there non sequentially:
          hand.MoveAxis( hand.All, False );

          // ... insert any other calculation here ...

          // Wait until all axes are there, with a timeout of 10s:
          hand.WaitAxis( hand.All, 10.0 );

          // now we are at the desired position.
       \endcode

       Example 2, WaitAxis and eCT_VELOCITY_ACCELERATION controller, see also the demo program demo-velocity-acceleration
       \code
          // Assuming "hand" is a cSDH object ...

          hand.SetController( eCT_VELOCITY_ACCELERATION);

          // Set a new target velocity for axis 1,2 and 3
          std::vector<int> axes123;
          axes123.push_back( 1 );
          axes123.push_back( 2 );
          axes123.push_back( 3 );

          std::vector<double> velocities123;
          velocities123.push_back( -20.0 );
          velocities123.push_back( -30.0 );
          velocities123.push_back( -40.0 );


          hand.SetAxisTargetVelocity( axes123, velocities123 ); // this will make the axes move!

          // The last call returned immediately so we now have time to
          // do something else while the hand is moving:

          // ... insert any calculation here ...

          // to break and stop the movement just set the target velocities to 0.0
          velocities123[0] = 0.0;
          velocities123[1] = 0.0;
          velocities123[2] = 0.0;

          hand.SetAxisTargetVelocity( axes123, velocities123 ); // this will make the axes break with the default (de)acceleration

          // The previous command returned immediately, so
          // before doing something else with the hand make sure the
          // selected axes have stopped:
          hand.WaitAxis( axes123 );

          // now the axes have stopped
       \endcode

       <hr>
    */
    void WaitAxis( std::vector<int> const& axes, double timeout = -1.0 )
        throw (cSDHLibraryException*);



    //----------------------------------------------------------------------
    /*!
        Like #WaitAxis(std::vector<int>const&,double),
        just for a single axis \a iAxis, see there for details and examples.

        If \a iAxis is #All then wait for all axes axes.
    */
    void WaitAxis( int iAxis, double timeout = -1.0 )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Set the target angle(s) for axis(axes).

        The target angles are stored in the %SDH, the movement
        is not executed until an additional move command is sent.

        \param axes   - A vector of axis indices to access.
        \param angles - A vector of axis target angles to set. If any
                        of the numbers in the vector is \c NaN (Not a Number)
                        then the currently set axis target angle will be kept
                        for the corresponding axis. The value(s) are expected
                        in the configured angle unit system
                        #uc_angle.

        \remark
        - Setting the target angle will \b not make the axis/axes move.
        - The lengths of the \a axes and \a angles vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c angles[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The angles are checked if they are in the allowed range
          [#f_min_angle_v .. #f_max_angle_v], i.e. it is checked that \c angles[i],
          converted to internal units, is in \c [#f_min_angle_v[axes[i]] .. \c f_max_angle_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        See also #SetAxisTargetAngle(int,double) for an overloaded variant to
        set a single axis target angle or to set the same target angle for all
        axes.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis angle of all axes to the given values:
          std::vector<double> all_angles;
          all_angles.push_back( 0.0 );
          all_angles.push_back( -11.0 );
          all_angles.push_back( -22.0 );
          all_angles.push_back( -33.0 );
          all_angles.push_back( -44.0 );
          all_angles.push_back( -55.0 );
          all_angles.push_back( -66.0 );

          hand.SetAxisTargetAngle( hand.all_axes, all_angles );


          // Set target axis angle of axis 3 to -42 degrees:
          hand.SetAxisTargetAngle( 3, -42.0 );

          // Set target angle of for axis 0, 4 and 2 to 0.0, -44.4 and -2.22 degrees respectively:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> angles042;
          angles042.push_back( 0.0 );
          angles042.push_back( -44.4 );
          angles042.push_back( -2.22 );

          hand.SetAxisTargetAngle( axes042, angles042 );


          // Set target axis angle of all axes to 0 degrees (home-position)
          hand.SetAxisTargetAngle( hand.All, 0.0 );

        \endcode

        <hr>
    */
    void SetAxisTargetAngle( std::vector<int> const& axes, std::vector<double> const& angles )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisTargetAngle(std::vector<int>const&,std::vector<double>const&),
        just for a single axis \a iAxis and a single angle \a angle, see there for details and examples.

        If \a iAxis is #All then \a motor_current is set for all axes.
    */
    void SetAxisTargetAngle( int iAxis, double angle )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Set the \b target angle(s) and get the \b actual angle(s) for axis(axes).

        Opposed to SetAxisTargetAngle() this will make the fingers move to the
        set target angles immediately, if the axis controllers are already enabled!

        \param axes       - A vector of axis indices to access.
        \param angles     - A vector of axis target angles to set. If any
                            of the numbers in the vector is \c NaN (Not a
                            Number) then the currently set axis target angle
                            will be kept for the corresponding axis. The
                            value(s) are expected in the configured angle
                            unit system #uc_angle.

        \return the actual angle(s) of the selected axes

        \remark
        - The lengths of the \a axes and \a angles vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c angles[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The angles are checked if they are in the allowed range
          [#f_min_angle_v .. #f_max_angle_v], i.e. it is checked that \c angles[i],
          converted to internal units, is in \c [#f_min_angle_v[axis[i]] .. \c f_max_angle_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis angles of all axes to the given values and read back the actual angle:
          std::vector<double> all_target_angles;
          all_target_angles.push_back( 0.0 );
          all_target_angles.push_back( 11.0 );
          all_target_angles.push_back( 22.0 );
          all_target_angles.push_back( 33.0 );
          all_target_angles.push_back( 44.0 );
          all_target_angles.push_back( 55.0 );
          all_target_angles.push_back( 66.0 );

          std::vector<double> all_acutal_angles;
          all_acutal_angles = hand.SetAxisTargetGetAxisActualAngle( hand.all_axes, all_target_angles );


          // Set target angle of for axis 0,4 and 2 to 0.0, 44.4 and 2.22 degrees per second respectively:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> target_angles042;
          target_angles042.push_back( 0.0 );
          target_angles042.push_back( 44.4 );
          target_angles042.push_back( 2.22 );

          std::vector<double> actual_angles042;
          actual_angles042 = hand.SetAxisTargetGetAxisActualAngle( axes042, target_angles042 );

        \endcode

        <hr>
    */
    std::vector<double> SetAxisTargetGetAxisActualAngle( std::vector<int> const& axes, std::vector<double> const& angles )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the target angle(s) of axis(axes).

        The currently set target angles are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the target angles of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angle unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisTargetAngle(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get target axis angle of all axes
          std::vector<double> v = hand.GetAxisTargetAngle( hand.all_axes );
          // now v is something like {0.0, 0.0, 42.0, 0.0, 47.11, 0,0, 0.0}

          // Get target axis angle of axis 2
          double v2 = hand.GetAxisTargetAngle( 2 );
          // v2 is now something like 42.0


          // Get target axis angle of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisTargetAngle( axes24 );
          // now v is something like {42.0, 47.11}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisTargetAngle( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisTargetAngle(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single angle, see there for details and
        examples.
    */
    double GetAxisTargetAngle( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
       Get the current actual angle(s) of axis(axes).

       The actual angles are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the actual angles of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angle unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisActualAngle(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get actual axis angle of all axes
          std::vector<double> v = hand.GetAxisActualAngle( hand.all_axes );
          // now v is something like {0.0, 0.0, 42.0, 0.0, 47.11, 0,0, 0.0}

          // Get actual axis angle of axis 2
          double v2 = hand.GetAxisActualAngle( 2 );
          // 2 is now something like 42.0


          // Get actual axis angle of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisActualAngle( axes24 );
          // now v is something like {42.0, 47.11}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisActualAngle( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisActualAngle(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single angle, see there for details and
        examples.
    */
    double GetAxisActualAngle( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Set the target velocity(s) for axis(axes).

        The target velocities are stored in the %SDH. The time at which
        a new target velocities will take effect depends on the current
        axis controller type:
        - in eCT_POSE controller type the new target velocities will
          not take effect until an additional move command is sent:
          MoveAxis(), MoveFinger(), MoveHand()
        - in eCT_VELOCITY and eCT_VELOCITY_ACCELERATION controller type
          the new target velocity will take effect immediately,
          if the axis controllers are already enabled.
          This means that in eCT_VELOCITY_ACCELERATION controller type
          the accelerations must be set with SetAxisTargetAcceleration()
          \b before calling SetAxisTargetVelocity().

        \param axes       - A vector of axis indices to access.
        \param velocities - A vector of axis target angles to set. If any
                            of the numbers in the vector is \c NaN (Not a
                            Number) then the currently set axis target velocity
                            will be kept for the corresponding axis. The
                            value(s) are expected in the configured angular
                            velocity unit system #uc_angular_velocity.



        \remark
        - The lengths of the \a axes and \a velocities vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c velocities[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The velocities are checked if they are in the allowed range:
          - in eCT_POSE controller type:
            [0 .. #f_max_velocity_v], i.e. it is checked that \c velocities[i],
            converted to internal units, is in \c [0 .. \c f_max_velocity_v[axes[i]]].
          - in eCT_VELOCITY and eCT_VELOCITY_ACCELERATION controller type:
            [-#f_max_velocity_v .. #f_max_velocity_v], i.e. it is checked that \c velocities[i],
            converted to internal units, is in \c [-f_max_velocity_v[axes[i]] .. \c f_max_velocity_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        See also #SetAxisTargetVelocity(int,double) for an overloaded variant
        to set a single axis target velocity or to set the same target velocity
        for all axes.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis velocity of all axes to the given values:
          std::vector<double> all_velocities;
          all_velocities.push_back( 0.0 );
          all_velocities.push_back( 11.0 );
          all_velocities.push_back( 22.0 );
          all_velocities.push_back( 33.0 );
          all_velocities.push_back( 44.0 );
          all_velocities.push_back( 55.0 );
          all_velocities.push_back( 66.0 );

          hand.SetAxisTargetVelocity( hand.all_axes, all_velocities );


          // Set target axis velocity of axis 3 to 42 degrees per second:
          hand.SetAxisTargetVelocity( 3, 42.0 );

          // Set target velocity of for axis 0,4 and 2 to 0.0, 44.4 and 2.22 degrees per second respectively:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> velocities042;
          velocities042.push_back( 0.0 );
          velocities042.push_back( 44.4 );
          velocities042.push_back( 2.22 );

          hand.SetAxisTargetVelocity( axes042, velocities042 );


          // Set target axis velocity of all axes to 47.11 degrees per second
          hand.SetAxisTargetVelocity( hand.All, 47.11 );

        \endcode

        <hr>
    */
    void SetAxisTargetVelocity( std::vector<int> const& axes, std::vector<double> const& velocities )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisTargetVelocity(std::vector<int>const&,std::vector<double>const&),
        just for a single axis \a iAxis and a single velocity \a velocity, see there for details and examples.
    */
    void SetAxisTargetVelocity( int iAxis, double velocity )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Set the \b target velocity(s) and get the \b actual velocitiy(s) for axis(axes).

        The target velocities are stored in the %SDH. The time at which
        a new target velocities will take effect depends on the current
        axis controller type:
        - in eCT_POSE controller type the new target velocities will
          not take effect until an additional move command is sent:
          MoveAxis(), MoveFinger(), MoveHand()
        - in eCT_VELOCITY and eCT_VELOCITY_ACCELERATION controller type
          the new target velocity will take effect immediately,
          if the axis controllers are already enabled.
          This means that in eCT_VELOCITY_ACCELERATION controller type
          the accelerations must be set with SetAxisTargetAcceleration()
          \b before calling SetAxisTargetVelocity().

        \param axes       - A vector of axis indices to access.
        \param velocities - A vector of axis target velocities to set. If any
                            of the numbers in the vector is \c NaN (Not a
                            Number) then the currently set axis target velocity
                            will be kept for the corresponding axis. The
                            value(s) are expected in the configured angular
                            velocity unit system #uc_angular_velocity.

        \return the actual velocity(s) of the selected axes

        \remark
        - The lengths of the \a axes and \a velocities vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c velocities[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The velocities are checked if they are in the allowed range:
          - in eCT_POSE controller type:
            [0 .. #f_max_velocity_v], i.e. it is checked that \c velocities[i],
            converted to internal units, is in \c [0 .. \c f_max_velocity_v[axes[i]]].
          - in eCT_VELOCITY and eCT_VELOCITY_ACCELERATION controller type:
            [-#f_max_velocity_v .. #f_max_velocity_v], i.e. it is checked that \c velocities[i],
            converted to internal units, is in \c [-f_max_velocity_v[axes[i]] .. \c f_max_velocity_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis velocity of all axes to the given values and read back the actual velocity:
          std::vector<double> all_target_velocities;
          all_target_velocities.push_back( 0.0 );
          all_target_velocities.push_back( 11.0 );
          all_target_velocities.push_back( 22.0 );
          all_target_velocities.push_back( 33.0 );
          all_target_velocities.push_back( 44.0 );
          all_target_velocities.push_back( 55.0 );
          all_target_velocities.push_back( 66.0 );

          std::vector<double> all_acutal_velocities;
          all_acutal_velocities = hand.SetAxisTargetGetAxisActualVelocity( hand.all_axes, all_target_velocities );


          // Set target velocity of for axis 0,4 and 2 to 0.0, 44.4 and 2.22 degrees per second respectively:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> target_velocities042;
          target_velocities042.push_back( 0.0 );
          target_velocities042.push_back( 44.4 );
          target_velocities042.push_back( 2.22 );

          std::vector<double> actual_velocities042;
          actual_velocities042 = hand.SetAxisTargetGetAxisActualVelocity( axes042, target_velocities042 );

        \endcode

        <hr>
    */
    std::vector<double> SetAxisTargetGetAxisActualVelocity( std::vector<int> const& axes, std::vector<double> const& velocities )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the target velocity(s) of axis(axes).

        The currently set target velocities are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the target velocities of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_velocity unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisTargetVelocity(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get target axis velocity of all axes
          std::vector<double> v = hand.GetAxisTargetVelocity( hand.all_axes );
          // now v is something like {0.0, 0.0, 42.0, 0.0, 47.11, 0,0, 0.0}

          // Get target axis velocity of axis 2
          double v2 = hand.GetAxisTargetVelocity( 2 );
          // v2 is now something like 42.0


          // Get target axis velocity of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisTargetVelocity( axes24 );
          // now v is something like {42.0, 47.11}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisTargetVelocity( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisTargetVelocity(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single velocity, see there for details
        and examples.
    */
    double GetAxisTargetVelocity( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the velocity limit(s) of axis(axes).

        The velocity limit(s) are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the velocity limits of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_velocity unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisLimitVelocity(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get axis velocity limits of all axes
          std::vector<double> v = hand.GetAxisLimitVelocity( hand.all_axes );
          // now v is something like {81.0, 140.0, 120.0, 140.0, 120.0, 140.0, 120.0}

          // Get axis velocity limit of axis 2
          double v2 = hand.GetAxisLimitVelocity( 2 );
          // v2 is now something like 120.0


          // Get axis velocity limits of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisLimitVelocity( axes24 );
          // now v is something like {120.0,120.0}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisLimitVelocity( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisLimitVelocity(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single velocity limit, see there for details
        and examples.
    */
    double GetAxisLimitVelocity( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the acceleration limit(s) of axis(axes).

        The acceleration limit(s) are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the acceleration limits of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_acceleration unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisLimitAcceleration(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get axis acceleration limits of all axes
          std::vector<double> v = hand.GetAxisLimitAcceleration( hand.all_axes );
          // now v is something like {81.0, 140.0, 120.0, 140.0, 120.0, 140.0, 120.0}

          // Get axis acceleration limit of axis 2
          double v2 = hand.GetAxisLimitAcceleration( 2 );
          // v2 is now something like 120.0


          // Get axis acceleration limits of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisLimitAcceleration( axes24 );
          // now v is something like {120.0,120.0}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisLimitAcceleration( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisLimitAcceleration(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single acceleration limit, see there for details
        and examples.
    */
    double GetAxisLimitAcceleration( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the actual velocity(s) of axis(axes).

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the actual velocities of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_velocity unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisActualVelocity(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get actual axis velocity of all axes
          std::vector<double> v = hand.GetAxisActualVelocity( hand.all_axes );
          // now v is something like {0.1, 0.2, 0.3, 13.2, 0.5, 0.0, 0.7}

          // Get actual axis velocity of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );
          v = hand.GetAxisActualVelocity( axes24 );
          // now v is something like {13.2, 0.0}

          // Get actual axis velocity of axis 2
          double v3 = hand.GetAxisActualVelocity( 2 );
          // v3 is now something like 13.2

        \endcode

        <hr>
    */
    std::vector<double> GetAxisActualVelocity( std::vector<int>const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisActualVelocity(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single velocity, see there for details
        and examples.
    */
    double GetAxisActualVelocity( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the current reference velocity(s) of axis(axes). (This velocity is used internally by the %SDH in eCT_VELOCITY_ACCELERATION mode).

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the reference velocities of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_velocity unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisReferenceVelocity(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Switch to "velocity control with acceleration ramp" controller mode first.
          // (When in another controller mode like the default eCT_POSE,
          //  then the reference velocities will not be valid):
          hand.SetController( eCT_VELOCITY_ACCELERATION );

          // Get reference axis velocity of all axes
          std::vector<double> v = hand.GetAxisReferenceVelocity( hand.all_axes );
          // now v is something like {0.1, 0.2, 0.3, 13.2, 0.5, 0.0, 0.7}

          // Get reference axis velocity of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );
          v = hand.GetAxisReferenceVelocity( axes24 );
          // now v is something like {13.2, 0.0}

          // Get reference axis velocity of axis 2
          double v3 = hand.GetAxisReferenceVelocity( 2 );
          // v3 is now something like 13.2

        \endcode

        \remark
          - the underlying rvel command of the %SDH firmware is not
            available in firmwares prior to 0.0.2.6. For such hands
            calling rvel will fail miserably.
          - The availability of an appropriate %SDH firmware is \b not checked
            here due to performance losses when this function is used often.

        <hr>
    */
    std::vector<double> GetAxisReferenceVelocity( std::vector<int>const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisReferenceVelocity(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single velocity, see there for details
        and examples.
    */
    double GetAxisReferenceVelocity( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Set the target acceleration(s) for axis(axes).

        The target accelerations are stored in the %SDH and are used only for:
        - the eCT_POSE controller type with eVP_RAMP velocity profile
        - the eCT_VELOCITY_ACCELERATION controller type

        Setting the target acceleration will not affect an ongoing movement,
        nor will it start a new movement. To take effect an additional command
        must be sent:
        - in eCT_POSE controller type a move command: MoveAxis() MoveFinger() MoveHand()
        - in eCT_VELOCITY_ACCELERATION controller type the velocity must be set: SetAxisTargetVelocity()

        \param axes          - A vector of axis indices to access.
        \param accelerations - A vector of axis target accelerations to set. If any
                               of the numbers in the vector is \c NaN
                               (Not a Number) then the currently set
                               axis target angle will be kept for the
                               corresponding axis. The value(s) are
                               expected in the configured angular
                               acceleration unit system
                               #uc_angular_acceleration.

        \remark
        - The lengths of the \a axes and \a accelerations vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c accelerations[i] will be applied
          to axis \c axes[i] (not axis \c i).
        - The indices are checked if they are valid axis indices.
        - The accelerations are checked if they are in the allowed range
          [0 .. #f_max_velocity_v], i.e. it is checked that \c accelerations[i],
          converted to internal units, is in \c [0 .. \c f_max_velocity_v[axes[i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        See also #SetAxisTargetAcceleration(int,double) for an
        overloaded variant to set a single axis target acceleration or
        to set the same target acceleration for all axes.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis acceleration of all axes to the given values:
          std::vector<double> all_accelerations;
          all_accelerations.push_back( 100.0 );
          all_accelerations.push_back( 101.0 );
          all_accelerations.push_back( 102.0 );
          all_accelerations.push_back( 103.0 );
          all_accelerations.push_back( 104.0 );
          all_accelerations.push_back( 105.0 );
          all_accelerations.push_back( 106.0 );

          hand.SetAxisTargetAcceleration( hand.all_axes, all_accelerations );


          // Set target axis acceleration of axis 3 to 420 degrees per square-second:
          hand.SetAxisTargetAcceleration( 3, 420.0 );

          // Set target acceleration of for axis 0,4 and 2 to 0.0, 444.0 and 222 degrees per square-second respectively:
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );
          std::vector<double> accelerations042;
          accelerations042.push_back( 100.0 );
          accelerations042.push_back( 104.0 );
          accelerations042.push_back( 102.0 );

          hand.SetAxisTargetAcceleration( axes042, accelerations042 );


          // Set target axis acceleration of all axes to 142.1 degrees per square-second
          hand.SetAxisTargetAcceleration( hand.All, 142.1 );

        \endcode

        <hr>
    */
    void SetAxisTargetAcceleration( std::vector<int>const& axes, std::vector<double>const& accelerations )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetAxisTargetAcceleration(std::vector<int>const&,std::vector<double>const&),
        just for a single axis \a iAxis and a single acceleration \a acceleration, see there for details and examples.
    */
    void SetAxisTargetAcceleration( int iAxis, double acceleration )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the target acceleration(s) of axis(axes).

        The currently set target accelerations are read from the %SDH.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the target accelerations of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_acceleration unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisTargetAcceleration(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get target axis acceleration of all axes
          std::vector<double> v = hand.GetAxisTargetAcceleration( hand.all_axes );
          // now v is something like {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}

          // Get target axis acceleration of axis 2
          double v2 = hand.GetAxisTargetAcceleration( 2 );
          // v2 is now something like 100.0


          // Get target axis acceleration of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisTargetAcceleration( axes24 );
          // now v is something like {100.0, 100.0}

        \endcode

        <hr>
    */
    std::vector<double> GetAxisTargetAcceleration( std::vector<int>const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisTargetAcceleration(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single acceleration, see there for details
        and examples.
    */
    double GetAxisTargetAcceleration( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the minimum angle(s) of axis(axes).

        The minimum angles are currently not read from the %SDH, but are stored
        in the library.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the min angles of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angle unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisMinAngle(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get minimum axis angles of all axes
          std::vector<double> v = hand.GetAxisMinAngle( hand.all_axes );
          // now v is something like {0.0, -90.0, -90.0, -90.0, -90.0, -90.0, -90.0}

          // Get minimum axis angle of axis 3
          double v3 = hand.GetAxisMinAngle( 3 );
          // v3 is now something like -90.0

          // Get minimum axis angle of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisMinAngle( axes24 );
          // now v is something like {-90.0, -90.0}


          // Or if you change the angle unit system:
          hand.UseRadians();

          v = hand.GetAxisMinAngle( hand.all_axes );
          // now v is something like {0.0, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966}
        \endcode

        <hr>
    */
    std::vector<double> GetAxisMinAngle( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisMinAngle(std::vector<int>const&), just for a single axis
        \a iAxis and returning a single minimum angle, see there for details
        and examples.
    */
    double GetAxisMinAngle( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the maximum angle(s) of axis(axes).

        The maximum angles are currently not read from the %SDH, but are stored
        in the library.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the max angles of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angle unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisMaxAngle(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get maximum axis angles of all axes
          std::vector<double> v = hand.GetAxisMaxAngle( hand.all_axes );
          // now v is something like {90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0}

          // Get maximum axis angle of axis 3
          double v3 = hand.GetAxisMaxAngle( 3 );
          // v3 is now something like 90.0

          // Get maximum axis angle of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisMaxAngle( axes24 );
          // now v is something like {90.0, 90.0}


          // Or if you change the angle unit system:
          hand.UseRadians();

          v = hand.GetAxisMaxAngle( hand.all_axes );
          // now v is something like { 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966}
        \endcode

        <hr>
    */
    std::vector<double> GetAxisMaxAngle( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisMaxAngle(std::vector<int>const&), just for a single axis
        \a iAxis and returning a single maximum angle, see there for details
        and examples.
    */
    double GetAxisMaxAngle( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
       Get the maximum velocity(s) of axis(axes). These are the
       (theoretical) maximum velocities as determined by the maximum motor
       velocity and gear box ratio. The values do not take things like
       friction or inertia into account. So it is likely that these
       maximum velocities cannot be reached by the real hardware in reality.

       The maximum velocities are currently read once from the %SDH when
       the communication to the %SDH is opened. Later queries of this
       maximum velocities will use the values stored in the library.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the max angular velocities of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_velocity unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisMaxVelocity(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get maximum axis angular velocities of all axes
          std::vector<double> v = hand.GetAxisMaxVelocity( hand.all_axes );
          // now v is something like {83.857,200.000,157.895,200.000,157.895,200.000,157.895}

          // Get maximum axis angular velocity of axis 3
          double v3 = hand.GetAxisMaxVelocity( 3 );
          // v3 is now something like 200.0

          // Get maximum axis angular velocity of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisMaxVelocity( axes24 );
          // now v is something like {157.895, 157.895}


          // Or if you change the angular velocity unit system:
          hand.UseRadians();

          v = hand.GetAxisMaxVelocity( hand.all_axes );
          // now v is something like {1.46358075084, 3.49065850399, 2.75578762244, 3.49065850399, 2.75578762244, 3.49065850399, 2.75578762244}
        \endcode

        <hr>
    */
    std::vector<double> GetAxisMaxVelocity( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisMaxVelocity(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single minimum angle, see there for
        details and examples.
    */
    double GetAxisMaxVelocity( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
       Get the maximum acceleration(s) of axis(axes).

       The maximum accelerations are currently not read from the %SDH, but are
       stored in the library.

        \param axes - A vector of axis indices to access.

        - The indices in \a axes are checked if they are valid axis indices.
        - If \b any axis index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of the max angular accelerations of the selected axes.
        - The values are converted to the selected external unit
          system using the configured #uc_angular_acceleration unit converter object.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisMaxAcceleration(int) for an
        overloaded variant to access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get maximum axis angular accelerations of all axes
          std::vector<double> v = hand.GetAxisMaxAcceleration( hand.all_axes );
          // now v is something like {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}

          // Get maximum axis angular acceleration of axis 3
          double v3 = hand.GetAxisMaxAcceleration( 3 );
          // v3 is now something like 1000.0

          // Get maximum axis angular acceleration of axis 2 and 4
          std::vector<int> axes24;
          axes24.push_back( 2 );
          axes24.push_back( 4 );

          v = hand.GetAxisMaxAcceleration( axes24 );
          // now v is something like {1000.0, 1000.0}


          // Or if you change the angular acceleration unit system:
          hand.UseRadians();

          v = hand.GetAxisMaxAcceleration( hand.all_axes );
          // now v is something like {17.453292519943293, 17.453292519943293, 17.453292519943293, 17.453292519943293, 17.453292519943293, 17.453292519943293, 17.453292519943293}
        \endcode

        <hr>
    */
    std::vector<double> GetAxisMaxAcceleration( std::vector<int> const& axes )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetAxisMaxAcceleration(std::vector<int>const&), just for a single
        axis \a iAxis and returning a single minimum angle, see there for
        details and examples.
    */
    double GetAxisMaxAcceleration( int iAxis )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Move selected axis/axes to the previously set target pose with
        the previously set velocity profile, (maximum) target
        velocities and target accelerations

        \param axes    - A vector of axis indices to access.
        \param sequ    - flag: if true (default) then the function executes sequentially
                         and returns not until after the %SDH has
                         finished the movement. If false then the
                         function returns immediately after the
                         movement command has been sent to the %SDH
                         (the currently set target axis angles for other axes will
                         then be \b overwritten with their current actual
                         axis angles).

        - The indices in \a axes are checked if they are valid axis indices.
        - If any index is invalid then no movement is performed, instead a
          #SDH::cSDHErrorInvalidParameter* exception is thrown.

        \return
        The expected/elapsed execution time for the movement in the configured time unit system #uc_time

        \remark
          - The axes will be enabled automatically.
          - Currently the actual movement velocity of an axis is
            determined by the %SDH firmware to make the movements of all
            involved axes start and end synchronously at the same time. Therefore
            the axis that needs the longest time for its movement at its
            given maximum velocity determines the velocities of all the
            other axes.
          - Other axes than those selected by \a axes will \b NOT move, even if
            target axis angles for the axes have been set.
            (Remember: as axis 0 is used by finger 0 and 2 these
            two fingers cannot be moved completely idependent of each other.)
          - If \a sequ is true then the currently set target axis angles for other
            fingers will be restored upon return of the function.
          - If \a sequ is false then the currently set target axis angles for other
            fingers will be \b overwritten with their current actual axis angles

        See also #MoveAxis(int,bool) for an overloaded variant to move a
        single axis.

        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // create an index vector for adressing axes 0, 4 and 2 (in that order)
          std::vector<int> axes042;
          axes042.push_back( 0 );
          axes042.push_back( 4 );
          axes042.push_back( 2 );

          // Set a new target pose for axes 0, 4 and 2:
          std::vector<double> angles042;
          angles042.push_back( 0.0 );
          angles042.push_back( -44.4 );
          angles042.push_back( -22.2 );

          hand.SetFingerTargetAngle( axes042, angles042 );


          // First move Axis 0 only to its new target position:
          hand.MoveAxis( 0 );

          // The axis 0 has now reached its target position 0.0 degrees.  The
          // target poses for axes 4 and 2 are still set since the
          // last MoveAxes() call was sequentially (und thus it could
          // restore the previously set target axis angles of not
          // selected axes after the movement finished)


          // So move axes 4 and 2 now, this time non-sequentially:
          std::vector<int> axes42;
          axes42.push_back( 4 );
          axes42.push_back( 2 );

          double t = hand.MoveAxes( axis42, false );

          // The two axes 4 and 2 are now moving to their target position.
          // We have to wait until the non-sequential call has finished:
          SleepSec( t );

          // The axes 4 and 2 have now moved to -44.4 and -22.2.

          // The target angles for other axes have by now been
          // overwritten since the last MoveAxis() call was
          // non-sequentially (und thus it could \b NOT restore the
          // previously set target axis angles of not selected axes
          // after the movement finished)


          // Set new target angles for all axes ("home pose");
          hand.SetAxisTargetAngle( hand.All, 0.0 );

          // Now move all axes back to home pose:
          hand.MoveAxes( hand.All );

        \endcode

        \bug
        With %SDH firmware < 0.0.2.7 calling MoveAxis() while some axes are moving
        in eCT_POSE controller type will make the joints jerk.
        This is resolved in %SDH firmware 0.0.2.7 for the eCT_POSE controller type with
        velocity profile eVP_RAMP. For the eCT_POSE controller type with velocity profile
        eVP_SIN_SQUARE changing target points/ velocities while moving will still make
        the axes jerk.
        <br><b>=> Partly resolved in %SDH firmware 0.0.2.7</b>

        <hr>
    */
    double MoveAxis( std::vector<int>const& axes, bool sequ=true )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #MoveAxis(std::vector<int>const&,bool), just for a single axis
        \a iAxis (or all axes if #All is given).
    */
    double MoveAxis( int iAxis, bool sequ=true )
        throw (cSDHLibraryException*);


    // unimplemented from SAH:
    // def GetJointAngle( int iFinger,double* pafAngle);
    // def GetJointSpeed( int iFinger,double* pafSpeed);
    // def GetJointTorque( int iFinger,double* pafTorque);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_axis
    //! @}
    //#####################################################################

    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_finger
       \name   Methods to access %SDH on finger-level

       @{
    */

    //-----------------------------------------------------------------
    /*!
        Set enabled/disabled state of axis controllers of finger(s).

        The controllers of the axes of the selected fingers are
        enabled/disabled in the %SDH. Disabled axes are not powered and thus
        might not remain in their current pose due to gravity, inertia or
        other external influences. But to prevent overheating the axis
        controllers should be switched of when not needed.

        \param fingers - A vector of finger indices to access.
        \param states  - A vector of enabled states (0 = disabled, !=0 = enabled) to set.
                         If any of the numbers in the vector is \c NaN (Not a
                         Number) then the currently set enabled state will be
                         kept for the corresponding axis.

        \remark
        - The lengths of the \a fingers and \a states vector must match.
        - The indices can be given in any order, but the order of
          their elements must match, i.e. \c state[i] will be applied
          to finger \c fingers[i] (not finger \c i).
        - The indices are checked if they are valid finger indices.
        - If \b any index is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.
        - As axis 0 is used for finger 0 and 2, axis 0 is disabled only
          if both finger 0 and 1 are disabled.

        See also #SetFingerEnable(int,double), #SetFingerEnable(int,bool)
        for overloaded variants to set a single finger enabled/disabled
        or to set the same state for all fingers.  See further
        #SetFingerEnable(std::vector<int>const&,std::vector<bool>const&) for a
        variant that accepts a \c bool vector for the states to set.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Enable finger 1 and 2 while disabling finger 0 :
          std::vector<double> states012;
          states012.push_back( 0.0 );
          states012.push_back( 1.0 );
          states012.push_back( 1.0 );

          hand.SetFingerEnable( hand.all_axes, states012 );
          // (this will keep axis 0 (used by the disabled finger 0) enabled,
          // since axis 0 is needed by the enabled finger 2 too);

          // Enable all fingers:
          hand.SetFingerEnable( hand.All,true );

          // Disable all fingers:
          hand.SetFingerEnable( hand.All, 0.0 );

          // Disable finger 2:
          hand.SetFingerEnable( 2, false );

        \endcode

        <hr>
    */
    void SetFingerEnable( std::vector<int> const& fingers, std::vector<double> const& states )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetFingerEnable(std::vector<int>const&,std::vector<double>const&),
        just for a single finger \a iAxis and a single angle \a angle, see
        there for details and examples.
    */
    void SetFingerEnable( int iFinger, double state=1.0 )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetFingerEnable(std::vector<int>const&,std::vector<double>const&),
        just with states as vector of \c bool values, see there for details and
        examples.
    */
    void SetFingerEnable( std::vector<int> const& fingers, std::vector<bool> const& states )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #SetFingerEnable(std::vector<int>const&,std::vector<double>const&),
        just for a single finger \a iAxis and a single angle \a angle, see
        there for details and examples.
    */
    void SetFingerEnable( int iFinger, bool state )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get enabled/disabled state of axis controllers of finger(s).

        The enabled/disabled state of the controllers of the selected
        fingers is read from the %SDH. A finger is reported disabled if
        any of its axes is disabled and reported enabled if all its
        axes are enabled.

        \param fingers - A vector of finger indices to access.

        - The indices in \a fingers are checked if they are valid finger indices.
        - If \b any finger index is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A vector of enabled/disabled states as doubles (0=disabled,
          1.0=enabled) of the selected axes.
        - The order of the elements of the \a axes vector and the returned values
          vector \a rv matches. I.e. \c rv[i] will be the value of axis
          \c axes[i] (not axis \c i).

        See also #GetAxisEnable(int) for an overloaded variant to
        access a single axis.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get enabled state of all fingers:
          std::vector<double> v = hand.GetFingerEnable( hand.all_fingers );
          // now v is something like {0.0, 1.0, 0.0}

          // Get enabled state of finger 0 and 2
          std::vector<int> fingers02;
          fingers02.push_back( 0 );
          fingers02.push_back( 2 );

          v = hand.GetFingerEnable( fingers02 );
          // now v is something like {0.0, 0.0}

          // Get enabled state of finger 1
          double v1 = hand.GetFingerEnable( 1 );
          // now v1 is something like 1.0

        \endcode

        <hr>
    */
    std::vector<double> GetFingerEnable( std::vector<int> const& fingers )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #GetFingerEnable(std::vector<int>const&),
        just for a single finger \a iFinger and returning a single
        double value
    */
    double GetFingerEnable( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Set the target angle(s) for a single finger.

        The target axis angles \a angle of finger \a iFinger are stored
        in the %SDH. The movement is not executed until an additional
        move command is sent.

        \param iFinger - index of finger to access.
                         This must be a single index.
        \param angles  - the angle(s) to set or \c None to set the current actual axis angles of the finger as target angle.
                         This can be a single number or a \ref sdhlibrary_cpp_sdh_h_csdh_vector "vector" of numbers.
                         The value(s) are expected in the configured angle unit system #uc_angle.

        \remark
        - Setting the target angles will \b not make the finger move.
        - The \a iFinger index is checked if it is a valid finger index.
        - The angles are checked if they are in the allowed range
          [0 .. #f_max_angle_v], i.e. it is checked that \c angles[i],
          converted to internal units, is in \c [0 .. \c f_max_angle_v[finger_axis_index[iFinger][i]]].
        - If \b any index or value is invalid then \b none of the specified
          values is sent to the %SDH, instead a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        See also #SetFingerTargetAngle(int,double,double,double) for an
        overloaded variant to set finger axis target angles from single \c
        double values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Set target axis angles of finger 0 to { 10.0, -08.15, 47.11 } degrees
          std::vector<double> angles;
          angles.push_back( 10.0 );
          angles.push_back( -08.15 );
          angles.push_back(  47.11 );

          hand.SetFingerTargetAngle( 0, angles );


          // Set target axis angles of finger 1 to { 0.0, 24.7, 17.4 } degrees
          angles[0] = 0.0;   // "virtual" base axis of finger 1
          angles[1] = 24.7;
          angles[2] = 17.4;
          hand.SetFingerTargetAngle( 1, { 0.0, 24.7, 17.4 } );


          // Set target axis angles of all axes of finger 0 to 12.34 degrees
          hand.SetFingerTargetAngle( 0, 12.34, 12.34, 12.34 );


          // REMARK: the last command changed the previously set target axis
          // angle for axis 0, since axis 0 is used as base axis for both
          // finger 0 and 2!
        \endcode

        <hr>
    */
    void SetFingerTargetAngle( int iFinger, std::vector<double> const& angles )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #SetFingerTargetAngle(int,std::vector<double>const&), just with
        individual finger axis angles \a a0, \a a1 and \a a2.
    */
    void SetFingerTargetAngle( int iFinger, double a0, double a1, double a2 )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the target axis angles of a single finger.

        The target axis angles of finger \a iFinger are read from the %SDH.

        \param iFinger - index of finger to access.
                         This must be a single index

        \remark
        - The \a iFinger index is checked if it is a valid finger index.
        - If \a iFinger is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A list of the selected fingers target axis angles
        - The values are returned in the configured angle unit system #uc_angle.

        See also #GetFingerTargetAngle(int,double&,double&,double&) for an
        overloaded variant to get finger axis target angles into single \c
        double values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get target axis angles of finger 0
          std::vector<double> v = hand.GetFingerTargetAngle( 0 );
          // now v is something like {42.0, -10.0, 47.11}

          // Get target axis angles of finger 1
          double a0, a1, a2;
          hand.GetFingerTargetAngle( 1, a0, a1, a2 );
          // now a0, a1, a2 are something like 0.0, 24.7 and -5.5 respectively.
        \endcode

        <hr>
    */
    std::vector<double> GetFingerTargetAngle( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #GetFingerTargetAngle(int), just returning the target axis angles
        in the \a a0, \a a1 and \a a2 parameters which are given by reference.
    */
    void GetFingerTargetAngle( int iFinger, double& a0, double& a1, double& a2 )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the current actual axis angles of a single finger.

        The current actual axis angles of finger \a iFinger are read from the %SDH.

        \param iFinger - index of finger to access.
                         This must be a single index.

        \remark
        - The \a iFinger index is checked if it is a valid finger index.
        - If \a iFinger is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
        - A list of the current actual axis angles of the selected finger
        - The values are returned in the configured angle unit system #uc_angle.

        See also #GetFingerActualAngle(int,double&,double&,double&) for an
        overloaded variant to get finger axis actual angles into single \c double
        values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get actual axis angles of finger 0
          std::vector<double> v = hand.GetFingerActualAngle( 0 );
          // v is now something like {42.0, -10.0, 47.11}

          // Get actual axis angles of finger 1
          double a0, a1, a2;
          hand.GetFingerTargetAngle( 1, a0, a1, a2 );
          // now a0, a1, a2 are something like 0.0, 24.7 and -5.5 respectively.
        \endcode

       <hr>
    */
    std::vector<double> GetFingerActualAngle( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #GetFingerActualAngle(int), just returning the actual axis angles
        in the \a a0, \a a1 and \a a2 parameters which are given by reference.
    */
    void GetFingerActualAngle( int iFinger, double& a0, double& a1, double& a2 )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the minimum axis angles of a single finger.

        The minimum axis angles of finger \a iFingers axes, stored in the
        library, are returned.

        \param iFinger - index of finger to access.
                         This must be a single index

        \remark
        - The \a iFinger index is checked if it is a valid finger index.
        - If \a iFinger is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
          - A list of the selected fingers minimum axis angles
          - The values are returned in the configured angle unit system #uc_angle.

        See also #GetFingerMinAngle(int,double&,double&,double&) for an
        overloaded variant to get finger axis min angles into single \c double
        values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get minimum axis angles of finger 0
          std::vector<double> v = hand.GetFingerMinAngle( 0 );
          // now v is something like {0.0, -90.0, -90.0}


          // Get target axis angles of finger 1
          double a0, a1, a2;
          hand.GetFingerMinAngle( 1, a0, a1, a2 );
          // now a0, a1, a2 are something like 0.0, -90.0, -90.0 respectively.


          // Or if you change the angle unit system:
          hand.UseRadians();
          v = hand.GetFingerMinAngle( 0 );
          // now v is something like {0.0, -1.5707963267948966, -1.5707963267948966}

        \endcode

        <hr>
    */
    std::vector<double> GetFingerMinAngle( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #GetFingerMinAngle(int), just returning
        the finger axis min angles in the \a a0, \a a1 and \a a2 parameters
        which are given by reference.
    */
    void GetFingerMinAngle( int iFinger, double& a0, double& a1, double& a2 )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get the maximum axis angles of a single finger.

        The maximum axis angles of finger \a iFingers axes, stored in the
        library, are returned.

        \param iFinger - index of finger to access.
                         This must be a single index

        \remark
        - The \a iFinger index is checked if it is a valid finger index.
        - If \a iFinger is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.

        \return
          - A list of the selected fingers maximum axis angles
          - The values are returned in the configured angle unit system #uc_angle.

        See also #GetFingerMaxAngle(int,double&,double&,double&) for an
        overloaded variant to get finger axis max angles into single \c double
        values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get maximum axis angles of finger 0
          std::vector<double> v = hand.GetFingerMaxAngle( 0 );
          // now v is something like {90.0, 90.0, 90.0}


          // Get target axis angles of finger 1
          double a0, a1, a2;
          hand.GetFingerMaxAngle( 1, a0, a1, a2 );
          // now a0, a1, a2 are something like 90.0, 90.0, 90.0 respectively.


          // Or if you change the angle unit system:
          hand.UseRadians();
          v = hand.GetFingerMaxAngle( 0 );
          // now v is something like {1.5707963267948966, 1.5707963267948966, 1.5707963267948966}

        \endcode

        <hr>
    */
    std::vector<double> GetFingerMaxAngle( int iFinger )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #GetFingerMaxAngle(int), just returning
        the finger axis max angles in the \a a0, \a a1 and \a a2 parameters
        which are given by reference.
    */
    void GetFingerMaxAngle( int iFinger, double& a0, double& a1, double& a2 )
        throw (cSDHLibraryException*);



    //-----------------------------------------------------------------
    /*!
        Get the cartesian xyz finger tip position of a single finger from the
        given axis angles (forward kinematics).

        \param iFinger - index of finger to access.
                         This must be a single index
        \param angles  - a vector of NUMBER_OF_AXES_PER_FINGER angles.
                         The values are expected in the configured angle unit
                         system #uc_angle.

        \remark
        - The \a iFinger index is checked if it is a valid finger index.
        - The angles are checked if they are in the allowed range
          [0 .. #f_max_angle_v], i.e. it is checked that \c angles[i],
          converted to internal units, is in \c [0 .. \c f_max_angle_v[finger_axis_index[iFinger][i]]].
        - If any index or value is invalid then a #SDH::cSDHErrorInvalidParameter*
          exception is thrown.


        \return
          - A vector of the x,y,z values of the finger tip position
          - The values are returned in the configured position unit system #uc_position.

        See also #GetFingerXYZ(int,double,double,double) for an overloaded
        variant to get finger tip position from single \c double values.


        \par Examples:
        \code
          // Assuming "hand" is a cSDH object ...

          // Get actual finger angles of finger 0:
          std::vector<double> angles = hand.GetFingerActualAngle( 0 );

          // Get actual finger tip position of finger 0:
          std::vector<double> position = hand.GetFingerXYZ( 0, angles );
          // now position is something like {18.821618775581801, 32.600000000000001, 174.0}
          // (assuming that finger 0 is at axis angles {0,0,0})

          // Get finger tip position of finger 2 at axis angles {90,-90,-90}:
          position = hand.GetFingerXYZ( 2, 90, -90, -90 );
          // now position is something like {18.821618775581804, 119.60000000000002, -53.0}

          // Or if you change the angle unit system:
          hand.UseRadians();
          position = hand.GetFingerXYZ( 0, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966 );
          // now position is still something like {18.821618775581804, 119.60000000000002, -53.0}

          // Or if you change the position unit system too:
          hand.uc_position = &cSDH::uc_position_meter
          position = hand.GetFingerXYZ( 0, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966 );
          // now position is still something like {0.018821618775581, 0.119.60000000000002, -0.052999999999}
        \endcode

       <hr>
    */
    std::vector<double> GetFingerXYZ( int iFinger, std::vector<double> const& angles )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Like #SetFingerTargetAngle(int,std::vector<double>const&), just with
        individual finger axis angles \a a0, \a a1 and \a a2.
    */
    std::vector<double> GetFingerXYZ( int iFinger, double a0, double a1, double a2 )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Move selected finger(s) to the previously set target pose with
        the previously set velocity profile, (maximum) target
        velocities and target accelerations.

        \param fingers - A vector of finger indices to access.
        \param sequ    - flag: if true (default) then the function executes sequentially
                         and returns not until after the %SDH has
                         finished the movement. If false then the
                         function returns immediately after the
                         movement command has been sent to the %SDH
                         (the currently set target axis angles for other fingers will
                         then be \b overwritten with their current actual
                         axis angles).

        - The indices in \a fingers are checked if they are valid finger indices.
        - If any index is invalid then no movement is performed, instead a
          #SDH::cSDHErrorInvalidParameter* exception is thrown.

        \return
        The expected/elapsed execution time for the movement in the configured time unit system #uc_time

        \remark
          - The axes will be enabled automatically.
          - Currently the actual movement velocity of an axis is
            determined by the %SDH firmware to make the movements of all
            involved axes start and end synchronously at the same time. Therefore
            the axis that needs the longest time for its movement at its
            given maximum velocity determines the velocities of all the
            other axes.
          - Other fingers than \a iFinger will \b NOT move, even if
            target axis angles for their axes have been set.
            (Exception: as axis 0 is used by finger 0 and 2 these
            two fingers cannot be moved completely idependent of each other.)
          - If \a sequ is true then the currently set target axis angles for other
            fingers will be restored upon return of the function.
          - If \a sequ is false then the currently set target axis angles for other
            fingers will be \b overwritten with their current actual axis angles

        See also #MoveFinger(int,bool) for an overloaded variant to move a
        single finger.

        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // Set a new target pose for finger 0:
          hand.SetFingerTargetAngle( 0, 0.0, 0.0, 0.0 );

          // Set a new target pose for finger 1
          hand.SetFingerTargetAngle( 1, 0.0, -10.0, -10.0 );

          // Set a new target pose for finger 2
          hand.SetFingerTargetAngle( 2, 20.0, -20.0, -20.0 );

          // Move finger 0 only (and finger 2 partly as axis 0 also belongs to finger 2);
          hand.MoveFinger( 0, true );
          // The finger 0 has been moved to {20,0,0}
          // (axis 0 is 'wrong' since the target angle for axis 0 has been overwritten
          //  while setting the target angles for finger 2);

          // The target poses for finger 1 and 2 are still set since the
          // last MoveFinger() call was sequentially.
          // So move finger 1 now:
          double t = hand.MoveFinger( 1, false );

          // wait until the non-sequential call has finished:
          SleepSec( t );

          // The finger 1 has been moved to {0,-10,-10}.

          // The target angles for finger 2 have been overwritten since the
          // last MoveFinger() call was non-sequentially.

          // Therefore this next call will just keep the fingers in their
          // current positions:
          hand.MoveFinger( hand.All, true );


          // Set new target angles for all axes ("home pose");
          hand.SetAxisTargetAngle( hand.All, 0.0 );

          // Now move all axes back to home pose:
          hand.MoveHand();

        \endcode

        \bug
        With %SDH firmware < 0.0.2.7 calling MoveFinger() while some axes are moving
        in eCT_POSE controller type will make the joints jerk.
        This is resolved in %SDH firmware 0.0.2.7 for the eCT_POSE controller type with
        velocity profile eVP_RAMP. For the eCT_POSE controller type with velocity profile
        eVP_SIN_SQUARE changing target points/ velocities while moving will still make
        the axes jerk.
        <br><b>=> Partly resolved in %SDH firmware 0.0.2.7</b>
        <hr>
    */
    double MoveFinger( std::vector<int>const& fingers, bool sequ=true )
        throw (cSDHLibraryException*);


    //----------------------------------------------------------------------
    /*!
        Like #MoveFinger(std::vector<int>const&,bool), just for a single finger
        \a iFinger (or all fingers if #All is given).
    */
    double MoveFinger( int iFinger, bool sequ=true )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
       Move all fingers to the previously set target pose with
       the previously set (maximum) velocities.

       This is just a shortcut to #MoveFinger(int,bool) with \a
       iFinger set to \c hand.All and \a sequ as indicated, so see there for
       details and examples.

       \bug
       With %SDH firmware < 0.0.2.7 calling MoveHand() while some axes are moving
       in eCT_POSE controller type will make the joints jerk.
       This is resolved in %SDH firmware 0.0.2.7 for the eCT_POSE controller type with
       velocity profile eVP_RAMP. For the eCT_POSE controller type with velocity profile
       eVP_SIN_SQUARE changing target points/ velocities while moving will still make
       the axes jerk.
       <br><b>=> Parltly resolved in %SDH firmware 0.0.2.7</b>
    */
    double MoveHand( bool sequ=true )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_finger
    //! @}
    //#####################################################################


    //######################################################################
    /*!
      \anchor sdhlibrary_cpp_sdh_h_csdh_grip
       \name   Methods to access %%SDH grip skills

       @{
    */

    //-----------------------------------------------------------------
    /*!
      Get the maximum velocity of grip skills

       The maximum velocity is currently not read from the %SDH, but is stored
       in the library.

       \return
         - a single double value is returned representing the velocity in the #uc_angular_velocity unit system


       \par Examples:
       \code
         // Assuming "hand" is a cSDH object ...

         // Get maximum grip skill velocity
         double v = hand.GetGripMaxVelocity();
         // v is now something like 100.0

         // Or if you change the velocity unit system:
         hand.UseRadians();
         v = hand.GetGripMaxVelocity();
         // now v is something like 1.7453292519943295
       \endcode

       <hr>
    */
    double GetGripMaxVelocity( void );


    //-----------------------------------------------------------------
    /*!
        Perform one of the internal #eGraspId "grips" or "grasps"

        \warning THIS DOES NOT WORK WITH %SDH FIRMWARE PRIOR TO 0.0.2.6 AND SDHLIBRARY-CPP PRIOR to 0.0.1.12
          This was a feature in the ancient times of the SDH1 and now does work
          again for %SDH firmware 0.0.2.6 and newer and SDHLIBRARY-CPP 0.0.1.12 and newer. We intend to further improve this
          feature (e.g. store user defined grips within the %SDH) in the future,
          but a particular deadline a has not been determined yet.

        \bug
          With %SDH firmware > 0.0.2.6 and SDHLibrary < 0.0.1.12 GripHand() does not work (<a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=575">Bug 575</a>)
          <br><b>=> Resolved in SDHLibrary 0.0.1.12</b>

        \bug
          With %SDH firmware < 0.0.2.6 GripHand() does not work and might
          yield undefined behaviour there
          <br><b>=> Resolved in %SDH firmware 0.0.2.6</b>

        \bug Currently the performing of a skill or grip with GripHand() can \b NOT be
             interrupted!!!  Even if the command is sent with \a sequ=false it \b cannot
             be stoped or emergency stopped.

        \param grip     - The index of the grip to perform [0..eGID_DIMENSION-1] (s.a. eGraspId)
        \param close    - close-ratio: [0.0 .. 1.0] where 0.0 is 'fully opened' and 1.0 is 'fully closed'
        \param velocity - maximum allowed angular axis velocity in the chosen external unit system #uc_angular_velocity
        \param sequ     - flag: if true (default) then the function executes sequentially
                          and returns not until after the %SDH has
                          finished the movement. If false then the
                          function returns immediately after the
                          movement command has been sent to the %SDH.

        - The \a close and \a velocity values are checked if they are in their
          allowed range.
        - If \b any value is invalid then \b no grip is perfomed, instead a
          #SDH::cSDHErrorInvalidParameter* exception is thrown.

        \return
          The expected/elapsed execution time for the movement in the configured time
          unit system #uc_time.

        \remark
          - Currently the actual movement velocity of an axis is
            determined by the %SDH firmware to make the movements of all
            involved axes start and end synchronously at the same time. Therefore
            the axis that needs the longest time for its movement at its
            given maximum velocity determines the velocities of all the
            other axes.
          - The currently set target axis angles are not changed by this command
          - The movement uses the eMotorCurrentMode motor current modes "eMCM_GRIP"
            while gripping and then changes the motor current mode to
            "eMCM_HOLD". After the movement previously set motor currents
            set for mode "eMCM_MOVE" are \b overwritten!

        \par Examples:
        \code
          // Assuming 'hand' is a cSDH object ...

          // Perform a fully opened centrical grip at 50 degrees per second:
          hand.GripHand( hand.eGID_CENTRICAL, 0.0, 50.0, true );

          // Now close it 50% with 30 degrees per second:
          hand.GripHand( hand.eGID_CENTRICAL, 0.5, 30.0, true );

          // Then close it completely with 20 degrees per second:
          hand.GripHand( hand.eGID_CENTRICAL, 1.0, 20.0, true );
        \endcode

        <hr>
    */
    double GripHand( eGraspId grip, double close, double velocity, bool sequ=true )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_sdh_h_csdh_grip
    //!  @}
    //#####################################################################

private:
    /*!
     * Update settings like min/max velocities and accelerations from the connected %SDH
     */
    void UpdateSettingsFromSDH();

    /*!
     * Adjust the limits for the velocity and acceleration according to the controller type.
     *
     * - in pose controller the velocities and accelerations are always positive and thus the minimum is 0.0
     * - in velocity based controllers the velocities and accelerations can be positive or negative and thus the minimum is -maximum
     */
    void AdjustLimits( cSDHBase::eControllerType controller );


    //! string containing the %SDH firmware release of the attaced %SDH (something like "0.0.2.7")
    std::string release_firmware;

    //! cached value of the axis controller type
    eControllerType controller_type;

    // unimplemented from SAH:
    // def GetFingerTipFT( int iFinger,double* pafFTData);

    // def GetFingerEnableState( int iFinger, int* piEnableState);

    // def GetCommandedValues( int iFinger, double* pafAngle,double* pafVelocity);
    // def GetHandConfig( int* piConfig);
    // def GetFingerLimitsStatus( int iPort,int iFinger,int* paiLimitStatus);
    // def ClearTorqueSensorOffset( int iPort,int iFinger);

    // def SetStiffnessFactor( int iFinger, double* pafStiffnessFactor);



}; // end of class cSDH
//#####################################################################

NAMESPACE_SDH_END

#endif


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored)
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================]
