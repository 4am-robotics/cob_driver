//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdh_cpp_general General file information
    \author   Dirk Osswald
    \date     2007-02-20


  \brief
    This file contains the interface to class #SDH::cSDH, the end user
    class to access the %SDH from a PC.


  \section sdhlibrary_cpp_sdh_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdh_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-08 13:36:01 +0100 (Di, 08 Mrz 2011) $
      \par SVN file revision:
        $Id: sdh.cpp 6521 2011-03-08 12:36:01Z Osswald2 $

  \subsection sdhlibrary_cpp_sdh_cpp_changelog Changelog of this file:
      \include sdh.cpp.log
*/
//======================================================================

#define _USE_MATH_DEFINES
#include <math.h>
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#include <vector>
#include <math.h>
#if SDH_USE_VCC
# include <float.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdh.h"
#include "util.h"
#include "release.h"
#include "simpletime.h"
#if SDH_USE_VCC
# include "rs232-vcc.h"
#else
# include "rs232-cygwin.h"
#endif
#if WITH_ESD_CAN
# include "canserial-esd.h"
#endif

#if WITH_PEAK_CAN
# include "canserial-peak.h"
#endif
#include "tcpserial.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions and classes (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function definitions
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class member definitions
//----------------------------------------------------------------------



cUnitConverter const cSDH::uc_angle_degrees( "angle", "degrees", "deg", 1.0, 0.0, 1 );

cUnitConverter const cSDH::uc_angle_radians( "angle", "radians", "rad", (2.0*M_PI)/360.0, 0.0, 3 );

cUnitConverter const cSDH::uc_time_seconds( "time", "seconds", "s", 1.0, 0.0, 3 );

cUnitConverter const cSDH::uc_time_milliseconds( "time", "milliseconds", "ms", 1000.0, 0.0, 0 );

cUnitConverter const cSDH::uc_temperature_celsius( "temparature", "degrees celsius", "deg C", 1.0, 0.0, 1 );

cUnitConverter const cSDH::uc_temperature_fahrenheit( "temparature", "degrees fahrenheit", "deg F", 1.8, 32.0, 1 );

cUnitConverter const cSDH::uc_angular_velocity_degrees_per_second( "angular velocity", "degrees/second", "deg/s", 1.0, 0.0, 1 );

cUnitConverter const cSDH::uc_angular_velocity_radians_per_second( "angular velocity", "radians/second", "rad/s", (2.0*M_PI)/360.0, 0.0, 3 );

cUnitConverter const cSDH::uc_angular_acceleration_degrees_per_second_squared( "angular acceleration", "degrees/(second*second)", "deg/(s*s)", 1.0, 0.0, 1 );

cUnitConverter const cSDH::uc_angular_acceleration_radians_per_second_squared( "angular acceleration", "radians/(second*second)", "rad/(s*s)", (2.0*M_PI)/360.0, 0.0, 3 );

cUnitConverter const cSDH::uc_motor_current_ampere( "motor current", "Ampere", "A", 1.0, 0.0, 3 );

cUnitConverter const cSDH::uc_motor_current_milliampere( "motor current", "milli Ampere", "mA", 1000.0, 0.0, 0 );

cUnitConverter const cSDH::uc_position_millimeter( "position", "millimeter", "mm", 1.0, 0.0, 1 );

cUnitConverter const cSDH::uc_position_meter( "position", "meter", "m", 1.0/1000.0, 0.0, 4 );


bool cSDH::IsVirtualAxis( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );
    return (iAxis >= NUMBER_OF_AXES);
}
//----------------------------------------------------------------------


std::vector<double> cSDH::SetAxisValueVector( std::vector<int> const& axes,
                                              std::vector<double> const& values,
                                              pSetFunction ll_set,
                                              pGetFunction ll_get,
                                              cUnitConverter const* uc,
                                              std::vector<double> const& min_values,
                                              std::vector<double> const& max_values,
                                              char const* name )
throw (cSDHLibraryException*)
{
    //---------------------
    // Check parameters:

    if (axes.size() != values.size())
    {
        throw new cSDHErrorInvalidParameter( cMsg( "Lengths of axis indices and %s values vectors do not match (%d != %d)", name, axes.size(), values.size()) );
    }

    int used = 0; // bit vector of used indices
    std::vector<int>::const_iterator ai = axes.begin();
    std::vector<double>::const_iterator vi = values.begin();
    for ( axes.begin(), vi = values.begin();
          ai != axes.end();
          ai++, vi++ )
    {
        CheckIndex( *ai, nb_all_axes, name );
        CheckRange( uc->ToInternal(*vi), min_values[ *ai ], max_values[ *ai ], name );

        used |= (1 << (*ai));
    }
    //
    //---------------------
    cdbg << "SetAxisValueVector: axes and values ok, used=" << used << "\n";

    //---------------------
    // Prepare data to send:
    // - ignore virtual axes
    // - get and use values from SDH if given value is NaN
    // - convert given

    cSimpleVector c_value;     // space to store value to set from SDH for those axes where the given value is NaN
    bool c_value_valid = false;

    cSimpleVector  all_values; // space to store ordered full vector if all axes are to be set
    cSimpleVector  returned_values; // space to store ordered full vector with responses


    // are all axes used?
    if ( (used & all_axes_used) != all_axes_used )
    {
        // No, so we have to get current target values first
        c_value = (comm_interface.*ll_get)( All, NULL );
        c_value_valid = true;
    }

    for ( ai = axes.begin(), vi = values.begin();
          ai != axes.end();
          ai++, vi++ )
    {
        double v;

        // Is it a virtual axis?
        if ( IsVirtualAxis( *ai ) )
            // yes, ignore virutal axes (low level functions know nothing about virtual axes)
            continue ;

        // It is a normal axis.

        // Is a real value given?
        if (SDH_ISNAN((*vi)))
        {
            // No, so use the value from the SDH.

            // Has that vector already been read from SDH?
            if (!c_value_valid)
            {
                // No, so read it from SDH (this is done only once in this loop).
                c_value = (comm_interface.*ll_get)( All, NULL );
                c_value_valid = true;
            }
            v = c_value[ *ai ];
        }
        else
            // only user given values must be converted to internal unit system
            v = uc->ToInternal( *vi );

        // Now v is the axis value to set for axis *ai in internal units

        cdbg << "SetAxisValueVector: setting v=" << v << "\n";

            all_values[ *ai ] = v;
    }

    returned_values = (comm_interface.*ll_set)( All, &(all_values[0]) );

    // convert returned_values to a std::vector
    std::vector<double> rv( axes.size(), 0.0 ); // the vector to return

    int i;
    for ( i=0, ai = axes.begin();
          ai != axes.end();
          i++, ai++ )
    {
        double v;

        if ( IsVirtualAxis( *ai ) )
        v = 0.0;
        else
        v = returned_values[ *ai ];

        rv[i] = uc->ToExternal( v );
    }

    return rv;
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisValueVector( std::vector<int> const& axes,
                                              pGetFunction ll_get,
                                              cUnitConverter const* uc,
                                              char const* name )
    throw (cSDHLibraryException*)
{

    //---------------------
    // Check parameters:

    std::vector<int>::const_iterator ai;

    for ( ai = axes.begin();
          ai != axes.end();
          ai++ )
    {
        CheckIndex( *ai, nb_all_axes, name );
    }
    //
    //---------------------


    // We want to read (most likely) more than one value, so
    // read all values at once in order to communicate as few as possible.
    cSimpleVector all_values = (comm_interface.*ll_get)( All, NULL );

    std::vector<double> rv( axes.size(), 0.0 ); // the vector to return

    int i;
    for ( i=0, ai = axes.begin();
          ai != axes.end();
          i++, ai++ )
    {
        double v;

        if ( IsVirtualAxis( *ai ) )
        v = 0.0;
        else
        v = all_values[ *ai ];

        rv[i] = uc->ToExternal( v );
    }

    return rv;
}
//----------------------------------------------------------------------


cSDH::cSDH( bool _use_radians, bool _use_fahrenheit, int _debug_level )
    :
    // call base class constructors:
    cSDHBase( _debug_level ),

    // init member objects:
    com( NULL ),
    comm_interface( _debug_level-1 ),
    controller_type( eCT_INVALID )
{
    cdbg.SetColor( "blue" );
    cdbg.PDM( "Debug messages of cSDH are printed like this.\n" );

    //---------------------
    // Initialize unit converters:
    if ( _use_radians )
        UseRadians();
    else
        UseDegrees();

    //! unit convert for times: default = uc_time_seconds
    uc_time  = &uc_time_seconds;

    if ( _use_fahrenheit )
    {
        uc_temperature = &uc_temperature_fahrenheit;
    }
    else
    {
        //! unit convert for temperatures: default = #uc_temperature_celsius
        uc_temperature = &uc_temperature_celsius;
    }

    //! unit converter for motor curent: default = #uc_motor_current_ampere
    uc_motor_current = &uc_motor_current_ampere;

    //! unit converter for position: default = #uc_position_millimeter
    uc_position = &uc_position_millimeter;

    //---------------------

    //---------------------
    // Initialize misc member variables

    //! The number of axis per finger (for finger 1 this includes the "virtual" base axis)
    NUMBER_OF_AXES_PER_FINGER = 3;

    //! The number of virtual axes
    NUMBER_OF_VIRTUAL_AXES = 1;

    //! Mapping of finger index to number of real axes of fingers:
    assert( NUMBER_OF_FINGERS == 3 );
    finger_number_of_axes.assign( NUMBER_OF_FINGERS, 3 );
    finger_number_of_axes[ 1 ] = 2;


    //! Mapping of finger index, finger axis index to axis index:
    std::vector<int>* f0_axis_index = new std::vector<int>( NUMBER_OF_AXES_PER_FINGER, 0 );
    std::vector<int>* f1_axis_index = new std::vector<int>( NUMBER_OF_AXES_PER_FINGER, 0 );
    std::vector<int>* f2_axis_index = new std::vector<int>( NUMBER_OF_AXES_PER_FINGER, 0 );
    (*f0_axis_index)[0] = 0;
    (*f0_axis_index)[1] = 1;
    (*f0_axis_index)[2] = 2;

    (*f1_axis_index)[0] = 7;
    (*f1_axis_index)[1] = 3;
    (*f1_axis_index)[2] = 4;

    (*f2_axis_index)[0] = 0;
    (*f2_axis_index)[1] = 5;
    (*f2_axis_index)[2] = 6;

    finger_axis_index.assign( NUMBER_OF_FINGERS, *f0_axis_index );
    finger_axis_index[1] = *f1_axis_index;
    finger_axis_index[2] = *f2_axis_index;

#if 0
    //! Vector of 3 epsilon values
    f_eps_v   = Numeric.array( [ eps   for i in range( 0, NUMBER_OF_AXES_PER_FINGER ) ] );

    //! Vector of 3 0 values
    f_zeros_v = Numeric.array( [ 0.0        for i in range( 0, NUMBER_OF_AXES_PER_FINGER ) ] );

    //! Vector of 3 1 values
    f_ones_v  = Numeric.array( [ 1.0        for i in range( 0, NUMBER_OF_AXES_PER_FINGER ) ] );
#endif

    nb_all_axes = NUMBER_OF_AXES + NUMBER_OF_VIRTUAL_AXES;

    zeros_v.assign(  nb_all_axes, 0.0 );
    ones_v.assign(   nb_all_axes, 1.0 );

    int i;
    all_axes.assign( nb_all_axes, 0 );
    for ( i = 0; i < nb_all_axes; i++ )
        all_axes[i] = i;

    all_real_axes.assign( nb_all_axes-NUMBER_OF_VIRTUAL_AXES, 0 );
    for ( i = 0; i < nb_all_axes-NUMBER_OF_VIRTUAL_AXES; i++ )
        all_real_axes[i] = i;

    all_fingers.assign( NUMBER_OF_FINGERS, 0 );
    for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
        all_fingers[i] = i;

    all_temperature_sensors.assign( NUMBER_OF_TEMPERATURE_SENSORS, 0 );
    for ( i = 0; i < NUMBER_OF_TEMPERATURE_SENSORS; i++ )
        all_temperature_sensors[i] = i;

    f_min_motor_current_v.assign( nb_all_axes, 0.0 );

    f_max_motor_current_v.assign( nb_all_axes, 1.0 );


    f_min_angle_v.assign( nb_all_axes, -90.0 );
    f_min_angle_v[ 0 ] = 0.0;                        // correct axis 0

    f_max_angle_v.assign( nb_all_axes, 90.0 );


    f_min_velocity_v.assign( nb_all_axes, 0.0 );

    // use some default values for the max velocities. read the correct values later from firmware when we are connected
    f_max_velocity_v.assign( nb_all_axes, 100.0 );
    f_max_velocity_v[ 0 ] = 80.0;                    // correct axis 0

    // use some default values for the max accelerations. read the correct values later from firmware when we are connected
    f_min_acceleration_v.assign( nb_all_axes, 0.0 );

    f_max_acceleration_v.push_back( 5000.0 );
    f_max_acceleration_v.push_back( 400.0 );
    f_max_acceleration_v.push_back( 1500.0 );
    f_max_acceleration_v.push_back( 400.0 );
    f_max_acceleration_v.push_back( 1500.0 );
    f_max_acceleration_v.push_back( 400.0 );
    f_max_acceleration_v.push_back( 1500.0 );
    f_max_acceleration_v.push_back( 5000.0 );          // virtual axis (setting to 0.0 would make SetAxisTargetAcceleration( All, x ) invalid for all x != 0.0

    //! Maximum allowed grip velocity (in internal units (degrees/second))
    grip_max_velocity = 100.0;

    //---------------------
    // set kinematic variables

    l1 = 86.5;

    l2 = 68.5;

    d = 66;

    h = 17.0;

    std::vector<double>* f0_offset = new std::vector<double>( 3, 0.0 );
    std::vector<double>* f1_offset = new std::vector<double>( 3, 0.0 );
    std::vector<double>* f2_offset = new std::vector<double>( 3, 0.0 );

    (*f0_offset)[ 0 ] = d/2.0;                          // x
    (*f0_offset)[ 1 ] = d/2.0*tan( DegToRad(30.0) );    // y
    (*f0_offset)[ 2 ] = h;                              // z

    (*f1_offset)[ 0 ] =  0.0;                           // x
    (*f1_offset)[ 1 ] =  -d/(2.0*cos( DegToRad(30.0) ));// y
    (*f1_offset)[ 2 ] =  h;                             // z

    (*f2_offset)[ 0 ] =  -d/2.0;                        // x
    (*f2_offset)[ 1 ] =  d/2.0*tan( DegToRad(30.0) );   // y
    (*f2_offset)[ 2 ] =  h;                             // z

    offset.assign( NUMBER_OF_FINGERS, *f0_offset );
    offset[1] = *f1_offset;
    offset[2] = *f2_offset;
    //
    //---------------------
}
//----------------------------------------------------------------------


cSDH::~cSDH()
{
    if ( IsOpen() )
    {
        cdbg << "Cleanup: Closing port in destructor ~cSDH\n";
        Close();
    }

    if ( com )
    {
        delete com;
        com = NULL;
    }
}
//----------------------------------------------------------------------


std::vector<int> cSDH::ToIndexVector( int index, std::vector<int>& all_replacement, int maxindex, char const* name )
    throw (cSDHLibraryException*)
{
    if (index == All)
    {
        return all_replacement;
    }
    else
    {
        CheckIndex( index, maxindex, name );
        return std::vector<int>( 1, index );
    }
}
//-----------------------------------------------------------------

#if 0
std::vector<double> cSDH::ToValueVector( double value, int length, pDoubleUnitConverterFunction convert )
{
    return std::vector<double>( length, *convert( value ) );
}
#endif

//-----------------------------------------------------------------
pSetFunction cSDH::GetMotorCurrentModeFunction( eMotorCurrentMode mode )
    throw(cSDHLibraryException*)
{
    switch (mode)
    {
    case eMCM_MOVE: return &cSDHSerial::ilim;
    case eMCM_GRIP: return &cSDHSerial::igrip;
    case eMCM_HOLD: return &cSDHSerial::ihold;
    default:
        throw new cSDHErrorInvalidParameter( cMsg( "Unknown mode '%d', not in [0..%d]!", int(mode), eMCM_DIMENSION-1) );
    }
}
//-----------------------------------------------------------------


#if 0
std::vector<std::vector<double> > cSDH::_GetHandXYZ( double angles )
{
    f0_angles = [ angles[i]  for i in finger_axis_index[0] ];
    f1_angles = [ angles[i]  for i in finger_axis_index[1] ];
    f2_angles = [ angles[i]  for i in finger_axis_index[2] ];

    return [_GetFingerXYZ( 0, f0_angles ),  // finger 0
            _GetFingerXYZ( 1, f1_angles ),  // finger 1
            _GetFingerXYZ( 2, f2_angles )]; // finger 2
}
//----------------------------------------------------------------------
#endif


//-----------------------------------------------------------------
std::vector<double> cSDH::_GetFingerXYZ( int fi, std::vector<double> r_angles )
    throw(cSDHLibraryException*)
{
    std::vector<double> rv(3,0.0);

    double fac_x, fac_y;
    switch (fi)
    {
    case 0:
        fac_x = -1.0;
        fac_y = -1.0;
        break;
    case 1:
        fac_x = 1.0;
        fac_y = 1.0;
        break;
    case 2:
        fac_x =  1.0;
        fac_y = -1.0;
        break;
    default:
        throw new cSDHErrorInvalidParameter( cMsg( "Unexpected finger index '%d' not in [0..3]!", fi ) );
    }

    double s_b  = sin( r_angles[1] );
    double s_bc = sin( r_angles[1] + r_angles[2] );
    double l1_s_b_l2_s_bc = (l1*s_b + l2*s_bc);

    rv[0] = fac_x * (l1_s_b_l2_s_bc) * sin( r_angles[0] ) + offset[ fi ][0]; // x
    rv[1] = fac_y * (l1_s_b_l2_s_bc) * cos( r_angles[0] ) + offset[ fi ][1]; // y
    rv[2] = l1*cos( r_angles[1] ) + l2*cos( r_angles[1] + r_angles[2] ) + offset[ fi][2]; // z

    return rv;
}
//----------------------------------------------------------------------


void cSDH::UseRadians( void )
{
    // set unit convert for (axis) angles:
    uc_angle = &uc_angle_radians;

    // set unit convert for (axis) angular velocities:
    uc_angular_velocity = &uc_angular_velocity_radians_per_second;

    // set unit convert for (axis) angular accelerations:
    uc_angular_acceleration = &uc_angular_acceleration_radians_per_second_squared;
}
//-----------------------------------------------------------------


void cSDH::UseDegrees( void )
{
    // set unit convert for (axis) angles:
    uc_angle = &uc_angle_degrees;

    // set unit convert for (axis) angular velocities:
    uc_angular_velocity = &uc_angular_velocity_degrees_per_second;

    // set unit convert for (axis) angular accelerations:
    uc_angular_acceleration = &uc_angular_acceleration_degrees_per_second_squared;
}
//-----------------------------------------------------------------


int cSDH::GetFingerNumberOfAxes( int iFinger )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );
    return finger_number_of_axes[iFinger];
}
//-----------------------------------------------------------------


int cSDH::GetFingerAxisIndex( int iFinger, int iFingerAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );
    CheckIndex( iFingerAxis, NUMBER_OF_AXES_PER_FINGER, "finger axis" );

    return finger_axis_index[ iFinger ][ iFingerAxis ];
}
//-----------------------------------------------------------------


char const* cSDH::GetLibraryRelease( void )
{
    return PROJECT_RELEASE;
}
//----------------------------------------------------------------------


char const* cSDH::GetLibraryName( void )
{
    return PROJECT_NAME;
}
//----------------------------------------------------------------------


char const* cSDH::GetFirmwareRelease( void )
    throw (cSDHLibraryException*)
{
    if ( ! comm_interface.IsOpen() )
        throw new cSDHErrorCommunication( "No connection to SDH" );

    return comm_interface.ver();
}
//----------------------------------------------------------------------

//-----------------------------------------------------------------
char const* cSDH::GetInfo( char const* what )
throw (cSDHLibraryException*)
{
    cdbg << "GetInfo: " << what << " is requested\n";

    if ( ! strcmp( what,"release" ) || ! strcmp( what, "release-library" ) )
        return PROJECT_RELEASE;
    if ( ! strcmp( what, "date" ) || ! strcmp( what, "date-library" ) )
        return PROJECT_DATE;

    if ( ! comm_interface.IsOpen() )
        throw new cSDHErrorCommunication("Interface to SDH is not open");

    if ( ! strcmp( what, "release-firmware" ) )
        return comm_interface.ver();
    if ( ! strcmp( what, "date-firmware") )
        return comm_interface.ver_date();
    if ( ! strcmp( what, "release-soc" ) )
        return comm_interface.soc();
    if ( ! strcmp( what, "date-soc" ) )
        return comm_interface.soc_date();
    if ( ! strcmp( what, "date-soc" ) )
        return comm_interface.soc_date();
    if ( ! strcmp( what, "id-sdh") )
        return comm_interface.id();
    if ( ! strcmp( what, "sn-sdh" ) )
        return comm_interface.sn();
    return "?";
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetTemperature( std::vector<int> const& sensors )
    throw (cSDHLibraryException*)
{
    cSimpleVector temperatures_axes = comm_interface.temp();
    cSimpleVector temperatures_electronics = comm_interface.temp_electronics();

    std::vector<double> rv;

    for ( std::vector<int>::const_iterator si = sensors.begin();
          si != sensors.end();
          si++ )
    {
        CheckIndex( *si, NUMBER_OF_TEMPERATURE_SENSORS, "temperature sensor" );

        if ( *si < NUMBER_OF_AXES )
            rv.push_back( uc_temperature->ToExternal( temperatures_axes[ *si ] ) );
        else
            rv.push_back( uc_temperature->ToExternal( temperatures_electronics[ *si - NUMBER_OF_AXES ] ) );
    }
    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetTemperature( int iSensor )
    throw (cSDHLibraryException*)
{
    CheckIndex( iSensor, NUMBER_OF_TEMPERATURE_SENSORS, "temperature sensor" );

    cSimpleVector temperatures;
    if ( iSensor < NUMBER_OF_AXES )
    {
        temperatures = comm_interface.temp();
        return uc_temperature->ToExternal( temperatures[ iSensor ] );
    }
    temperatures = comm_interface.temp_electronics();
    return uc_temperature->ToExternal( temperatures[ iSensor-NUMBER_OF_AXES ] );
}
//----------------------------------------------------------------------


void cSDH::OpenRS232(  int _port, unsigned long _baudrate, double _timeout, char const* _device_format_string )
    throw (cSDHLibraryException*)
{
    //---------------------
    // Try to create a cSDHSerial object and save as member comm_interface:

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cRS232( _port, _baudrate, _timeout, _device_format_string );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-2 (cSDHSerial uses level-1 already)
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenRS232() successfully opened RS232 port.\n";
}
//----------------------------------------------------------------------

void cSDH::OpenCAN_ESD(  int _net,  unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write )
    throw (cSDHLibraryException*)
{
#if WITH_ESD_CAN
    //---------------------
    // Try to create a cSDHSerial object and save as member comm_interface:

    if ( _timeout < 0.0 )
        _timeout = 0.0;

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cCANSerial_ESD( _net, _baudrate, _timeout, _id_read, _id_write );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-1
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenCAN_ESD() successfully opened CAN port.\n";
#else
    throw new cSDHErrorInvalidParameter( cMsg( "Cannot open ESD CAN net: The SDHLibrary was compiled without ESD CAN support" ) );
#endif
}
//----------------------------------------------------------------------

void cSDH::OpenCAN_ESD( tDeviceHandle _ntcan_handle, double _timeout, Int32 _id_read, Int32 _id_write )
    throw (cSDHLibraryException*)
{
#if WITH_ESD_CAN
    //---------------------
    // Try to create a cSDHSerial object and save as member comm_interface:

    if ( _timeout < 0.0 )
        _timeout = 0.0;

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cCANSerial_ESD( _ntcan_handle, _timeout, _id_read, _id_write );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-1
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenCAN_ESD() successfully opened CAN port.\n";
#else
    throw new cSDHErrorInvalidParameter( cMsg( "Cannot open ESD CAN handle: The SDHLibrary was compiled without ESD CAN support" ) );
#endif
}
//----------------------------------------------------------------------

void cSDH::OpenCAN_PEAK( unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write, char const* _device )
    throw (cSDHLibraryException*)
{
#if WITH_PEAK_CAN
    //---------------------
    // Try to create a cSDHSerial object and save as member comm_interface:

    if ( _timeout < 0.0 )
        _timeout = 0.0;

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cCANSerial_PEAK( _baudrate, _timeout, _id_read, _id_write, _device );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-1
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenCAN_PEAK() successfully opened PEAK CAN device \"" << _device << "\".\n";
#else
    throw new cSDHErrorInvalidParameter( cMsg( "Cannot open PEAK CAN device: The SDHLibrary was compiled without PEAK CAN support" ) );
#endif
}
//----------------------------------------------------------------------

void cSDH::OpenCAN_PEAK( tDeviceHandle _handle, double _timeout, Int32 _id_read, Int32 _id_write )
    throw (cSDHLibraryException*)
{
#if WITH_PEAK_CAN
    //---------------------
    // Try to create a cSDHSerial object and save as member comm_interface:

    if ( _timeout < 0.0 )
        _timeout = 0.0;

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cCANSerial_PEAK( _handle, _timeout, _id_read, _id_write );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-1
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenCAN_PEAK() successfully reopened CAN device.\n";
#else
    throw new cSDHErrorInvalidParameter( cMsg( "Cannot open PEAK CAN handle: The SDHLibrary was compiled without PEAK CAN support" ) );
#endif
}
//----------------------------------------------------------------------

void cSDH::OpenTCP( char const* _tcp_adr, int _tcp_port, double _timeout )
    throw (cSDHLibraryException*)
{
    //---------------------
    // Try to create a cTCPSerial object and save as member comm_interface:

    if ( com )
    {
        delete com;
        com = NULL;
    }
    com = new cTCPSerial( _tcp_adr, _tcp_port, _timeout );
    com->dbg.SetFlag( debug_level > 2 ); // set debug level of low level com interface to our own level-1
    comm_interface.Open( com );

    // Now that we're connected update settings from the connected SDH
    UpdateSettingsFromSDH();

    cdbg << "cSDH.OpenTCP() successfully opened TCP connection to \"" << _tcp_adr << ":" << _tcp_port << "\"\n";
}
//----------------------------------------------------------------------


void cSDH::Close( bool leave_enabled )
    throw (cSDHLibraryException*)
{
    if (comm_interface.IsOpen())
    {
        try
        {
            if (! leave_enabled )
            {
                cdbg << "Switching off power before closing connection to SDH\n";
                comm_interface.power( All, &(zeros_v[0]) );
            }

            comm_interface.Close();
            cdbg << "Connection to SDH closed.\n";
        }
        catch ( cSDHLibraryException* e )
        {
            cdbg << "Ignoring exception while closing connection to SDH (" << e->what() << ")\n";
            delete e;
        }
    }
    else
    {
        throw new cSDHErrorCommunication( "No connection to SDH" );
    }
}
//-----------------------------------------------------------------


bool cSDH::IsOpen( void )
    throw()
{
    return comm_interface.IsOpen();
}
//-----------------------------------------------------------------


void cSDH::EmergencyStop( void )
    throw(cSDHLibraryException*)
{
    // switch off controllers
    comm_interface.power( All, &(zeros_v[0]) );

    // save current actual axis angles as new target axis angles
    cSimpleVector angles = comm_interface.pos( All );
    // limit angles to allowed ranges:
    ToRange( angles, GetAxisMinAngle( all_axes ), GetAxisMaxAngle( all_axes) );
    comm_interface.p( All, &(angles[0]) );
}
//-----------------------------------------------------------------


void cSDH::Stop( void )
    throw(cSDHLibraryException*)
{
    comm_interface.stop();
}
//-----------------------------------------------------------------


void cSDH::SetController( eControllerType controller )
    throw(cSDHLibraryException*)
{
    if (controller >= eCT_DIMENSION)
    {
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid controller type %d = '%s'", int(controller), GetStringFromControllerType(controller) ) );
    }

    if ( controller > eCT_POSE && CompareReleases( release_firmware.c_str(), "0.0.2.6" ) < 0 )
    {
        throw new cSDHErrorInvalidParameter( cMsg( "Controller type %d not available in firmware %s of currently attached SDH", controller, release_firmware.c_str() ) );
    }


    if ( controller == eCT_POSE && CompareReleases( release_firmware.c_str(), "0.0.2.6" ) < 0 )
        // nothing more to do here: for firmwares prior to 0.0.2.6 the controller is fixed to eCT_POSE anyway
        // (and setting the controller would yield an error from the firmware (unknown command))
        controller_type = controller;
    else
        controller_type = comm_interface.con(controller);

    AdjustLimits( controller_type );
}
//-----------------------------------------------------------------


cSDHBase::eControllerType cSDH::GetController( void  )
    throw (cSDHLibraryException*)
{
    if ( CompareReleases( release_firmware.c_str(), "0.0.2.6" ) < 0.0 )
    {
        // cannot read controller in firmwares prior to 0.0.2.6 where controller is fixed to POSE
        controller_type =eCT_POSE;
    }
    else
        controller_type = comm_interface.con( eCT_INVALID );

    return controller_type;
}
//-----------------------------------------------------------------


void cSDH::SetVelocityProfile( eVelocityProfile velocity_profile )
    throw (cSDHLibraryException*)
{
    comm_interface.vp( velocity_profile );
}
//-----------------------------------------------------------------


cSDHBase::eVelocityProfile cSDH::GetVelocityProfile( void )
    throw (cSDHLibraryException*)
{
    eVelocityProfile evp = (eVelocityProfile) comm_interface.vp();
    return evp;
    //return (eVelocityProfile) comm_interface.vp();
}
//----------------------------------------------------------------------


void cSDH::SetAxisMotorCurrent( std::vector<int> const& axes, std::vector<double> const& motor_currents, eMotorCurrentMode mode )
    throw (cSDHLibraryException*)
{
    SetAxisValueVector( axes, motor_currents,
                        GetMotorCurrentModeFunction( mode ),
                        GetMotorCurrentModeFunction( mode ),
                        uc_motor_current,
                        f_min_motor_current_v, f_max_motor_current_v,
                        "motor current" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisMotorCurrent( int iAxis, double motor_current, eMotorCurrentMode mode )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes = ToIndexVector( iAxis, all_axes, nb_all_axes, "axis" );
    // now axes is a vector of all axis indices to access

    std::vector<double> motor_currents( axes.size(), motor_current );
    // now motor_currents is a vector of all axis motor currents in external unit
    // system

    SetAxisValueVector( axes, motor_currents,
                        GetMotorCurrentModeFunction( mode ),
                        GetMotorCurrentModeFunction( mode ),
                        uc_motor_current,
                        f_min_motor_current_v, f_max_motor_current_v,
                        "motor current" );
}
//----------------------------------------------------------------------



std::vector<double> cSDH::GetAxisMotorCurrent( std::vector<int> const& axes, eMotorCurrentMode mode )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               GetMotorCurrentModeFunction( mode ),
                               uc_motor_current,
                               "motor current" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisMotorCurrent( int iAxis, eMotorCurrentMode mode )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    pGetFunction get_func = GetMotorCurrentModeFunction( mode );

    return uc_motor_current->ToExternal( (comm_interface.*get_func)( iAxis, NULL )[ iAxis ] );
}
//----------------------------------------------------------------------


void cSDH::SetAxisEnable( std::vector<int> const& axes, std::vector<double> const& states )
    throw (cSDHLibraryException*)
{
    SetAxisValueVector( axes, states,
                        &cSDHSerial::power,
                        &cSDHSerial::power,
                        &uc_identity,
                        zeros_v, ones_v,
                        "enable/disable" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisEnable( int iAxis, double state )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes = ToIndexVector( iAxis, all_axes, nb_all_axes, "axis" );
    // now axes is a list of all axis indices to access

    std::vector<double> states( axes.size(), state );
    // now states is a list of all axis states

    SetAxisValueVector( axes, states,
                        &cSDHSerial::power,
                        &cSDHSerial::power,
                        &uc_identity,
                        zeros_v, ones_v,
                        "enable/disable" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisEnable( std::vector<int> const& axes, std::vector<bool> const& states )
    throw (cSDHLibraryException*)
{
    std::vector<double> dstates( states.size(), 0.0 );
    std::vector<bool>::const_iterator bi;
    std::vector<double>::iterator di;
    for ( bi = states.begin(), di = dstates.begin();
          bi != states.end() && di != dstates.end();
          bi++, di++ )
        *di = *bi;

    SetAxisEnable( axes, dstates );
}
//----------------------------------------------------------------------


void cSDH::SetAxisEnable( int iAxis, bool state )
    throw (cSDHLibraryException*)
{
    SetAxisEnable( iAxis, state ? 1.0 : 0.0 );
}
//----------------------------------------------------------------------



std::vector<double> cSDH::GetAxisEnable( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::power,
                               &uc_identity,
                               "enabled/disabled" );
}
//-----------------------------------------------------------------


double cSDH::GetAxisEnable( int iAxis )
    throw(cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return comm_interface.power( iAxis, NULL )[ iAxis ];
}
//-----------------------------------------------------------------


std::vector<cSDH::eAxisState> cSDH::GetAxisActualState( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    std::vector<double> fstates = GetAxisValueVector( axes,
                                                      &cSDHSerial::state,
                                                      &uc_identity,
                                                      "state" );
    std::vector<eAxisState> estates;
    std::vector<double>::const_iterator si;

    for ( si =  fstates.begin();
          si != fstates.end();
          si++ )
    {
        estates.push_back( eAxisState( int( *si ) ) );
    }

    return estates;
}
//----------------------------------------------------------------------


cSDH::eAxisState cSDH::GetAxisActualState( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );


    return eAxisState( int( comm_interface.state( iAxis, NULL )[ iAxis ] ) );
}
//----------------------------------------------------------------------


void cSDH::WaitAxis( std::vector<int> const& axes, double timeout )
    throw (cSDHLibraryException*)
{
    cSimpleTime start_time;
    bool finished;

    eAxisState busy;
    if ( controller_type == eCT_POSE )
        busy = eAS_POSITIONING;
    else
        busy = eAS_SPEED_MODE;

    do
    {
        std::vector<eAxisState> states = GetAxisActualState( axes );

        finished = true;

        std::vector<eAxisState>::const_iterator si;
        for( si =  states.begin();
             si != states.end();
             si++ )
        {
            finished = finished  &&  (*si != busy );
        }

        if (!finished  &&  timeout >= 0.0  &&  start_time.Elapsed() > timeout )
        {
            throw new cSDHErrorCommunication( cMsg( "Timeout in WaitAxis" ) );
        }
    } while ( ! finished );
}
//-----------------------------------------------------------------


void cSDH::WaitAxis( int iAxis, double timeout )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes;

    if (iAxis == All)
        axes = all_axes;
    else
    {
        CheckIndex( iAxis, nb_all_axes, "axis" );
        axes.push_back( iAxis );
    }

    WaitAxis( axes, timeout );
}
//-----------------------------------------------------------------


void cSDH::SetAxisTargetAngle( std::vector<int> const& axes, std::vector<double> const& angles )
    throw (cSDHLibraryException*)
{
    SetAxisValueVector( axes, angles,
                        &cSDHSerial::p,
                        &cSDHSerial::p,
                        uc_angle,
                        f_min_angle_v, f_max_angle_v,
                        "target angle" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisTargetAngle( int iAxis, double angle )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes = ToIndexVector( iAxis, all_axes, nb_all_axes, "axis" );
    // now axes is a vector of all axis indices to access

    std::vector<double> angles( axes.size(), angle );
    // now angles is a vector of all target axis angles in external unit system

    SetAxisValueVector( axes, angles,
                        &cSDHSerial::p,
                        &cSDHSerial::p,
                        uc_angle,
                        f_min_angle_v, f_max_angle_v,
                        "target angle" );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::SetAxisTargetGetAxisActualAngle( std::vector<int> const& axes, std::vector<double> const& angles )
throw (cSDHLibraryException*)
{
    return SetAxisValueVector( axes, angles,
                               &cSDHSerial::tpap,
                               &cSDHSerial::p,     // when only some target angles are set then we have to get the others from the SDH first with the ordinary cSDHSerial::p()
                               uc_angle,
                               f_min_angle_v, f_max_angle_v,
                               "set target, get actual angle" );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisTargetAngle( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::p,
                               uc_angle,
                               "target angle" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisTargetAngle( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( comm_interface.p( iAxis, NULL )[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisActualAngle( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::pos,
                               uc_angle,
                               "target angle" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisActualAngle( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( comm_interface.pos( iAxis )[ iAxis ] );
}
//----------------------------------------------------------------------


void cSDH::SetAxisTargetVelocity( std::vector<int> const& axes, std::vector<double> const& velocities )
    throw (cSDHLibraryException*)
{
    SetAxisValueVector( axes, velocities,
                        &cSDHSerial::v,
                        &cSDHSerial::v,
                        uc_angular_velocity,
                        f_min_velocity_v, f_max_velocity_v,
                        "target velocity" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisTargetVelocity( int iAxis, double velocity )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes = ToIndexVector( iAxis, all_axes, nb_all_axes, "axis" );
    // now axes is a vector of all axis indices to access

    std::vector<double> velocities( axes.size(), velocity );
    // now velocities is a vector of all target axis velocities in external unit system

    SetAxisValueVector( axes, velocities,
                        &cSDHSerial::v,
                        &cSDHSerial::v,
                        uc_angular_velocity,
                        f_min_velocity_v, f_max_velocity_v,
                        "target velocity" );
}
//----------------------------------------------------------------------

std::vector<double> cSDH::SetAxisTargetGetAxisActualVelocity( std::vector<int> const& axes, std::vector<double> const& velocities )
throw (cSDHLibraryException*)
{
    return SetAxisValueVector( axes, velocities,
                               &cSDHSerial::tvav,
                               &cSDHSerial::v,         // when not all target velocities are set then we have to get the others from the SDH first with the ordinary cSDHSerial::v()
                               uc_angular_velocity,
                               f_min_velocity_v, f_max_velocity_v,
                               "set target, get actual velocity" );
}
//----------------------------------------------------------------------

std::vector<double> cSDH::GetAxisTargetVelocity( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::v,
                               uc_angular_velocity,
                               "target velocity" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisTargetVelocity( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angular_velocity->ToExternal( comm_interface.v( iAxis, NULL )[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisLimitVelocity( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    if ( CompareReleases( release_firmware.c_str(), "0.0.2.1" ) < 0 )
    {
        // if firmware is older than "0.0.2.1" then use fixed default:
        double all_velocities[] = { 85.7, 200.0, 157.8, 200.0, 157.8, 200.0, 157.8, 85.7 };
        std::vector<double> rv;
        std::vector<int>::const_iterator ai;

        for ( ai = axes.begin();
              ai != axes.end();
              ai++ )
        {
            rv.push_back( uc_angular_velocity->ToExternal( all_velocities[ *ai ] ) );
        }
        return rv;
    }

    return GetAxisValueVector( axes,
                               &cSDHSerial::vlim,
                               uc_angular_velocity,
                               "velocity limit" );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisLimitAcceleration( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    if ( CompareReleases( release_firmware.c_str(), "0.0.2.7" ) < 0 )
    {
        // firmware before 0.0.2.7 does not provide the acceleration limit
        // so use fake default:
        double all_accelerations[] = { 5000.0, 400.0, 1500.0, 400.0, 1500.0, 400.0, 1500.0, 400.0 };
        std::vector<double> rv;
        std::vector<int>::const_iterator ai;

        for ( ai = axes.begin();
              ai != axes.end();
              ai++ )
        {
            rv.push_back( uc_angular_acceleration->ToExternal( all_accelerations[ *ai ] ) );
        }
        return rv;
    }

    return GetAxisValueVector( axes,
                               &cSDHSerial::alim,
                               uc_angular_acceleration,
                               "acceleration limit" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisLimitVelocity( int iAxis )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes( 1, iAxis );

    return GetAxisLimitVelocity( axes )[0];
}
//----------------------------------------------------------------------


double cSDH::GetAxisLimitAcceleration( int iAxis )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes( 1, iAxis );

    return GetAxisLimitAcceleration( axes )[0];
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisActualVelocity( std::vector<int>const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::vel,
                               uc_angular_velocity,
                               "actual velocity" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisActualVelocity( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angular_velocity->ToExternal( comm_interface.vel( iAxis )[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisReferenceVelocity( std::vector<int>const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::rvel,
                               uc_angular_velocity,
                               "reference velocity" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisReferenceVelocity( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angular_velocity->ToExternal( comm_interface.rvel( iAxis )[ iAxis ] );
}
//----------------------------------------------------------------------


void cSDH::SetAxisTargetAcceleration( std::vector<int>const& axes, std::vector<double>const& accelerations )
    throw (cSDHLibraryException*)
{
    SetAxisValueVector( axes, accelerations,
                        &cSDHSerial::a,
                        &cSDHSerial::a,
                        uc_angular_acceleration,
                        f_min_acceleration_v, f_max_acceleration_v,
                        "target acceleration" );
}
//----------------------------------------------------------------------


void cSDH::SetAxisTargetAcceleration( int iAxis, double acceleration )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes = ToIndexVector( iAxis, all_axes, nb_all_axes, "axis" );
    // now axes is a vector of all axis indices to access

    std::vector<double> accelerations( axes.size(), acceleration );
    // now accelerations is a vector of all target axis accelerations in external unit system

    SetAxisValueVector( axes, accelerations,
                        &cSDHSerial::a,
                        &cSDHSerial::a,
                        uc_angular_acceleration,
                        f_min_acceleration_v, f_max_acceleration_v,
                        "target acceleration" );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisTargetAcceleration( std::vector<int>const& axes )
    throw (cSDHLibraryException*)
{
    return GetAxisValueVector( axes,
                               &cSDHSerial::a,
                               uc_angular_acceleration,
                               "target acceleration" );
}
//----------------------------------------------------------------------


double cSDH::GetAxisTargetAcceleration( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angular_acceleration->ToExternal( comm_interface.a( iAxis, NULL )[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisMinAngle( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    std::vector<double> rv( axes.size(), 0.0 );

    std::vector<int>::const_iterator ai;
    std::vector<double>::iterator    vi;
    for( ai = axes.begin(), vi = rv.begin();
         ai != axes.end();
         ai++, vi++ )
    {
        CheckIndex( *ai, nb_all_axes, "axis" );
        *vi = uc_angle->ToExternal( f_min_angle_v[ *ai ] );
    }

    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetAxisMinAngle( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( f_min_angle_v[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisMaxAngle( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    std::vector<double> rv( axes.size(), 0.0 );

    std::vector<int>::const_iterator ai;
    std::vector<double>::iterator    vi;
    for( ai = axes.begin(), vi = rv.begin();
         ai != axes.end();
         ai++, vi++ )
    {
        CheckIndex( *ai, nb_all_axes, "axis" );
        *vi = uc_angle->ToExternal( f_max_angle_v[ *ai ] );
    }

    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetAxisMaxAngle( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( f_max_angle_v[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisMaxVelocity( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    std::vector<double> rv( axes.size(), 0.0 );

    std::vector<int>::const_iterator ai;
    std::vector<double>::iterator    vi;
    for( ai = axes.begin(), vi = rv.begin();
         ai != axes.end();
         ai++, vi++ )
    {
        CheckIndex( *ai, nb_all_axes, "axis" );
        *vi = uc_angle->ToExternal( f_max_velocity_v[ *ai ] );
    }

    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetAxisMaxVelocity( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( f_max_velocity_v[ iAxis ] );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetAxisMaxAcceleration( std::vector<int> const& axes )
    throw (cSDHLibraryException*)
{
    std::vector<double> rv( axes.size(), 0.0 );

    std::vector<int>::const_iterator ai;
    std::vector<double>::iterator    vi;
    for( ai = axes.begin(), vi = rv.begin();
         ai != axes.end();
         ai++, vi++ )
    {
        CheckIndex( *ai, nb_all_axes, "axis" );
        *vi = uc_angle->ToExternal( f_max_acceleration_v[ *ai ] );
    }

    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetAxisMaxAcceleration( int iAxis )
    throw (cSDHLibraryException*)
{
    CheckIndex( iAxis, nb_all_axes, "axis" );

    return uc_angle->ToExternal( f_max_acceleration_v[ iAxis ] );
}
//----------------------------------------------------------------------


void cSDH::SetFingerEnable( std::vector<int> const& fingers, std::vector<double> const& states )
    throw (cSDHLibraryException*)
{
#if SDH_USE_VCC
    // VCC does not know nan to create a NAN value
    unsigned long nan[2]={0xffffffff, 0x7fffffff};
    std::vector<double> all_states( NUMBER_OF_AXES, *( double* )nan);
#else
    std::vector<double> all_states( NUMBER_OF_AXES, nan("") );
#endif

    std::vector<int>::const_iterator fi;
    std::vector<double>::const_iterator vi;
    for( fi  = fingers.begin(), vi  = states.begin();
         fi != fingers.end() && vi != states.end();
         fi++, vi++ )
    {
        CheckIndex( *fi, NUMBER_OF_FINGERS, "finger" );

        std::vector<int>::const_iterator fai;
        for( fai =  finger_axis_index[ *fi ].begin();
         fai != finger_axis_index[ *fi ].end();
         fai++ )
        {
        if ( *fai == 0  &&  !SDH_ISNAN( all_states[ *fai ] )  && !SDH_ISNAN( *vi ) )
        {
            // special treatment for axis 0:
            //   "or-together" all given states, so that axis
            //   0 is enabled if at least one finger using it
            //   is eanabled.

            all_states[ *fai ] += *vi;
        }
        all_states[ *fai ] = *vi;
        }
    }

    // limit all_states[ 0 ] to [0.0 .. 1.0]
    if ( !SDH_ISNAN( all_states[ 0 ] ) )
        all_states[ 0 ] = ToRange( all_states[ 0 ], 0.0, 1.0 );

    SetAxisEnable( all_axes, all_states );
}
//----------------------------------------------------------------------


void cSDH::SetFingerEnable( std::vector<int> const& fingers, std::vector<bool> const& states )
    throw (cSDHLibraryException*)
{
    std::vector<double> dstates( states.size(), 0.0 );
    std::vector<bool>::const_iterator bi;
    std::vector<double>::iterator di;
    for ( bi = states.begin(), di = dstates.begin();
          bi != states.end() && di != dstates.end();
          bi++, di++ )
        *di = *bi;

    SetFingerEnable( fingers, dstates );

}
//----------------------------------------------------------------------


void cSDH::SetFingerEnable( int iFinger, double state )
    throw (cSDHLibraryException*)
{
    std::vector<int> axes;

    if ( iFinger == All )
    {
        axes = all_axes;
    }
    else
    {
        CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );
        axes = finger_axis_index[ iFinger ];
    }
    // now axes is a list of all axis indices to access

    std::vector<double> states( axes.size(), state );
    // now states is a list of all axis states to set

    SetAxisValueVector( axes, states,
                        &cSDHSerial::power,
                        &cSDHSerial::power,
                        &uc_identity,
                        zeros_v, ones_v,
                        "enable/disable" );
}
//----------------------------------------------------------------------


void cSDH::SetFingerEnable( int iFinger, bool state )
    throw (cSDHLibraryException*)
{
    SetFingerEnable( iFinger, state ? 1.0 : 0.0 );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerEnable( std::vector<int> const& fingers )
    throw (cSDHLibraryException*)
{
    std::vector<double> rv;

    std::vector<double> all_states = GetAxisEnable( all_axes );


    std::vector<int>::const_iterator fi;

    for( fi  = fingers.begin();
         fi != fingers.end();
         fi++ )
    {
        CheckIndex( *fi, NUMBER_OF_FINGERS, "finger" );

        double finger_state = 0.0;
        std::vector<int>::const_iterator fai;
        for( fai =  finger_axis_index[ *fi ].begin();
         fai != finger_axis_index[ *fi ].end();
         fai++ )
        {
        if ( IsVirtualAxis( *fai ) )
            finger_state += 1.0; // count virtual axes as enabled
        else
            finger_state += all_states[ *fai ];
        }

        rv.push_back( (finger_state == double(NUMBER_OF_AXES_PER_FINGER)) ? 1.0 : 0.0 );
    }
    return rv;
}
//----------------------------------------------------------------------


double cSDH::GetFingerEnable( int iFinger )
    throw (cSDHLibraryException*)
{
    return GetFingerEnable( std::vector<int>( 1, iFinger ) )[0];
}
//----------------------------------------------------------------------


void cSDH::SetFingerTargetAngle( int iFinger, std::vector<double> const& angles )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    SetAxisTargetAngle( finger_axis_index[ iFinger ], angles );
}
//----------------------------------------------------------------------


void cSDH::SetFingerTargetAngle( int iFinger, double a0, double a1, double a2 )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    std::vector<double> angles( NUMBER_OF_AXES_PER_FINGER, a0 );
    angles[1] = a1;
    angles[2] = a2;

    SetAxisTargetAngle( finger_axis_index[ iFinger ], angles );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerTargetAngle( int iFinger )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    return GetAxisTargetAngle( finger_axis_index[iFinger] );
}
//----------------------------------------------------------------------


void cSDH::GetFingerTargetAngle( int iFinger, double& a0, double& a1, double& a2 )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    std::vector<double> angles = GetAxisTargetAngle( finger_axis_index[iFinger] );
    a0 = angles[0];
    a1 = angles[1];
    a2 = angles[2];
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerActualAngle( int iFinger )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    return GetAxisActualAngle( finger_axis_index[iFinger] );
}
//----------------------------------------------------------------------


void cSDH::GetFingerActualAngle( int iFinger, double& a0, double& a1, double& a2 )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    std::vector<double> angles = GetAxisActualAngle( finger_axis_index[iFinger] );
    a0 = angles[0];
    a1 = angles[1];
    a2 = angles[2];
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerMinAngle( int iFinger )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    return GetAxisMinAngle( finger_axis_index[iFinger] );
}
//----------------------------------------------------------------------


void cSDH::GetFingerMinAngle( int iFinger, double& a0, double& a1, double& a2 )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    std::vector<double> angles = GetAxisMinAngle( finger_axis_index[iFinger] );
    a0 = angles[0];
    a1 = angles[1];
    a2 = angles[2];
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerMaxAngle( int iFinger )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    return GetAxisMaxAngle(finger_axis_index[iFinger] );
}
//----------------------------------------------------------------------


void cSDH::GetFingerMaxAngle( int iFinger, double& a0, double& a1, double& a2 )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    std::vector<double> angles = GetAxisMaxAngle( finger_axis_index[iFinger] );
    a0 = angles[0];
    a1 = angles[1];
    a2 = angles[2];
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerXYZ( int iFinger, std::vector<double> const& angles )
    throw (cSDHLibraryException*)
{
    CheckIndex( iFinger, NUMBER_OF_FINGERS, "finger" );

    // we need angle in radians
    std::vector<double> r_angles;

    if (uc_angle != &uc_angle_radians)
    {
        // angles is in other units

        // convert to internal (deg)
        std::vector<double> d_angles = uc_angle->ToInternal( angles );
        // convert to rad
        r_angles = map( DegToRad, d_angles );
    }

    // now r_angles is in rad

    // get xyz in internal unit (mm) and return converted to external unit
    return uc_position->ToExternal( _GetFingerXYZ( iFinger, r_angles ) );
}
//----------------------------------------------------------------------


std::vector<double> cSDH::GetFingerXYZ( int iFinger, double a0, double a1, double a2 )
    throw (cSDHLibraryException*)
{
    std::vector<double> a012;
    a012.push_back( a0 );
    a012.push_back( a1 );
    a012.push_back( a2 );

    return GetFingerXYZ( iFinger, a012 );
}
//----------------------------------------------------------------------


double cSDH::MoveAxis( std::vector<int>const& axes, bool sequ )
    throw (cSDHLibraryException*)
{
    // save currently set target axis angles of all axes in external units
    std::vector<double> t_angles = GetAxisTargetAngle( all_axes );

    // save current actual axis angles of all axes in external units
    std::vector<double> a_angles = GetAxisActualAngle( all_axes );

    // The angle positions might be reported slightly out of range if
    // an axis is close to the limit of its range. This is due to
    // inaccuracies of the absolute encoders. (Don't worry, this does
    // not influence repetitive accuracy, only absolute accuracy.)

    // Therefore we must limit the read actual angle back to the
    // allowed range to be able to send it back to the hand without
    // getting an exception:
    ToRange( a_angles,                                // a_angle is in external units
             uc_angle->ToExternal( f_min_angle_v ),   // so convert min and max
             uc_angle->ToExternal( f_max_angle_v ) ); // to external untis too first

    //---------------------
    // generate new target axis angles:
    // - actual axis angle for not selected axes axes
    // - target axis angle for iAxiss axes
    //
    for ( std::vector<int>::const_iterator ai = axes.begin();
          ai != axes.end();
          ai++ )
    {
        CheckIndex( *ai, nb_all_axes, "axis" );

        if ( IsVirtualAxis( *ai ) )
            continue;   // ignore virtual axes

        // set new target axis angles for the axes selected by the
        // given axes index vector \a axes into the actual axis
        // angles variable \a a_angles
        a_angles[ *ai ] = t_angles[ *ai ] ;
    }
    //---------------------

    // set modified actual axis angles as new target axis angles
    SetAxisTargetAngle( all_axes, a_angles );
    // and move there
    double t = comm_interface.m(sequ);

    // restore the saved target axis angles so that
    // previously set target axis angles for unselected axes remain
    // active
    if (sequ)
    {
        SetAxisTargetAngle( all_axes, t_angles );
    }

    return uc_time->ToExternal( t );
}
//----------------------------------------------------------------------


double cSDH::MoveAxis( int iAxis, bool sequ )
    throw (cSDHLibraryException*)
{
    if ( iAxis==All )
        return MoveAxis( all_axes, sequ );
    else
        return MoveAxis( std::vector<int>( 1, iAxis ), sequ );
}
//----------------------------------------------------------------------


double cSDH::MoveFinger( std::vector<int>const& fingers, bool sequ )
    throw (cSDHLibraryException*)
{
    // save currently set target axis angles of all axes in external units
    std::vector<double> t_angles = GetAxisTargetAngle( all_axes );

    // save current actual axis angles of all axes in external units
    std::vector<double> a_angles = GetAxisActualAngle( all_axes );

    // The angle positions might be reported slightly out of range if
    // an axis is close to the limit of its range. This is due to
    // inaccuracies of the absolute encoders. (Don't worry, this does
    // not influence repetitive accuracy, only absolute accuracy.)

    // Therefore we must limit the read actual angle back to the
    // allowd range to be able to send it back to the hand without
    // getting an exception:
    ToRange( a_angles,                                // a_angle is in external units
             uc_angle->ToExternal( f_min_angle_v ),   // so convert min and max
             uc_angle->ToExternal( f_max_angle_v ) ); // to external untis too first

    //---------------------
    // generate new target axis angles:
    // - actual axis angle for not selected fingers axes
    // - target axis angle for iFingers axes
    //
    for ( std::vector<int>::const_iterator fi = fingers.begin();
          fi != fingers.end();
          fi++ )
    {
        CheckIndex( *fi, NUMBER_OF_FINGERS, "finger" );

        for ( std::vector<int>::const_iterator fai = finger_axis_index[ *fi ].begin();
          fai != finger_axis_index[ *fi ].end();
          fai++ )
        {
            if ( IsVirtualAxis( *fai ) )
                continue;   // ignore virtual axes

            // set new target axis angles for the axes of finger *fi in actual axis angles
            a_angles[ *fai ] = t_angles[ *fai ] ;
        }
    }
    //---------------------

    // set modified actual axis angles as new target axis angles
    SetAxisTargetAngle( all_axes, a_angles );
    // and move there
    double t = comm_interface.m(sequ);

    // restore the saved target axis angles so that
    // previously set target axis angles for unselected fingers remain
    // active
    if (sequ)
    {
        SetAxisTargetAngle( all_axes, t_angles );
    }

    return uc_time->ToExternal( t );
}
//----------------------------------------------------------------------


double cSDH::MoveFinger( int iFinger, bool sequ )
    throw (cSDHLibraryException*)
{
    if ( iFinger==All )
        return MoveFinger( all_fingers, sequ );
    else
        return MoveFinger( std::vector<int>( 1, iFinger ), sequ );
}
//----------------------------------------------------------------------


double cSDH::MoveHand( bool sequ )
    throw(cSDHLibraryException*)
{
    return MoveFinger( all_fingers, sequ );
}
//----------------------------------------------------------------------


double cSDH::GetGripMaxVelocity( void )
{
    return uc_angular_velocity->ToExternal( grip_max_velocity );
}
//----------------------------------------------------------------------


double cSDH::GripHand( eGraspId grip, double close, double velocity, bool sequ )
    throw (cSDHLibraryException*)
{
    CheckRange( close, 0.0, 1.0, "open/close ratio" );
    CheckRange( velocity, 0.0, grip_max_velocity, "grip velocity" );

    double t0 = comm_interface.selgrip( grip, true ); // the selgrip must always be sequential

    double t1 = comm_interface.grip( close, uc_angular_velocity->ToInternal( velocity ), sequ );

    return uc_time->ToExternal( t0 + t1 );
}
//----------------------------------------------------------------------


void cSDH::UpdateSettingsFromSDH()
{
    // get the actual realease of the firmware (some code has to be version specific)
    release_firmware = GetInfo("release-firmware");

    // use the real axis velocity limits as reported by the firmware via the vlim command
    f_max_velocity_v = GetAxisLimitVelocity(all_real_axes);

    // virtual axes will have maximum of all other velocities as limit velocity
    // Useful to make SetAxisTargetVelocity( All, x ) work,
    f_max_velocity_v.push_back( *max_element( f_max_velocity_v.begin(), f_max_velocity_v.end() ) );


    // use the real axis acceleration limits as reported by the firmware via the alim command
    f_max_acceleration_v = GetAxisLimitAcceleration(all_real_axes);

    // virtual axes will have maximum of all other accelerations as limit acceleration
    // usefull to make SetAxisTargetAcceleration( All, x ) work,
    f_max_acceleration_v.push_back( *max_element( f_max_acceleration_v.begin(), f_max_acceleration_v.end() ) );

    AdjustLimits( GetController() );
}
//----------------------------------------------------------------------


void cSDH::AdjustLimits( eControllerType controller )
{
    f_min_acceleration_v = zeros_v;
    switch (controller)
    {
    case eCT_POSE:
        // in pose controller the velocities are always positive and thus the minimum is 0.0
        f_min_velocity_v     = zeros_v;
        break;

    case eCT_VELOCITY:
        // no break here
    case eCT_VELOCITY_ACCELERATION:
        // in velocity based controllers the velocities can be positive or negative and thus the minimum is -maximum
        for ( int i = 0; i < nb_all_axes; i++ )
        {
            f_min_velocity_v[i]     = -f_max_velocity_v[i];
        }
        break;

    case eCT_INVALID:
    case eCT_DIMENSION:
        assert( "controller invalid" == NULL );
    }

    cdbg << "AdjustLimits( " << GetStringFromControllerType( controller ) << " )\n";
    cdbg << "  f_min_velocity_v = " << f_min_velocity_v << "   ";
    cdbg << "  f_min_acceleration_v = " << f_min_acceleration_v << "\n";
}

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
