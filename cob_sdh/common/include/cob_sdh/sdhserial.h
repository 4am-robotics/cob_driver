//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhserial_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSDHSerial.

  \section sdhlibrary_cpp_sdhserial_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhserial_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
      \par SVN file revision:
        $Id: sdhserial.h 3686 2008-10-13 15:07:24Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhserial_h_changelog Changelog of this file:
      \include sdhserial.h.log
*/
//======================================================================

#ifndef SDHSERIAL_H_
#define SDHSERIAL_H_

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "sdhexception.h"
#include "simplevector.h"
#include "simplestringlist.h"
#include "sdhbase.h"
#include "serialbase.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


/*!
  \brief The class to communicate with a SDH via RS232.

   End-Users should \b NOT use this class directly! The interface
   of cSDHSerial is subject to change in future releases. End users
   should use the class cSDH instead, as that interface is considered
   more stable.

   <hr>
*/
class cSDHSerial : public cSDHBase
{
protected:
    /*!
        \brief additional time in seconds to wait for sequential execution of m
         command (as these are always executed non-sequentially by the
         firmware
    */
    double m_sequtime;

    //! String to use as "End Of Line" marker when sending to SDH
    char* EOL;

    //! The communication object to the serial device (RS232 port or ESD CAN net)
    cSerialBase* com;


    //! Space for the replies from the SDH
    cSimpleStringList reply;

    //! number of remaining reply lines of a previous (non-sequential) command
    int nb_lines_to_ignore;

public:
    //#################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_internal
       \name   Internal methods

       @{
    */

    //-----------------------------------------------------------------
    /*!
      \brief Constructor of cSDHSerial.

       \param _debug_level : debug level of the created object.
                             If the \a debug_level of an object is > 0 then
                             it will output debug messages.
                             (forwared to constructor of base class)
       <hr>
    */
    cSDHSerial( int _debug_level=0 );


    //-----------------------------------------------------------------
    /*!
      virtual destructor to make compiler happy
    */
    virtual ~cSDHSerial()
    {}


    //-----------------------------------------------------------------
    /*!
       Open the serial device and check connection to SDH by querying
       the SDH firmware version

       \param _com - ptr to the serial device to use

       This may throw an exception on failure.

       The serial port on the PC-side can be opened successfully even
       if no SDH is attached. Therefore this routine tries to read the
       SDH firmware version with a 1s timeout after the port is
       opened. If the SDH does not reply in time then
       - an error message is printed on stderr,
       - the port is closed
       - and a cSerialBaseException* exception is thrown.

       <hr>
    */
    void Open( cSerialBase* _com )
        throw(cSDHLibraryException*);


    /*!
        Close connection to serial port.
    */
    void Close()
        throw (cSDHLibraryException*);


    /*!
        Return true if connection to SDH firmware/hardware is open
    */
    virtual bool IsOpen( void );


    /*!
        Send command string s+EOL to com and read reply according to nb_lines.

        If nb_lines == All then reply lines are read until a line
        without "@" prefix is found.
        If nb_lines != All it is the number of lines to read.

        firmware_state is set according to reply (if read)
        nb_lines_total contains the total number of lines replied for
        the s command. If fewer lines are read then
        nb_lines_total-nb_lines will be remembered to be ignored
        before the next command can be sent.

        Return a list of all read lines of the reply from the SDH hardware.
    */
    void Send( char const* s, int nb_lines=All, int nb_lines_total=All, int max_retries=3 )
        throw( cSDHLibraryException* );


    /*!
        Try to extract the state of the SDH firmware from the last reply
    */
    void ExtractFirmwareState()
        throw( cSDHErrorCommunication* );


    /*!
        Return duration of the execution of a SDH command as reported by line
    */
    double GetDuration( char* line )
        throw( cSDHErrorCommunication* );


    /*!
    Send get_duration command. Returns the calculated duration of the
    currently configured movement (target positions, velocities,
    accelerations and velocity profile.

    return the expected duration of the execution of the command in seconds
    */
    double get_duration( void );

    /*!
      Read all pending lines from SDH to resync execution of PC and SDH.
     */
    void Sync( )
        throw( cSDHErrorCommunication* );

    /*!
      Read an unknown number of lines from SDH to resync execution of PC and SDH.
     */
    void SyncUnknown( )
        throw( cSDHErrorCommunication* );


    /*!
        Get/Set values.

        - If axis is All and value is None then a
          NUMBER_OF_AXES-list of the actual values
          read from the SDH is returned
        - If axis is a single number and value is None then the
          actual value for that axis is read from the SDH and is returned
        - If axis and value are single numbers then that value
          is set for that axis and returned.
        - If axis is All and value is a NUMBER_OF_AXES-vector then all axes
          values are set accordingly, a NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector AxisCommand( char* command, int axis=All, double* value=NULL )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_csdhserial_internal
    //! @}
    //################################################################

    //################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_setup_commands
       \name   Setup and configuration methods
       @{
    */

    //-----------------------------------------------------------------
    /*!
        Get/Set PID controller parameters

        - axis must be a single number: the index of the axis to get/set
        - If p,i,d are None then a list of the actually
          set PID controller parameters of the axis is returned
        - If p,i,d are numbers then the PID controller parameters for
          that axis are set (and returned).
    */
    cSimpleVector pid( int axis, double* p=NULL, double* i=NULL, double* d=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set kv parameter

        - If axis is All and kv is None then a
          NUMBER_OF_AXES-list of the actually set kv parameters is returned
        - If axis is a single number and kv is None then the
          kv parameter for that axis is returned.
        - If axis and kv are single numbers then the kv parameter
          for that axis is set (and returned).
        - If axis is All and kv is a NUMBER_OF_AXES-vector then all axes
          kv parameters are set accordingly, NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector kv( int axis=All, double* kv=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set actual motor current limit for m command

        - If axis is All and limit is None then a NUMBER_OF_AXES-list
          of the actually set motor current limits is returned
        - If axis is a single number and limit is None then the
          motor current limit for that axis is returned.
        - If axis and limit are single numbers then the motor current limit
          for that axis is set (and returned).
        - If axis is All and limit is a NUMBER_OF_AXES-vector then
          all axes motor current limits are set accordingly, the NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector ilim( int axis=All, double* limit=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set actual power state

        - If axis is All and flag is None then a NUMBER_OF_AXES-list
          of the actually set power states is returned
        - If axis is a single number and flag is None then the
          power state for that axis is returned.
        - If axis is a single number and flag is a single number or a
          boolean value then the power state
          for that axis is set (and returned).
        - If axis is All and flag is a NUMBER_OF_AXES-vector then all axes
          power states are set accordingly, the NUMBER_OF_AXES-list is returned.
        - If axis is All and flag is a a single number or a boolean
          value then all axes power states are set to that value, the
          NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector power( int axis=All, double* flag=NULL )
        throw (cSDHLibraryException*);

    //  end of doxygen name group sdhlibrary_cpp_csdhserial_setup_commands
    //! @}
    //#################################################################


    //#################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_misc_commands
       \name   Misc. methods
       @{
    */

    //-----------------------------------------------------------------
    /*!
        Enable/disable SCHUNK demo
    */
    void demo( bool onoff );

    //-----------------------------------------------------------------
    /*!
        Set named property

        Valid propnames are:
        - "user_errors"
        - "terminal"
        - "debug"
    */
    int property( char* propname, int value );

    //-----------------------------------------------------------------
    /*!
    */
    int user_errors( int value );

    //-----------------------------------------------------------------
    /*!
    */
    int terminal( int value );

    //-----------------------------------------------------------------
    /*!
    */
    int debug( int value );

    //-----------------------------------------------------------------

    //  end of doxygen name group sdhlibrary_cpp_csdhserial_misc_commands
    //! @}
    //#################################################################

    //#################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_movement_commands
       \name   Movement methods
       @{
    */

    //-----------------------------------------------------------------
    /*!
        Get/Set target velocity. (NOT the actual velocity!)

        The default velocity is 40 deg/s for axes 0-6.
        Due to some firmware bug axis 0 can go no faster than 14 deg/s

        - If axis is All and velocity is None then a NUMBER_OF_AXES-list
          of the actually set target velocities is returned
        - If axis is a single number and velocity is None then the
          target velocity for that axis is returned.
        - If axis and velocity are single numbers then the target
          velocity for that axis is set (and returned).
        - If axis is All and velocity is a NUMBER_OF_AXES-vector
          then all axes target velocities are set accordingly, the NUMBER_OF_AXES-list
          is returned.

        Velocities are set/reported in degrees per second.
    */
    cSimpleVector v( int axis=All, double* velocity=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get velocity limits.

        - If axis is All then a NUMBER_OF_AXES-list
          of the velocity limits is returned
        - If axis is a single number then the
          velocity limit for that axis is returned.

        dummy parameter is just needed to make this function have the
        same signature as e.g. v(), so it can be used as a function
        pointer.

        Velocity limits are reported in degrees per second.
    */
    cSimpleVector vlim( int axis=All, double* dummy=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set target acceleration for axis. (NOT the actual acceleration!)

        - If axis is All and acceleration is None then a NUMBER_OF_AXES-list
          of the actually set target accelerations is returned
        - If axis is a single number and acceleration is None then the
          target acceleration for that axis is returned.
        - If axis and acceleration are single numbers then the target
          acceleration for that axis is set (and returned).
        - If axis is All and acceleration is a NUMBER_OF_AXES-vector
          then all axes target accelerations are set accordingly, the NUMBER_OF_AXES-list
          is returned.

        Accelerations are set/reported in degrees per second squared.
    */
    cSimpleVector a( int axis=All, double* acceleration=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set target angle for axis. (NOT the actual angle!)

        - If axis is All and angle is None then a NUMBER_OF_AXES-list
          of the actually set target angles is returned
        - If axis is a single number and angle is None then the
          target angle for that axis is returned.
        - If axis and angle are single numbers then the target
          angle for that axis is set (and returned).
        - If axis is All and angle is a NUMBER_OF_AXES-vector
          then all axes target angles are set accordingly, the NUMBER_OF_AXES-list
          is returned.

        Angles are set/reported in degrees.
    */
    cSimpleVector p( int axis=All, double* angle=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Send move command. Moves all enabled axes to their previously
        set target angle. The movement duration is determined by
        that axis that takes longest with its actually set velocity.
        The actual velocity of all other axes is set so that all axes
        begin and end their movements synchronously.

        If sequ is True then wait until SDH hardware fully executed
        the command.  Else return immediately and do not wait until SDH
        hardware fully executed the command.

        return the expected duration of the execution of the command in seconds
    */
    double m( bool sequ )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Stop sdh.

        Will NOT interrupt a previous "selgrip" or "grip" command, only an "m" command!
    */
    void stop( void )
        throw(cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get/set velocity profile.

        If \a velocity_profile is < 0 then the actually set velocity profile is
        read from the firmware and returned. Else the given velocity_profile type
        is set in the firmware if valid.
    */
    eVelocityProfile vp( eVelocityProfile velocity_profile = eVP_INVALID )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_csdhserial_movement_commands
    //! @}
    //#################################################################

    //#################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_diagnosis_commands
       \name   Diagnostic and identification methods
       @{
    */

    //-----------------------------------------------------------------
    /*!
        Get actual angle/s of axis/axes.

        - If axis is All then a NUMBER_OF_AXES-vector of the actual
          axis angles is returned
        - If axis is a single number then the
          actual angle of that axis is returned.

        Angles are reported in degrees.

        \remark
          \a dummy ptr is never used, but needed nonetheless to make
          the signature of the function the same as for the other
          axis-access functions. This way a pointer to it can be used as
          a pointer to the other functions, which is needed by the
          generic cSDH::SetAxisValue and cSDH::GetAxisValue functions.
    */
    cSimpleVector pos( int axis=All, double* dummy=NULL )
        throw (cSDHLibraryException*);

    /*!
        Save actual angle/s to non volatile memory. (Usefull for axes that dont have an absolute encoder)

        - If value is None then an exception is thrown since
          this is NOT usefull if any axis has an absolute encoder that
          the LLC knows about since these positions will be invalidated at the next start
        - If axis and value are single numbers then that axis is saved.
        - If axis is All and value is a NUMBER_OF_AXES-vector
          then all axes are saved if the corresponding value is 1.
        - This will yield a E_RANGE_ERROR if any of the given values is not  0 or 1
    */
    //-----------------------------------------------------------------
    cSimpleVector pos_save( int axis=All, double* value=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------

    /*!
        Do reference movements with selected axes. (Usefull for axes that dont have an absolute encoder)

        each \a value must be either
        - 0 : do not reference
        - 1 : reference till mechanical block in positive direction
        - 2 : reference till mechanical block in negative direction

        - If \a axis and \a value are single numbers then that axis is referenced as requested
        - If \a axis is All and \a value is a NUMBER_OF_AXES-vector
          then all axes are referenced as requested.
        - This will yield a E_RANGE_ERROR if any of the given values is not  0 or 1 or 2
    */
    cSimpleVector ref( int axis=All, double* value=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get actual angular velocity/s of axis/axes.

        - If axis is All then a NUMBER_OF_AXES-vector of the actual
          axis angular velocities is returned
        - If axis is a single number then the
          actual angular velocity of that axis is returned.

        Angular velocities are reported in degrees/second.

        \remark
          \a dummy ptr is never used, but needed nonetheless to make
          the signature of the function the same as for the other
          axis-access functions. This way a pointer to it can be used as
          a pointer to the other functions, which is needed by the
          generic cSDH::SetAxisValue and cSDH::GetAxisValue functions.
    */
    cSimpleVector vel( int axis=All, double* dummy=NULL )
        throw (cSDHLibraryException*);


    //-----------------------------------------------------------------
    /*!
        Get actual state/s of axis/axes.

        A state of 0 means "not moving" while 1 means "moving".

        - If axis is All then a NUMBER_OF_AXES-vector of the actual
          axis states is returned
        - If axis is a single number then the
          actual state of that axis is returned.
    */
    cSimpleVector state( int axis=All, double* dummy=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get actual temperatures of the axis motors

        Returns a list of the actual controller and driver temperature in degrees celsius.

    */
    cSimpleVector temp( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get actual temperatures of the electronics, i.e. teh FPGA and the PCB.
        (FPGA = Field Programmable Gate Array = the reprogrammable chip with the soft processors)
        (PCB = Printed Circuit Board)

        Returns a list of the actual controller and driver temperature in degrees celsius.

    */
    cSimpleVector temp_electronics( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return version of SDH firmware

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* ver( void )
        throw (cSDHLibraryException*);
    //-----------------------------------------------------------------

    /*!
        Return date of SDH firmware

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* ver_date( void )
        throw (cSDHLibraryException*);
    //-----------------------------------------------------------------

    /*!
        Return id of SDH

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* id( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return sn of SDH

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* sn( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return soc (System On Chip) ID of SDH

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* soc( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return date of soc (System On Chip) ID of SDH

        \attention
        The string returned is stored internally in this object and
        might be overwritten by the next command to this object
    */
    char* soc_date( void )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Return number of axis of SDH
    */
    int numaxis( void )
        throw (cSDHLibraryException*);


    //  end of doxygen name group sdhlibrary_cpp_csdhserial_diagnosis_commands
    //! @}
    //#################################################################

    //#################################################################
    /*!
      \anchor sdhlibrary_cpp_csdhserial_grip_commands
       \name   Grip methods
       @{
    */

    //-----------------------------------------------------------------
    /*!
        Get/Set motor current limits for grip commands

        - If axis is All and limit is None then a NUMBER_OF_AXES-list
          of the actually set motor current limits is returned
        - If axis is a single number and limit is None then the
          motor current limit for that axis is returned.
        - If axis and limit are single numbers then the motor current limit
          for that axis is set (and returned).
        - If axis is All and limit is a NUMBER_OF_AXES-vector then all axes
          motor current limits are set accordingly, the NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector igrip( int axis=All, double* limit=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Get/Set motor current limits for hold commands

        - If axis is All and limit is None then a NUMBER_OF_AXES-list
          of the actually set motor current limits is returned
        - If axis is a single number and limit is None then the
          motor current limit for that axis is returned.
        - If axis and limit are single numbers then the motor current limit
          for that axis is set (and returned).
        - If axis is All and limit is a NUMBER_OF_AXES-vector then all axes
          motor current limits are set accordingly, the NUMBER_OF_AXES-list is returned.
    */
    cSimpleVector ihold( int axis=All, double* limit=NULL )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        Send "selgrip grip" command to SDH. Where grip is in [0..eGID_DIMENSION-1]
        or one of the eGraspId enums.

        If sequ is True then wait until SDH hardware fully executed
        the command.  Else return immediately and do not wait until SDH
        hardware fully executed the command.

        This seems to work with sin square velocity profile only,
        so the velocity profile is switched to that if necessary.

        return the expected duration of the execution of the command in seconds
     */
    double selgrip( eGraspId grip, bool sequ )
        throw (cSDHLibraryException*);

    //-----------------------------------------------------------------
    /*!
        send "grip=close,velocity" command to SDH
        close    : [0.0 .. 1.0] where 0.0 is 'fully opened' and 1.0 is 'fully closed'
        velocity : ]0.0 .. 100.0] where 0.0 (not allowed) is very slow and 100.0 is very fast

        If sequ is True then wait until SDH hardware fully executed
        the command.  Else return immediately and do not wait until SDH
        hardware fully executed the command.

        This seems to work with sin square velocity profile only,
        so the velocity profile is switched to that if necessary.

        return the expected duration of the execution of the command in seconds
    */
    double grip( double close, double velocity, bool sequ )
        throw (cSDHLibraryException*);

    //  end of doxygen name group sdhlibrary_cpp_csdhserial_grip_commands
    //! @}
    //#################################################################

}; // end of class cSDHSerial
//=====================================================================

//! Type of a pointer to a "set-axis-values" function like cSDHSerial::p, cSDHSerial::pos, ..., cSDHSerial::igrip, cSDHSerial::ihold or cSDHSerial::ilim
typedef cSimpleVector (cSDHSerial::*pSetFunction)(int, double*);

//! Type of a pointer to a "get-axis-values" function like cSDHSerial::p, cSDHSerial::pos, ..., cSDHSerial::igrip, cSDHSerial::ihold or cSDHSerial::ilim
typedef cSimpleVector (cSDHSerial::*pGetFunction)(int, double*);

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

