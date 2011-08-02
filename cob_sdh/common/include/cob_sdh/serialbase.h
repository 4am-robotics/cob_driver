//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_serialbase_h_general General file information

  \author   Dirk Osswald
  \date     2008-05-02

  \brief
  Interface of class #SDH::cSerialBase, a virtal base class to access serial communication channels like RS232 or CAN

  \section sdhlibrary_cpp_serialbase_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_serialbase_h_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
  \par SVN file revision:
  $Id: serialbase.h 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_serialbase_h_changelog Changelog of this file:
  \include serialbase.h.log
*/
//======================================================================

#ifndef SERIALBASE_H_
#define SERIALBASE_H_

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4290)
#endif

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#if SDH_USE_VCC
# include <windows.h>
# include <strsafe.h>
#else
# include <errno.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "dbg.h"
#include "sdhbase.h" // for g_sdh_debug_log

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START

typedef void* tDeviceHandle;  //!< generic device handle for CAN devices

#if SDH_USE_VCC
# define snprintf _snprintf
#endif

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member declarations
//----------------------------------------------------------------------





/*!
  \brief Derived exception class for low-level serial communication related exceptions.
*/
class VCC_EXPORT cSerialBaseException: public cSDHErrorCommunication
{
 public:
    cSerialBaseException( cMsg const & _msg )
        : cSDHErrorCommunication( "cSerialBaseException", _msg )
    {}

    cSerialBaseException( char const* _type, cMsg const & _msg )
        : cSDHErrorCommunication( _type, _msg )
    {}
};
//======================================================================


/*!
  \brief Low-level communication class to access a serial port

  (This is an abstract base class with pure virtual functions)
*/
class VCC_EXPORT cSerialBase
{
 protected:

    //! an already read data byte of the next line
    char ungetch;

    //! Flag, true if ungetch is valid
    bool ungetch_valid;

    //! timeout in seconds
    double timeout;

 public:
    //! ctor
    cSerialBase()
    : // init members
        ungetch('\0'),
        ungetch_valid(false),
        // setting the timeout does not make sense here. should be left to virtual SetTimeout()
        dbg( false, "cyan", g_sdh_debug_log )
    {
        // nothing more to do
    }

    //! dtor
    virtual ~cSerialBase( void )
    {
        // do nothing
    }

    //! Open rs232 port \a port
    /*!
      Open the device with the parameters provided in the constructor
    */
    virtual void Open( void )
        throw (cSerialBaseException*) = 0;

    //! Return true if communication channel is open
    virtual bool IsOpen( void )
        throw() = 0;

    //! Close the previously opened communication channel.
    virtual void Close( void )
        throw (cSerialBaseException*) = 0;

    //! set the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
    virtual void SetTimeout( double _timeout )
        throw (cSerialBaseException*)
    {
        timeout = _timeout;
    }

    //! get the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
    virtual double GetTimeout()
    {
        return timeout;
    }

    //! helper class to set timeout of _serial_base on construction and reset to previous value on destruction. (RAII-idiom)
    class cSetTimeoutTemporarily
    {
        cSerialBase* serial_base;
        double       old_timeout;
    public:
        //! CTOR: remember current timeout of \a _serial_base and set its timeout to \a new_timeout, but only if current timeout and new_timeout differ
        cSetTimeoutTemporarily( cSerialBase* _serial_base, double new_timeout )
        : serial_base(_serial_base),
          old_timeout( serial_base->GetTimeout() )
        {
            if ( new_timeout != old_timeout )
                serial_base->SetTimeout( new_timeout );
        }

        //! DTOR: restore the remembered timeout
        ~cSetTimeoutTemporarily()
        {
            if ( old_timeout != serial_base->GetTimeout() )
                serial_base->SetTimeout( old_timeout );
        }
    };


    //! Write data to a previously opened port.
    /*!
      Write \a len bytes from \a *ptr to the rs232 device

      \param ptr - pointer the byte array to send in memory
      \param len - number of bytes to send

      \return the number of bytes actually written
    */
    virtual int write( char const *ptr, int len=0 )
        throw (cSerialBaseException*) = 0;

    /*!
      Read data from device. This function waits until \a max_time_us us passed or
      the expected number of bytes are received via serial line.
      if (\a return_on_less_data is true (default value), the number of bytes
      that have been received are returned and the data is stored in \a data
      If the \a return_on_less_data is false, data is only read from serial line, if at least
      \a size bytes are available.
    */
    virtual ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
        throw (cSerialBaseException*) = 0;

    //! Read a line from the device.
    /*!
      A line is terminated with one of the end-of-line (\a eol)
      characters ('\n' by default) or until timeout.
      Up to \a size-1 bytes are read and a '\0' char is appended.

      \param line - ptr to where to store the read line
      \param size - space available in line (bytes)
      \param eol  - a string containing all the chars that mark an end of line
      \param return_on_less_data - if (\a return_on_less_data is true (default value), the number of bytes
                                   that have been received are returned and the data is stored in \a data
                                   If the \a return_on_less_data is false, data is only read from serial line, if at least
                                   \a size bytes are available.

      A pointer to the line read is returned.

    */
    virtual char* readline( char* line, int size, char const* eol = "\n", bool return_on_less_data = false )
        throw (cSerialBaseException*);

    //! A stream object to print colored debug messages
    cDBG dbg;

    //! type of the error code, DWORD on windows and int on Linux/cygwin
#if SDH_USE_VCC
    typedef DWORD tErrorCode;
#else
    typedef int tErrorCode;
#endif

    /*!
     * Helper function that returns the last error number.
     * - On windows GetLastError() is used
     * - On cygwin/linux this uses errno
     */
    virtual tErrorCode GetErrorNumber()
    {
#if SDH_USE_VCC
        return GetLastError();
#else
        return errno;
#endif
    }

    /*!
     * Helper function that returns an error message for error code dw.
     * - On windows FormatMessageA() is used
     * - On cygwin/linux this uses errno
     *
     * \remark The string returned will be overwritten by the next call to the function
     */
    virtual char const* GetErrorMessage( tErrorCode dw );

    //! return the last error message as string. The string returned will be overwritten by the next call to the function
    char const* GetLastErrorMessage( void )
    {
        return GetErrorMessage( GetErrorNumber() );
    }

    /*!
     *  function that returns true if a CRC16 is used to protect binary communication.
     *
     *  The default is false since only RS232 communication needs this.
     */
    virtual bool UseCRC16()
    {
        return false;
    }
    //----------------------------------------------------------------------

};
//======================================================================

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
