//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_vcc_h_general General file information
    \author   Martin
    \date     2008-05-23


  \brief
    Implementation of class #SDH::cRS232, a class to access serial RS232 port with VCC compiler on Windows.

  \section sdhlibrary_cpp_rs232_vcc_h_cpp_copyright Copyright
    Code kindly provided by Martin from the RoboCluster project Denmark.

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_vcc_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-16 18:59:36 +0200 (Do, 16 Okt 2008) $
      \par SVN file revision:
        $Id: rs232-vcc.h 3722 2008-10-16 16:59:36Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_vcc_h_changelog Changelog of this file:
      \include rs232-vcc.h.log
*/
//======================================================================

#ifndef RS232_VCC_h
#define RS232_VCC_h

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4996)
# pragma warning(disable : 4290)
# pragma once
#endif

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <windows.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "serialbase.h"
#include "sdhlibrary_settings.h"


//#define RS_232_TEST
#define SDH_RS232_VCC_ASYNC 0

NAMESPACE_SDH_START


class cRS232Exception: public cSerialBaseException
{
public:
    cRS232Exception( cMsg const & _msg )
    : cSerialBaseException( "cRS232Exception", _msg )
    {}
};

/*!
\brief Low-level communication class to access a serial port on VCC Windows
*/
class cRS232 : public cSerialBase
{
private:
#ifndef RS_232_TEST
    HANDLE       _hCOM;
#endif
#if SDH_RS232_VCC_ASYNC
    OVERLAPPED   o;
#endif
    COMMTIMEOUTS comm_timeouts;
    long         read_timeout_us;

protected:
    //! the RS232 port number to use (port 0 is COM1)
    int port;

    //! the baudrate in bit/s
    unsigned long baudrate;


public:
    /*!
    * Constructor: constructs an object to communicate with an SDH via RS232

    * \param _port     - rs232 device number: 0='COM1'='/dev/ttyS0', 1='COM2'='/dev/ttyS1', ...
    * \param _baudrate - the baudrate in bit/s
    * \param _timeout  - the timeout in seconds
    */
    cRS232( int _port, unsigned long _baudrate, double _timeout );
    ~cRS232(void);

    void Open( void )
    throw (cRS232Exception*);
    void Close( void )
    throw (cRS232Exception*);

    virtual void SetTimeout( double _timeout )
        throw (cSerialBaseException*);

#ifndef RS_232_TEST
    bool IsOpen()
    throw()
    { return _hCOM != NULL; }
#else
    bool IsOpen() { return true; }
#endif
    int write( char const *ptr, int len=0 )
    throw (cRS232Exception*);

    /*!
    * Read data from device. This function waits until \a max_time_us us passed or
    * the expected number of bytes are received via serial line.
    * if (\a return_on_less_data is true (default value), the number of bytes
    * that have been received are returned and the data is stored in \a data
    * If the \a return_on_less_data is false, data is only read from serial line, if at least
    * \a size bytes are available.
    */
    ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
    throw (cRS232Exception*);

    char* readline(char* line, int size, char* eol, bool return_on_less_data)
    throw (cRS232Exception*);
};

NAMESPACE_SDH_END

#endif
