//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_tcpserial_h_general General file information

  \author   Dirk Osswald
  \date     2010-10-30

  \brief
  Interface of class #SDH::cTCPSerial, class to access TCP port cygwin/linux.

  \section sdhlibrary_cpp_tcpserial_h_copyright Copyright

  Copyright (c) 2010 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_tcpserial_h_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
  \par SVN file revision:
  $Id: tcpserial.h 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_tcpserial_h_changelog Changelog of this file:
  \include tcpserial.h.log
*/
//======================================================================

#ifndef TCP_SERIAL_H_
#define TCP_SERIAL_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#if SDH_USE_VCC
# include <sys/timeb.h>
# include <time.h>
# include <winsock.h>
#else
# include <sys/time.h>
#endif
#include <string>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "serialbase.h"
#include "basisdef.h"

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member declarations
//----------------------------------------------------------------------


/*!
  \brief Derived exception class for low-level CAN ESD related exceptions.
*/
class VCC_EXPORT cTCPSerialException: public cSerialBaseException
{
public:
    cTCPSerialException( cMsg const & _msg )
        : cSerialBaseException( "cTCPSerialException", _msg )
    {}
};
//======================================================================


/*!
  \brief Low-level communication class to access a CAN port
*/
class VCC_EXPORT cTCPSerial : public cSerialBase
{

protected:

    std::string tcp_adr;

    //! the TCP port to use
    int tcp_port;

    //! the file descriptor of the socket
#if SDH_USE_VCC
    SOCKET fd;
#else
    int    fd;
    static const int INVALID_SOCKET = -1;
#endif
private:
    //! timeout for setsockopt
    struct  timeval timeout_timeval;
    //! cached timeout in us for read()
    long    timeout_us;

public:
    static double const TIMEOUT_WAIT_FOR_EVER_S;
    static double const TIMEOUT_RETURN_IMMEDITELY_S;
    static long const   TIMEOUT_WAIT_FOR_EVER_US;
    static long const   TIMEOUT_RETURN_IMMEDITELY_US;

    /*!
      Constructor: constructs an object to communicate with an %SDH via TCP on \a _tcp_adr and \a _tcp_port.
      \param _tcp_adr  - a string describing the hostname or IP address of the SDH
      \param _tcp_port - the port number on the SDH
      \param _timeout  - the timeout when receiving / sending data:
                         -  < 0.0 : no timeout = wait for ever
                         - == 0.0 : zero timeout = return immediately
                         -  > 0.0 : timeout in seconds
     */
    cTCPSerial( char const* _tcp_adr, int _tcp_port, double _timeout )
        throw (cTCPSerialException*);

    /*!
      Open the device as configured by the parameters given to the constructor
    */
    void Open( void )
        throw (cTCPSerialException*);

    //! Return true if interface to CAN ESD is open
    bool IsOpen( void )
        throw();

    //! Close the previously opened CAN ESD interface port.
    void Close( void )
        throw (cTCPSerialException*);

    //! Write data to a previously opened port.
    /*!
      Write \a len bytes from \a *ptr to the CAN device

      \param ptr - pointer the byte array to send in memory
      \param len - number of bytes to send

      \return the number of bytes actually written
    */
    int write( char const *ptr, int len=0 )
        throw (cTCPSerialException*);

    /*!
      Read data from device. This function waits until \a max_time_us us passed or
      the expected number of bytes are received via serial line.
      if (\a return_on_less_data is true (default value), the number of bytes
      that have been received are returned and the data is stored in \a data
      If the \a return_on_less_data is false, data is only read from serial line, if at least
      \a size bytes are available.
    */
    ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
        throw (cTCPSerialException*);

    //! set the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
    void SetTimeout( double _timeout )
        throw (cSerialBaseException*);

    /*!
     * Overloaded helper function that returns the last TCP error number.
     */
    virtual tErrorCode GetErrorNumber()
    {
#if SDH_USE_VCC
        return WSAGetLastError();
#else
        return errno;
#endif
    }
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
