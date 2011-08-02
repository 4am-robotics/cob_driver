//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_cygwin_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-20

  \brief
    Interface of class #SDH::cRS232, a class to access serial RS232 port on cygwin/linux.

  \section sdhlibrary_cpp_rs232_cygwin_h_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_cygwin_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-04-26 17:40:10 +0200 (Di, 26 Apr 2011) $
      \par SVN file revision:
        $Id: rs232-cygwin.h 6744 2011-04-26 15:40:10Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_cygwin_h_changelog Changelog of this file:
      \include rs232-cygwin.h.log
*/
//======================================================================

#ifndef RS232_CYGWIN_H_
#define RS232_CYGWIN_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <string>
#include <termios.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "serialbase.h"
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
   \brief Derived exception class for low-level RS232 related exceptions.
*/
class cRS232Exception: public cSerialBaseException
{
public:
  cRS232Exception( cMsg const & _msg )
    : cSerialBaseException( "cRS232Exception", _msg )
  {}
};
//======================================================================


/*!
   \brief Low-level communication class to access a serial port on Cygwin and Linux
*/
class cRS232 : public cSerialBase
{

protected:
    //! the RS232 portnumber to use
    int port;

    //! the sprintf format string to generate the device name from the port, see Constructor
    std::string device_format_string;

    //! the baudrate in bit/s
    unsigned long baudrate;

    //! the file descriptor of the RS232 port
    int fd;

    //! Translate a baudrate given as unsigned long into a baudrate code for struct termios
    tcflag_t BaudrateToBaudrateCode( unsigned long baudrate )
    throw (cRS232Exception*);

    int status;

    termios io_set_old;

public:
    /*!
      Constructor: constructs an object to communicate with an %SDH via RS232

      \param _port     - rs232 device number: 0='COM1'='/dev/ttyS0', 1='COM2'='/dev/ttyS1', ...
      \param _baudrate - the baudrate in bit/s
      \param _timeout  - the timeout in seconds
      \param _device_format_string - a format string (C string) for generating the device name, like "/dev/ttyS%d" (default) or "/dev/ttyUSB%d".
                                     Must contain a %d where the port number should be inserted.
                                     This char array is duplicated on construction
    */
  cRS232( int _port, unsigned long _baudrate, double _timeout, char const* _device_format_string = "/dev/ttyS%d" );

  /*!
   *  Open the device as configured by the parameters given to the constructor
   *
   * \bug
   *   The binary communication introduced in firmware 0.0.2.15 and
   *   SDHLibrary-C++ 0.0.2.0 did not work properly on Linux when RS232 was
   *   used for communication.
   *   <br>See also:
   *   - <a href="https://192.168.101.101/mechatronik/show_bug.cgi?id=1011)">Bug 1011: Bug: Binary communication does not work via RS232 on Linux</a>
   *   - #SDH_USE_BINARY_COMMUNICATION
   *   <br><b>=> Resolved in %SDHLibrary 0.0.2.2</b>
  */
  void Open( void )
      throw (cRS232Exception*);

  //! Return true if port to RS232 is open
  bool IsOpen( void )
      throw();

  //! Close the previously opened rs232 port.
  void Close( void )
      throw (cRS232Exception*);

  //! Write data to a previously opened port.
  /*!
      Write \a len bytes from \a *ptr to the rs232 device

      \param ptr - pointer the byte array to send in memory
      \param len - number of bytes to send

      \return the number of bytes actually written
  */
  int write( char const *ptr, int len=0 )
      throw (cRS232Exception*);

  /*!
    Read data from device. This function waits until \a max_time_us us passed or
    the expected number of bytes are received via serial line.
    if (\a return_on_less_data is true (default value), the number of bytes
    that have been received are returned and the data is stored in \a data
    If the \a return_on_less_data is false, data is only read from serial line, if at least
    \a size bytes are available.
   */
  ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
      throw (cRS232Exception*);

  //! overloaded from cSerialBase::UseCRC16 since we want to use a CRC16 to protect binary RS232 communication
  virtual bool UseCRC16()
  {
      return true;
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
