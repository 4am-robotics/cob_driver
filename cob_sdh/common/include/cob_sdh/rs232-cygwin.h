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
      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
      \par SVN file revision:
        $Id: rs232-cygwin.h 3659 2008-10-08 08:48:38Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_cygwin_h_changelog Changelog of this file:
      \include rs232-cygwin.h.log
*/
//======================================================================

#ifndef RS232_CYGWIN_H_
#define RS232_CYGWIN_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

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
      Constructor: constructs an object to communicate with an SDH via RS232

      \param _port     - rs232 device number: 0='COM1'='/dev/ttyS0', 1='COM2'='/dev/ttyS1', ...
      \param _baudrate - the baudrate in bit/s
      \param _timeout  - the timeout in seconds
    */
  cRS232( int _port, unsigned long _baudrate, double _timeout );

  /*!
      Open the device as configured by the parameters given to the constructor
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
