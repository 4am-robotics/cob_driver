//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_cygwin_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-20

  \brief
    Implementation of class #SDH::cRS232, a class to access serial RS232 port on cygwin/linux.

  \section sdhlibrary_cpp_rs232_cygwin_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_cygwin_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-05-10 13:55:42 +0200 (Di, 10 Mai 2011) $
      \par SVN file revision:
        $Id: rs232-cygwin.cpp 6819 2011-05-10 11:55:42Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_cygwin_cpp_changelog Changelog of this file:
      \include rs232-cygwin.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif
#include <errno.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <iostream>
#include <exception>
#include <string>
#include <stdarg.h>
#include <cstring>    // needed in gcc-4.3 for prototypes like strcmp according to http://gcc.gnu.org/gcc-4.3/porting_to.html

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "rs232-cygwin.h"
#include "simpletime.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

/*!
* Flag, if true then code for debug messages is included.
*
* The debug messages must still be enabled at run time by
* setting the \c some_cRS232_object.dbg.SetFlag(1).
*
* This 2 level scheme is used since this is the lowlevel communication,
* so debug outputs might really steal some performance.
*/
#define SDH_RS232_CYGWIN_DEBUG 1

/*!
 * instead of guarding every debug output with \#if SDH_RS232_CYGWIN_DEBUG / \#endif
 * we use this DBG macro that expands to a stream output to a dbg object or to ";" depending on the value of SDH_RS232_CYGWIN_DEBUG
 */
#if SDH_RS232_CYGWIN_DEBUG
# define DBG( ... )         \
    do {                    \
        __VA_ARGS__;        \
    } while (0)
#else
# define DBG( ... )
#endif

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;
USING_NAMESPACE_SDH

//! helper function, duplicate string s into a char array allocated with new[]
char* StrDupNew( char* const s )
{
    char* dup = new char[strlen( s ) + 1];
    return strcpy( dup, s );
}
//----------------------------------------------------------------------

cRS232::cRS232( int _port, unsigned long _baudrate, double _timeout, char const* _device_format_string ) :
    // init base class and members
    cSerialBase(),
    port( _port ),
    device_format_string( _device_format_string ),
    baudrate( _baudrate ),
    fd( -1 ),
    status( 0 )
    // io_set_old cannot be initialized until we have an fd
{
    SetTimeout( _timeout );
}
//----------------------------------------------------------------------

void cRS232::Open( void )
throw (cRS232Exception*)
{
    char device[ device_format_string.size()+5 ]; // initializer just to get just enough space

    sprintf( device, device_format_string.c_str(), port );
    DBG( dbg << "Opening RS232 device '" << std::string(device) << "', baudrate: " << baudrate << "\n" );

    fd = open( device, O_RDWR | O_NOCTTY | O_NDELAY );

    if (fd <0)
        throw new cRS232Exception( cMsg( "Could not open device \"%s\": %s",
                                         device, GetLastErrorMessage() ) );

    termios io_set_new;
    // get device-settings
    if (tcgetattr( fd, &io_set_old ) < 0)
    {
        status = errno;
        throw new cRS232Exception( cMsg( "Could not get attributes of device \"%s\": %s",
                                         device, GetLastErrorMessage() ) );
    }
    else
        status = 0;

    // copy settings from old settings
    io_set_new=io_set_old;

    // declare new settings
    // (according to http://www.easysw.com/~mike/serial/serial.html
    //  the bits should be modified from current settings, not just set
    //  to the desired values)

    // The c_cflag member controls the baud rate, number of data bits,
    // parity, stop bits, and hardware flow control.
    io_set_new.c_cflag |=  CLOCAL;    // set Local line - do not change "owner" of port
    io_set_new.c_cflag |=  HUPCL;     // Hangup (drop DTR) on last close
    io_set_new.c_cflag |=  CREAD;     // enable reader
    io_set_new.c_cflag &= ~PARENB;    // set 8N1
    io_set_new.c_cflag &= ~CSTOPB;
    io_set_new.c_cflag &= ~CSIZE;
    io_set_new.c_cflag |=  CS8;
    io_set_new.c_cflag &= ~CRTSCTS;   // disable hardware flow control
    io_set_new.c_cflag &= ~CBAUD;     // clear baudrate bits first
    io_set_new.c_cflag |=  BaudrateToBaudrateCode( baudrate ); // set requested baudrate bits

    // The c_oflag member contains output filtering options.
    // Like the input modes, you can select processed or raw data output
    io_set_new.c_oflag &= ~OPOST;     // disable output processing

    // The input modes member c_iflag controls any input processing that is
    // done to characters received on the port.
    io_set_new.c_iflag &= ~INPCK;     // disable parity checking
    io_set_new.c_iflag |=  IGNPAR;    // ignore parity errors
    io_set_new.c_iflag &= ~ISTRIP;    // do NOT strip off parity bits
    //io_set_new.c_iflag |= ISTRIP;     // strip off parity bits (On Linux this will set the most significant bit to 0! On Cygwin this has no effect)
    io_set_new.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
    io_set_new.c_iflag |=  IGNBRK;    // do Ignore break condition
    io_set_new.c_iflag &= ~BRKINT;    // do not send a SIGINT when a break condition is detected
    io_set_new.c_iflag &= ~INLCR;     // do not Map NL to CR
    io_set_new.c_iflag &= ~IGNCR;     // do not Ignore CR
    io_set_new.c_iflag &= ~ICRNL;     // do not Map CR to NL
    io_set_new.c_iflag &= ~IUCLC;     // do not Map uppercase to lowercase
    io_set_new.c_iflag &= ~IMAXBEL;   // do not Echo BEL on input line too long

    // The local modes member c_lflag controls how input characters are managed
    // by the serial driver. In general you will configure the c_lflag member
    // for canonical or raw input.
    io_set_new.c_lflag &= ~ICANON;   // disable canonical input, enable raw input
    io_set_new.c_lflag &= ~ECHO;     // disable echoing of input characters
    io_set_new.c_lflag &= ~ECHOE;    // disable echo erase character as BS-SP-BS
    io_set_new.c_lflag &= ~ISIG;     // disable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals

    /*
     * source: http://www.easysw.com/~mike/serial/serial.html
     *
     * VMIN specifies the minimum number of characters to read. If it is set
     * to 0, then the VTIME value specifies the time to wait for every
     * character read. Note that this does not mean that a read call for N
     * bytes will wait for N characters to come in. Rather, the timeout will
     * apply to the first character and the read call will return the number
     * of characters immediately available (up to the number you request).
     *
     * If VMIN is non-zero, VTIME specifies the time to wait for the first
     * character read. If a character is read within the time given, any read
     * will block (wait) until all VMIN characters are read. That is, once the
     * first character is read, the serial interface driver expects to receive
     * an entire packet of characters (VMIN bytes total). If no character is
     * read within the time allowed, then the call to read returns 0. This
     * method allows you to tell the serial driver you need exactly N bytes
     * and any read call will return 0 or N bytes. However, the timeout only
     * applies to the first character read, so if for some reason the driver
     * misses one character inside the N byte packet then the read call could
     * block forever waiting for additional input characters.
     *
     * VTIME specifies the amount of time to wait for incoming characters in
     * tenths of seconds. If VTIME is set to 0 (the default), reads will
     * block (wait) indefinitely unless the NDELAY option is set on the port
     * with open or fcntl.
     */
    io_set_new.c_cc[VMIN] = 1;
    io_set_new.c_cc[VTIME]= 0;

    cfsetispeed(&io_set_new, BaudrateToBaudrateCode( baudrate ));
    cfsetospeed(&io_set_new, BaudrateToBaudrateCode( baudrate ));

    // set new settings
    if (tcsetattr(fd,TCSANOW,&io_set_new) < 0)
    {
        status=errno;
        throw new cRS232Exception( cMsg( "Could not set attributes of device \"%s\": %s",
                                         device, GetLastErrorMessage() ) );
    }
    else
        status=0;

}
//----------------------------------------------------------------------


bool cRS232::IsOpen( void )
throw()
{
    return (fd>=0);
}
//----------------------------------------------------------------------


void cRS232::Close( void )
throw (cRS232Exception*)
{
    if ( fd < 0 )
        throw new cRS232Exception( cMsg( "Could not close un-opened device" ) );

    close( fd );
    fd = -1;
}
//----------------------------------------------------------------------

tcflag_t cRS232::BaudrateToBaudrateCode( unsigned long baudrate )
throw (cRS232Exception*)
{
    switch (baudrate)
    {
#ifdef B3000000
    case 3000000: return B3000000;
#endif
#ifdef B2500000
    case 2500000: return B2500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B1152000
    case 1152000: return B1152000;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B576000
    case 576000: return B576000;
#endif
#ifdef B500000
    case 500000: return B500000;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B256000
    case 256000: return B256000;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
    case 115200: return B115200;
    case 57600:  return B57600;
    case 38400:  return B38400;
    case 19200:  return B19200;
    case 9600:   return B9600;
    case 4800:   return B4800;
    case 2400:   return B2400;
    case 1800:   return B1800;
    case 1200:   return B1200;
    case 600:    return B600;
    case 300:    return B300;
    case 200:    return B200;
    case 150:    return B150;
    case 134:    return B134;
    case 110:    return B110;
    case 75:     return B75;
    case 50:     return B50;
    }

    throw new cRS232Exception( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cRS232::write( char const *ptr, int len )
throw (cRS232Exception*)
{
    if ( len == 0 )
        len = strlen( ptr );

    int written = ::write( fd, ptr, len );

    DBG( dbg << "cRS232::write wrote " << len << "/" << written << " bytes (hex):" << cHexByteString( ptr, written ) << "\n" );

    return written;
}
//----------------------------------------------------------------------


ssize_t cRS232::Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
throw (cRS232Exception*)
{
    if (fd < 0)
        return status;

    fd_set      fds;
    int         bytes_read = 0;
    int         bytes_read_inc;
    int         select_return;
    char*       buffer =(char*) data;
    long        max_time_us = timeout_us;
    if ( max_time_us <= 0 )
        max_time_us = 1;
    cSimpleTime start_time;
    timeval     time_left;
    long        us_left;
    timeval*    timeout_p;

    status = 0;
    // We wait max max_time_us
    do
    {
        //---------------------
        // Look for received data with select()

        // calculate time left (min 1 us, otherwise we will read nothing at all)
        if ( max_time_us >= 0)
        {
            us_left = max_time_us - start_time.Elapsed_us();
            time_left.tv_sec  = us_left / 1000000;
            time_left.tv_usec = us_left % 1000000;

            if (time_left.tv_sec <= 0 && time_left.tv_usec < 1 )
            {
                time_left.tv_sec  = 0;
                time_left.tv_usec = 1;
            }

            timeout_p = &time_left;
        }
        else
            timeout_p = NULL; // wait indefinitely



        // prepare select call:
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        select_return = select( fd+1, &fds, NULL, NULL, timeout_p );

        ////cerr << "select_return = " << select_return <<"\n"; ////
        if ( select_return < 0 )
        {
            // error from select
            ////cerr << "Error calling select(): " <<  GetLastErrorMessage() << "\n";
            throw new cRS232Exception( cMsg( "Error calling select(): %s", GetLastErrorMessage() ) );
        }
        else if (select_return > 0)
        {
            // select says something is available for reading
            if (return_on_less_data)
            {
                // we can return on less, so just read what is available
                bytes_read_inc = read( fd, buffer + bytes_read, size - bytes_read );

                DBG( dbg << "cRS232::Read: Read " << bytes_read_inc << "/" << (size-bytes_read) << " bytes (hex): " << cHexByteString( buffer+bytes_read, bytes_read_inc ) << "\n" );

                if (bytes_read_inc < 0)
                {
                    // error from read
                    ////cerr << "Error calling read(): " <<  GetLastErrorMessage() << "\n";
                    throw new cRS232Exception( cMsg( "Error calling read(): %s", GetLastErrorMessage() ) );
                }
                // Any bytes read ?
                if (bytes_read_inc>0)
                {
                    bytes_read+=bytes_read_inc;
                    if (bytes_read==size)
                        return bytes_read;
                    //printf("data read:%i\n",bytes_read);
                }
            }
            else
            {
                // we can NOT return on less, so check if enough is available

                //printf("serial:time left %lu\n",Time2Long(tz));
                // Are there already enough bytes received ?

                // from sys/termios.h on cygwin:
                /* TIOCINQ is utilized instead of FIONREAD which has been
             accupied for other purposes under CYGWIN.
             Other UNIX ioctl requests has been omited because
             effects of their work one can achive by standard
             POSIX commands */

                errno = 0;
                int irc = ioctl( fd, TIOCINQ, &bytes_read_inc );
                if ( irc < 0)
                {
                    // error from ioctl
                    ////cerr << "Error calling ioctl(): " <<  GetLastErrorMessage() << "\n";
                    throw new cRS232Exception( cMsg( "Error calling ioctl(): %s", GetLastErrorMessage() ) );
                }
                else
                {
                    //if ( bytes_read_inc > 200 ) //size )
                    //    cerr << "ioctl ok, bytes_read_inc=" << bytes_read_inc << "\n";  // FIX ME: remove me
                    // Yes? then read data
                    if (bytes_read_inc>=size)
                    {
                        if ((bytes_read = read( fd, data, size )) < 0)
                        {
                            // error from read
                            ////cerr << "Error calling read() 2: " <<  GetLastErrorMessage() << "\n";
                            throw new cRS232Exception( cMsg( "Error calling read(): %s", GetLastErrorMessage() ) );
                        }
                        else
                        {
                            DBG( dbg << "cRS232::Read: Read " << bytes_read << "/" << size << " bytes (hex): " << cHexByteString( (char const*) data, bytes_read ) << "\n" );
                            return bytes_read;
                        }
                    }
                    // No ? do nothing
                }
            }
        }
        else
        {
            // (select_return == 0) select returned, but nothing is available

            if (return_on_less_data)
                return bytes_read;

            // else keep trying...
        }
    }
    // Look again for data, if any time left
    while ( timeout_us < 0 || start_time.Elapsed_us() < (long) max_time_us );

    return bytes_read;
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
