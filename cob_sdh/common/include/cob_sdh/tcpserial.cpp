//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_tcpserial_cpp_general General file information

  \author   Dirk Osswald
  \date     2010-10-30

  \brief
  Implementation of class #SDH::cTCPSerial, a class to access a TCP port on cygwin/linux and Visual Studio.

  \section sdhlibrary_cpp_tcpserial_cpp_copyright Copyright

  Copyright (c) 2010 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_tcpserial_cpp_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2011-04-26 17:40:10 +0200 (Di, 26 Apr 2011) $
  \par SVN file revision:
  $Id: tcpserial.cpp 6744 2011-04-26 15:40:10Z Osswald2 $

  \subsection sdhlibrary_cpp_tcpserial_cpp_changelog Changelog of this file:
  \include tcpserial.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <errno.h>
#include <string.h>
#include <sys/types.h>  // for setsockopt
#if SDH_USE_VCC
# include   <winsock.h>
# pragma comment (lib, "Ws2_32.lib")
# include <windows.h>
# include <strsafe.h>
#else
# include <sys/socket.h> // for setsockopt, inet_aton
# include <netdb.h>      // for gethostbyname
# include <arpa/inet.h>  // for htons, inet_aton
# include <netinet/in.h> // for inet_aton
# include <netinet/tcp.h> // for TCP_NODELAY
# include <unistd.h>
#endif
#include <fcntl.h>      // for fcntl

#include <iostream>
#include <exception>
#include <string>
//#include <stdarg.h>
#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "tcpserial.h"
#include "simpletime.h"
//#include "util.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

/*!
 * Flag, if true then code for debug messages is included.
 *
 * The debug messages must still be enabled at run time by
 * setting the \c some_cTCPSerial_object.dbg.SetFlag(1).
 *
 * This 2 level scheme is used since this is the lowlevel communication,
 * so debug outputs might really steal some performance.
 */
#define SDH_TCP_DEBUG 1


#if SDH_TCP_DEBUG
/*!
 * instead of guarding every debug output with \#if SDH_TCP_DEBUG / \#endif
 * we use this DBG macro that expands to a stream output to a dbg object or to ";" depending on the value of SDH_TCP_DEBUG
 */
# define DBG( ...  )        \
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

double const cTCPSerial::TIMEOUT_WAIT_FOR_EVER_S     = -1.0;
double const cTCPSerial::TIMEOUT_RETURN_IMMEDITELY_S =  0.0;
long const cTCPSerial::TIMEOUT_WAIT_FOR_EVER_US      = -1;
long const cTCPSerial::TIMEOUT_RETURN_IMMEDITELY_US  =  0;
//----------------------------------------------------------------------


cTCPSerial::cTCPSerial( char const* _tcp_adr, int _tcp_port, double _timeout )
    throw (cTCPSerialException*)
{
#if SDH_USE_VCC
    static WSADATA wsa;
    static bool    wsa_startup_called;

    if  ( !wsa_startup_called  &&  WSAStartup ( MAKEWORD ( 1, 1 ) , &wsa ) != 0 )
    {
        throw new cTCPSerialException( cMsg( "WSAStartup() failed: %s", GetLastErrorMessage() ) );
    }
    wsa_startup_called = true;
#endif

    tcp_adr = string( _tcp_adr );
    tcp_port = _tcp_port;
    fd = INVALID_SOCKET;
    SetTimeout( _timeout );
}
//----------------------------------------------------------------------


void cTCPSerial::Open( void )
    throw (cTCPSerialException*)
{
    struct hostent *host;
    struct sockaddr_in addr;
    // test whether the given hostname is an IP Address in dotted notation:
#if SDH_USE_VCC
    if  ( (addr.sin_addr.s_addr = inet_addr ( tcp_adr.c_str() )) == -1 )
#else
    if ( !inet_aton( tcp_adr.c_str(), &addr.sin_addr ) )
#endif
    {
        // no, its a hostname, so translate it to an IP address
        host = gethostbyname( tcp_adr.c_str() );
        if ( !host )
        {
            throw new cTCPSerialException( cMsg( "Invalid hostname \"%s\", gethostbyname() failed: %s", tcp_adr.c_str(), GetLastErrorMessage() ) );
        }
        addr.sin_addr = *(struct in_addr*) host->h_addr;
    }
    fd = socket( PF_INET, SOCK_STREAM, 0 );
    if ( fd == INVALID_SOCKET )
        throw new cTCPSerialException( cMsg( "Could not create TCP socket, socket() failed: %s", GetLastErrorMessage() ) );

    DBG( dbg << "Opening TCP connection to host: " << inet_ntoa( addr.sin_addr ) << ", port: " << tcp_port << "\n" );

    addr.sin_port = htons( tcp_port );
    addr.sin_family = AF_INET;

    int rc = connect( fd, (struct sockaddr*) &addr, sizeof(addr) );
    if ( rc == -1 )
        throw new cTCPSerialException( cMsg( "Could not connect to \"%s:%d\", connect() failed: %s", tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );

    //int zero = 0;
    int one  = 1;
#if SDH_USE_VCC
    rc = setsockopt( fd, SOL_SOCKET, TCP_NODELAY, (const char*) &one, sizeof( one ) );
#else
    rc = setsockopt( fd, SOL_SOCKET, TCP_NODELAY, &one, sizeof( one ) );
#endif
    if ( rc != 0 )
        throw new cTCPSerialException( cMsg( "Could not set option TCP_NODELAY for connection to \"%s:%d\", setsockopt failed: %s", tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );

    // set the timeout again after opening to set the O_NONBLOCK flag correctly
    SetTimeout( GetTimeout() );
}
//----------------------------------------------------------------------


bool cTCPSerial::IsOpen( void )
    throw()
{
    return ( fd != INVALID_SOCKET );
}
//----------------------------------------------------------------------


void cTCPSerial::Close( void )
    throw (cTCPSerialException*)
{
    if ( !IsOpen() )
        throw new cTCPSerialException( cMsg( "Could not close un-opened TCP socket" ) );

#if SDH_USE_VCC
    closesocket( fd );
#else
    close( fd );
#endif
    fd = INVALID_SOCKET;
}
//----------------------------------------------------------------------

int cTCPSerial::write( char const *ptr, int len )
    throw (cTCPSerialException*)
{
    assert( IsOpen() );

    if ( len == 0 )
        len = int( strlen( ptr ) );

    DBG( dbg << "cTCPSerial::write(): sending " << len << " bytes (hex): " << cHexByteString( ptr, len ) << "\n" );

    //---------------------
    int bytes_sent = send( fd, ptr, len, 0 );

    if ( bytes_sent < 0  &&  errno == EAGAIN  &&  timeout_us != TIMEOUT_WAIT_FOR_EVER_US ) // TODO: does this work in native windows?
        // expected timeout occurred
        return 0;
    if ( bytes_sent < 0  )
        throw new cTCPSerialException( cMsg( "Error from send to TCP \"%s:%d\": %s", tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );
    if ( bytes_sent != len )
        throw new cTCPSerialException( cMsg( "Could only send %d/%d bytes via TCP \"%s:%d\"", bytes_sent, len, tcp_adr.c_str(), tcp_port ) );
    //---------------------

    return bytes_sent;
}
//----------------------------------------------------------------------


ssize_t cTCPSerial::Read( void *_data, ssize_t size, long _timeout_us, bool return_on_less_data )
    throw (cTCPSerialException*)
{
    assert( IsOpen() );

    char* data = (char*) _data;

    //---------------------
    // adjust rx timeout if necessary
    if ( _timeout_us != timeout_us )
    {
        SetTimeout( double(_timeout_us) / 1E6 );
    }
    //---------------------

    //---------------------
    int bytes_received = 0;

    if ( _timeout_us > 0L )
    {
        // a timeout is set, so we have to use select() before recv() since timeouts do not work for recv() on cygwin (see also SetTimeout())

        //------------------------
        // Prepare the file descriptor set readfds for select():
        // - zero out
        // - set connected socket
        fd_set readfds;
#if SDH_USE_VCC
        SOCKET max_socket;
#else
        int    max_socket;
#endif
        FD_ZERO( &(readfds) );
        FD_SET( fd, &(readfds) );
        max_socket = fd+1;
        //------------------------

        //------------------------
        // Call select to see if new data is available on our sockets:
        int rc;
        rc = select( (int) max_socket, &readfds, NULL, NULL, &timeout_timeval );
        if( rc < 0 )
            throw new cTCPSerialException( cMsg( "Error from select() for TCP connection to \"%s:%d\": %s", tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );
        //------------------------

        //------------------------
        // Check if new data is available in fd:
        // If not then we have timeout, so break out .
        if ( !FD_ISSET( fd, &readfds ) )
            return bytes_received;
        //------------------------
    }

    //------------------------
    // do the actual receive:
    bytes_received = recv( fd, data, size, 0 );

    // and check for errors:
    if ( bytes_received < 0  &&  errno == EAGAIN  &&  timeout_us == TIMEOUT_RETURN_IMMEDITELY_US ) // TODO: does this work in native windows?
        // expected timeout occurred
        return 0;
    if ( bytes_received < 0 )
        throw new cTCPSerialException( cMsg( "Error from recv() for TCP connection to \"%s:%d\": %s", tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );

    DBG( dbg << "cTCPSerial::Read(): read " << bytes_received << "/" << size << " bytes (hex): " << cHexByteString( data, bytes_received ) << "\n" );

    if ( bytes_received < size && !return_on_less_data )
        throw new cTCPSerialException( cMsg( "Could only receive %d/%d bytes via TCP \"%s:%d\"", bytes_received, size, tcp_adr.c_str(), tcp_port ) );
    //---------------------

    return bytes_received;
}
//----------------------------------------------------------------------


void cTCPSerial::SetTimeout( double _timeout )
    throw (cSerialBaseException*)
{
    DBG( dbg << "cTCPSerial::SetTimeout(): " << _timeout << "\n" );

    if ( _timeout < 0.0 )
    {
        _timeout = TIMEOUT_WAIT_FOR_EVER_S;
        timeout_us = TIMEOUT_WAIT_FOR_EVER_US;
        timeout_timeval.tv_sec  = 0;
        timeout_timeval.tv_usec = 0;
    }
    else
    {
        timeout_timeval.tv_sec  = (tTimevalSec) _timeout;
        double v3 = (_timeout - ((double)timeout_timeval.tv_sec)) * 1.0E6;
        timeout_timeval.tv_usec = (tTimevalUSec) ( (_timeout - ((double)timeout_timeval.tv_sec)) * 1.0E6 );
        timeout_timeval.tv_usec = (tTimevalUSec) (v3);
        double v = (_timeout*1.0E6);
        timeout_us = (long)v;
    }
    cSerialBase::SetTimeout( _timeout );

    if ( IsOpen() )
    {
#if SDH_USE_VCC
        // see http://msdn.microsoft.com/en-us/library/ms738573%28VS.85%29.aspx
        u_long mode = (_timeout == 0.0);
        ioctlsocket( fd, FIONBIO, &mode );
#else
        int flags = fcntl( fd, F_GETFL );
        if ( _timeout == 0.0 )
            fcntl( fd, F_SETFL, flags | O_NONBLOCK );
        else
            fcntl( fd, F_SETFL, flags & ~O_NONBLOCK );
#endif
        /*
         * remark: the SO_RCVTIMEO dose NOT work on cygwin, see also http://cygwin.ru/ml/cygwin/2003-01/msg00833.html
         *
         * so independent of what timeout you set with SO_RCVTIMEO a call to
         * recv with no data to read will:
         * - return immediately if O_NONBLOCK has been set (with errno = 11 EAGAIN)
         * - return not at all if O_NONBLOCK has not been set (no matter what SO_RCVTIMEO has been set)
         *
         * => if timeout is > 0 then we have to use select() before recv() in Read()
         *
        int rc;
        rc = setsockopt( fd, SOL_SOCKET, SO_RCVTIMEO, &timeout_timeval, sizeof( timeout_timeval ) );
        if ( rc != 0 )
            throw new cTCPSerialException( cMsg( "Could not set option SO_RCVTIMEO for TCP connection to \"%s:%d\", setsockopt failed: %s",
                                                 tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );

        rc = setsockopt( fd, SOL_SOCKET, SO_SNDTIMEO, &timeout_timeval, sizeof( timeout_timeval ) );
        if ( rc != 0 )
            throw new cTCPSerialException( cMsg( "Could not set option SO_SNDTIMEO for TCP connection to \"%s:%d\", setsockopt failed: %s",
                                                 tcp_adr.c_str(), tcp_port, GetLastErrorMessage() ) );
         */
    }
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
