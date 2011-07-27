//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_vcc_cpp_general General file information
    \author   Martin
    \date     2008-05-23


  \brief
    Implementation of class #SDH::cRS232, a class to access serial RS232 port with VCC compiler on Windows.

  \section sdhlibrary_cpp_rs232_vcc_cpp_cpp_copyright Copyright
    Code kindly provided by Martin from the RoboCluster project Denmark.

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_vcc_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-08 13:36:01 +0100 (Di, 08 Mrz 2011) $
      \par SVN file revision:
        $Id: rs232-vcc.cpp 6521 2011-03-08 12:36:01Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_vcc_cpp_changelog Changelog of this file:
      \include rs232-vcc.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include "iostream"
#include <windows.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#define _CRT_SECURE_NO_WARNINGS 1

#include "rs232-vcc.h"
#include "simpletime.h"
#include "sdhlibrary_settings.h"
#include "util.h"
#include "dbg.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

/*!
 * Flag, if true then code for debug messages is included.
 *
 * The debug messages must still be enabled at run time by
 * setting the some_cRS232_object.dbg.SetFlag(1).
 *
 * This 2 level scheme is used since this is the lowlevel communication,
 * so debug outputs might really steal some performance.
 */
#define SDH_RS232_VCC_DEBUG 1

#if SDH_RS232_VCC_DEBUG
# define DBG( ... )         \
    do {                    \
        __VA_ARGS__;        \
    } while (0)
#else
# define DBG( ... )
#endif

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


cRS232::cRS232(  int _port, unsigned long _baudrate, double _timeout, char const* _device_format_string )
:   // init base class and members
    cSerialBase(),
#ifndef RS_232_TEST
    _hCOM(INVALID_HANDLE_VALUE),
#endif
    port(_port),
    baudrate(_baudrate),
    read_timeout_us(-1)
{
    SetTimeout( _timeout );
}

cRS232::~cRS232(void)
{
    if (_hCOM != INVALID_HANDLE_VALUE)
      Close();
}

#ifndef RS_232_TEST

void cRS232::Open( void )
    throw (cRS232Exception*)
{
    // see e.g. http://msdn.microsoft.com/de-de/magazine/cc301786(en-us).aspx
    //
    char device[] = "\\\\.\\COM00"; // initializer just to get just enough space
    sprintf(device, "\\\\.\\COM%d", port+1);
    DBG( dbg << "cRS232-vcc::Open: Opening RS232 device '" << device << "', baudrate: " << baudrate << "\n" );

    _hCOM = CreateFileA(device,                        // lpFileName
                        GENERIC_READ | GENERIC_WRITE,  // dwDesiredAccess
                        0,                             // dwShareMode
                        0,                             // lpSecurityAttributes
                        OPEN_EXISTING,                 // dwCreationDisposition
#if SDH_RS232_VCC_ASYNC
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, // dwFlagsAndAttributes
#else
                        FILE_ATTRIBUTE_NORMAL,         // dwFlagsAndAttributes
#endif
                        0);                            // hTemplateFile
    if ( _hCOM == INVALID_HANDLE_VALUE )
        throw new cRS232Exception( cMsg( "Could not create handle to RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );

#if SDH_RS232_VCC_ASYNC
    memset(&o, 0, sizeof(OVERLAPPED));
#endif
    BOOL rc;
    DCB dcbInitState;
    dcbInitState.DCBlength = sizeof( DCB );

    rc = GetCommState(_hCOM, &dcbInitState);
    if ( rc == 0 )
        throw new cRS232Exception( cMsg( "Could not get comm state of RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );
    dcbInitState.DCBlength = sizeof( DCB );
    dcbInitState.BaudRate = baudrate;
    dcbInitState.fBinary = 1;
    dcbInitState.fParity = 0;
    dcbInitState.fOutxCtsFlow = 0;
    dcbInitState.fOutxDsrFlow = 0;
    dcbInitState.fDtrControl = DTR_CONTROL_DISABLE;
    dcbInitState.fDsrSensitivity = 0;
    dcbInitState.fTXContinueOnXoff = 1;
    dcbInitState.fOutX = 0;
    dcbInitState.fInX = 0;
    dcbInitState.fErrorChar = 0;
    dcbInitState.fNull = 0;
    dcbInitState.fRtsControl = RTS_CONTROL_DISABLE;
    dcbInitState.fAbortOnError = 0;
    dcbInitState.ByteSize  = 8;
    dcbInitState.Parity    = NOPARITY;
    dcbInitState.StopBits = ONESTOPBIT;
    rc = SetCommState(_hCOM, &dcbInitState);
    if ( rc == 0 )
        throw new cRS232Exception( cMsg( "Could not set comm state of RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );
    SleepSec(0.060);

    rc = GetCommTimeouts( _hCOM, &comm_timeouts );
    if ( rc == 0 )
        throw new cRS232Exception( cMsg( "Could not get timeouts of RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );

    SetTimeout( timeout );
}

void cRS232::Close()
    throw (cRS232Exception*)
{
    CloseHandle(_hCOM);
    _hCOM = INVALID_HANDLE_VALUE;
}

void cRS232::SetTimeout( double _timeout )
    throw (cSerialBaseException*)
{
  DBG( dbg << "cRS232-vcc::SetTimeout( " << _timeout << ")\n" );
  if ( _hCOM != INVALID_HANDLE_VALUE )
  {
      // we have a valid handle, so set the new timeout
      // see http://msdn.microsoft.com/en-us/library/aa363194(VS.85).aspx

      if ( _timeout < 0.0 )
      {
          // negative timeout means wait for ever
          comm_timeouts.ReadIntervalTimeout        = 0;
          comm_timeouts.ReadTotalTimeoutMultiplier = 0;
          comm_timeouts.ReadTotalTimeoutConstant   = 0;
      }
      else if ( _timeout == 0.0 )
      {
          // we want 0 timeout (return immediately with all there is (even 0))
          comm_timeouts.ReadIntervalTimeout        = MAXDWORD;
          comm_timeouts.ReadTotalTimeoutMultiplier = 0;
          comm_timeouts.ReadTotalTimeoutConstant   = 0;
      }
      else
      {
          comm_timeouts.ReadIntervalTimeout        = 0;
          comm_timeouts.ReadTotalTimeoutMultiplier = 0;
          comm_timeouts.ReadTotalTimeoutConstant   = DWORD( _timeout * 1000.0 ); // new timeout in ms
      }

      BOOL rc = SetCommTimeouts( _hCOM, &comm_timeouts );
      if ( !rc )
          throw new cRS232Exception( cMsg( "Could not set timeouts of RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );
  }
  read_timeout_us = long( _timeout * 1000000.0 );
  cSerialBase::SetTimeout( _timeout );
}

ssize_t cRS232::Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
    throw (cRS232Exception*)
{
    char* buffer = (char*) data;
    cSerialBase::cSetTimeoutTemporarily set_timeout_temporarily( this, double( timeout_us ) / 1000000.0 );
    // previous timeout will be automatically restored when function is left (by return or exception)

    //memset(data, 0, size*sizeof(unsigned char));
    DWORD offset=0;
    DWORD bytes_read;
    BOOL rc;
    do
    {
#if SDH_RS232_VCC_ASYNC
        memset(&o, 0, sizeof(OVERLAPPED));
        o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        rc = ReadFile(_hCOM, buffer + offset, size-offset, &bytes_read, &o); // FIXed: reading all requested bytes in one call gave an extra 5 fps (78 vs 72 fps) in demo-benchmark
#else
        rc = ReadFile(_hCOM, buffer + offset, size-offset, &bytes_read, NULL ); //async , &o); // FIXed: FIXed: reading all requested bytes in one call gave an extra 5 fps (78 vs 72 fps) in demo-benchmark
#endif
        if ( rc == 0 )
        {
#if SDH_RS232_VCC_ASYNC
            DWORD last_error = GetLastError();
            if ( last_error == ERROR_IO_PENDING )
            {
                DWORD nRetVal;
                if(return_on_less_data)
                    nRetVal = WaitForSingleObject(o.hEvent, 10);
                else
                    nRetVal = WaitForSingleObject(o.hEvent, 5000);
                switch(nRetVal)
                {
                case WAIT_OBJECT_0:             // ReadFile event
                    break;
                case WAIT_TIMEOUT:
                    DBG( dbg << "cRS232-vcc::Read: WAIT_TIMEOUT bytes_read = " << bytes_read << "\n" );
                    CloseHandle(o.hEvent);
                    throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
                }
            }
            else
            {
                std::cerr << "error " << last_error << " from ReadFile / GetLastError: " << GetLastErrorMessage() << "\n";
            }
#else
            throw new cRS232Exception( cMsg( "ReadFile error Timeout while reading data from RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );
#endif
        }
        DBG( dbg << "cRS232-vcc::Read: Read " << bytes_read << "/" << size << " bytes (hex): " << cHexByteString( buffer + offset, bytes_read ) << "\n" );
        if ( bytes_read == 0 )
        {
             if ( timeout_us == 0 )
                 // no bytes read, but timeout is 0, so break
                 break;
             // no bytes read but a timeout > 0 is set, so this an unwanted timeout occurred
             DBG( dbg << "cRS232-vcc::Read timeout! bytes_read = 0\n" );
#if SDH_RS232_VCC_ASYNC
             CloseHandle(o.hEvent);
#endif
             throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
        }
#if SDH_RS232_VCC_ASYNC
        CloseHandle(o.hEvent);
#endif
        offset += bytes_read;
        //if(buffer[offset-1] != eol[0])
        //    SleepSec(0.001);
    } while( offset < (DWORD) size );

    return (ssize_t) offset;
}

char* cRS232::readline(char* line, int size, char* eol, bool return_on_less_data)
    throw (cRS232Exception*)
{
    memset(line, 0, size*sizeof(char));
    DWORD offset=0;
    DWORD bytes_read;
    BOOL rc;
    do
    {
#if SDH_RS232_VCC_ASYNC
        memset(&o, 0, sizeof(OVERLAPPED));
        o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        rc = ReadFile(_hCOM, &line[offset], 1, &bytes_read, &o);
#else
        rc = ReadFile(_hCOM, &line[offset], 1, &bytes_read, NULL);
#endif
        if ( rc == 0 )
        {
#if SDH_RS232_VCC_ASYNC
            DWORD last_error = GetLastError();
            if ( last_error == ERROR_IO_PENDING )
            {
                DWORD nRetVal;
                if(return_on_less_data)
                    nRetVal = WaitForSingleObject(o.hEvent, 10);
                else
                    nRetVal = WaitForSingleObject(o.hEvent, 5000);
                switch(nRetVal)
                {
                case WAIT_OBJECT_0:             // ReadFile event
                    break;
                case WAIT_TIMEOUT:
                    throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
                }
            }
            else
            {
                std::cerr << "error " << last_error << " from ReadFile / GetLastError: " << GetLastErrorMessage() << "\n";
            }
#else
            throw new cRS232Exception( cMsg( "ReadFile error while reading data from RS232 port %d = COM%d: %s", port, port+1, GetLastErrorMessage() ) );
#endif
        }
        if ( bytes_read == 0 )
            throw new cRS232Exception( cMsg( "ReadFile Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );

#if SDH_RS232_VCC_ASYNC
        CloseHandle(o.hEvent);
#endif
        offset += bytes_read;
        //if(line[offset-1] != eol[0])
        //    SleepSec(0.001);
    } while(line[offset-1] != eol[0]);

    return line;
}

int cRS232::write(char const *ptr, int len)
    throw (cRS232Exception*)
{
    if(len == 0)
        len = static_cast<int>(strlen(ptr));

    DWORD dwWritten;
    BOOL rc;
#if SDH_RS232_VCC_ASYNC
    //OVERLAPPED o = {0};
    o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    rc = WriteFile(_hCOM, (LPCVOID)ptr, len, &dwWritten, &o);
#else
    rc = WriteFile(_hCOM, (LPCVOID)ptr, len, &dwWritten, NULL );
#endif
    if ( rc == 0 )
        throw new cRS232Exception( cMsg( "Could not write %d bytes to RS232 port %d = COM%d: %s", len, port, port+1, GetLastErrorMessage() ) );

    DBG( dbg << "cRS232::write wrote " << len << "/" << dwWritten << " bytes (hex): " << cHexByteString( ptr, len ) << "\n" );

    //!!! dwWritten is always 0! Damn bloody windows
    //if(dwWritten != len)
    //    throw new cRS232Exception( cMsg( "Unable to write %d bytes to port %d = COM%d (only %d written)", len, port, port+1, dwWritten ) );

    //return dwWritten;
    return len;
}

#else

void cRS232::Open(int port, unsigned long baudrate, double timeout)
    throw (cRS232Exception*)
{
    _timeout = timeout;
}

void cRS232::Close()
    throw (cRS232Exception*)
{
}

char* cRS232::readline(char* line, int size, char* eol, bool return_on_less_data)
    throw (cRS232Exception*)
{
    std::cout << "EOL size=" << strlen(eol) << std::endl;
    if(return_on_less_data)
        throw new cRS232Exception("return_on_less_data");

    return "cRS232::readline";
}

int cRS232::write(char const *ptr, int len)
    throw (cRS232Exception*)
{
    if(len == 0)
        len = strlen( ptr );
    std::cout << ">>> " << ptr << std::endl;
    return len;
}

#endif
