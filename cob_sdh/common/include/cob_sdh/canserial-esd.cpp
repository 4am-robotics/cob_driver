//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_canserial_esd_cpp_general General file information

  \author   Dirk Osswald
  \date     2007-02-20

  \brief
  Implementation of class #SDH::cCANSerial_ESD, a class to access an ESD CAN interface  on cygwin/linux and Visual Studio.

  \section sdhlibrary_cpp_canserial_esd_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_canserial_esd_cpp_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2011-05-10 13:55:42 +0200 (Di, 10 Mai 2011) $
  \par SVN file revision:
  $Id: canserial-esd.cpp 6819 2011-05-10 11:55:42Z Osswald2 $

  \subsection sdhlibrary_cpp_canserial_esd_cpp_changelog Changelog of this file:
  \include canserial-esd.cpp.log
*/
//======================================================================

// on cygwin ntcan.h includes windef.h which defines macros named max/min which the compiler confuses with max/min templates
// but defining NOMINMAX prevents those evil macros from being defined
// The braindead ntcan.h defines typedefs like uin32_t and int32_t without defining
// the __uint32_t_defined or __nt8_t_defined macros needed by stdint.h on linux
// => we have to include ntcan.h first and then do some moc up as if stdint.h had
//    been included...
#define NOMINMAX
#include "ntcan.h"

#define __uint32_t_defined
#define __int8_t_defined

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
//#include <termios.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif
//#include <errno.h>
//#include <string.h>
//#include <sys/select.h>
//#include <sys/ioctl.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "canserial-esd.h"
#include "simpletime.h"
#include "util.h"


/*
   The Linux version of the ESD ntcan.h file differ a little from the
   Windows (and Cygwin) version.
   So define some macros for mapping/defining names to make the code compile
   on Linux systems too.
*/
#ifdef OSNAME_LINUX

// Linux ntcan.h maps its internal error number to std unix error numbers, so include their definitions:
# include <errno.h>


// Old Linux ntcan.h used HANDLE where Windows ntcan uses NTCAN_HANDLE
// so we had to make the following define:
//# define NTCAN_HANDLE HANDLE
// But newer ESD drivers seem to use NTCAN_HANDLE for both Linux and Windows
// and give deprecation warnings on the use of HANDLE

# ifndef NTCAN_BAUD_1000
   // Some Linux ntcan.h do not define any baudrate codes. The following ones were taken from
   // the Windows version in the hope that they are the same for Linux. Lets see...:
#  define NTCAN_BAUD_1000                 0
#  define NTCAN_BAUD_800                 14
#  define NTCAN_BAUD_500                  2
#  define NTCAN_BAUD_250                  4
#  define NTCAN_BAUD_125                  6
#  define NTCAN_BAUD_100                  7
#  define NTCAN_BAUD_50                   9
#  define NTCAN_BAUD_20                  11
#  define NTCAN_BAUD_10                  13
# endif
#endif

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START

//! internal hardware specific implementation details of the lowlevel ESD CAN interface
/*!
 * private data of cCANSerial_ESD.
 *
 * Required to keep inclusion of ntcan.h out of public interface of cCANSerial_ESD.
 */
class cCANSerial_ESD_Internal
{
public:
    //! the internal handle to the driver
    NTCAN_HANDLE ntcan_handle;   /*  remark: if you get a compiler error here
                                     (e.g. "error: NTCAN_HANDLE does not name a type")
                                     then please consider updating your ESD CAN
                                     driver (and hence ntcan.h). See also the
                                     comment on NTCAN_HANDLE on the beginning
                                     of this file.
     */

    int32_t timeout_ms;

    /*!
    * received messages might be split over several CAN messages
    * it might therefore happen that more data is received than
    * can be returned to the user. To not loose that data it is
    * kept here to be be returned in a later call
    */
    CMSG m_cmsg;
    //! index of next received data byte to return to user in m_cmsg
    int m_cmsg_next;

    NTCAN_RESULT rc;

};
NAMESPACE_SDH_END


USING_NAMESPACE_SDH

/*!
* Flag, if true then code for debug messages is included.
*
* The debug messages must still be enabled at run time by
* setting the \c some_cRS232_object.dbg.SetFlag(1).
*
* This 2 level scheme is used since this is the lowlevel communication,
* so debug outputs might really steal some performance.
*/
#define SDH_CANSERIAL_ESD_DEBUG 1

/*!
 * instead of guarding every debug output with \#if SDH_CANSERIAL_ESD_DEBUG / \#endif
 * we use this DBG macro that expands to a stream output to a dbg object or to ";" depending on the value of SDH_CANSERIAL_ESD_DEBUG
 */
#if SDH_CANSERIAL_ESD_DEBUG
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


cCANSerial_ESD::cCANSerial_ESD( int _net, unsigned long _baudrate, double _timeout, int _id_read, int _id_write )
throw (cCANSerial_ESDException*)
{
    pimpl = NULL;
    assert( sizeof( NTCAN_HANDLE ) == sizeof( tDeviceHandle) ); // if this fails, please adjust the tDeviceHandle type

    if ( _timeout < 0.0 )
        throw new cCANSerial_ESDException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

    pimpl = new cCANSerial_ESD_Internal();
    pimpl->ntcan_handle = NTCAN_HANDLE(NTCAN_INVALID_HANDLE);
    net = _net;
    baudrate = _baudrate;
    SetTimeout( _timeout );
    id_read = _id_read;
    id_write = _id_write;

    ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_ESD::cCANSerial_ESD( tDeviceHandle _ntcan_handle, double _timeout, int _id_read, int _id_write )
throw (cCANSerial_ESDException*)
{
    pimpl = NULL;
    assert( sizeof( NTCAN_HANDLE ) == sizeof( tDeviceHandle) ); // if this fails, please adjust the tDeviceHandle type

    if ( _timeout < 0.0 )
        throw new cCANSerial_ESDException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

    if ( _ntcan_handle == tDeviceHandle(NTCAN_HANDLE(NTCAN_INVALID_HANDLE)) )
        throw new cCANSerial_ESDException( cMsg( "Cannot reuse invalid ESD CAN handle" ) );

    pimpl = new cCANSerial_ESD_Internal();
    pimpl->ntcan_handle = NTCAN_HANDLE(_ntcan_handle);
    net = -1;
    baudrate = 0;
    SetTimeout( _timeout );
    id_read = _id_read;
    id_write = _id_write;

    ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_ESD::~cCANSerial_ESD()
{
    if ( pimpl )
        delete pimpl;
}
//----------------------------------------------------------------------

char const* ESD_strerror( NTCAN_RESULT rc )
{
    switch (rc)
    {
    DEFINE_TO_CASECOMMAND( NTCAN_SUCCESS );
    DEFINE_TO_CASECOMMAND( NTCAN_RX_TIMEOUT );
    DEFINE_TO_CASECOMMAND( NTCAN_TX_TIMEOUT );
    DEFINE_TO_CASECOMMAND( NTCAN_TX_ERROR );
    DEFINE_TO_CASECOMMAND( NTCAN_CONTR_OFF_BUS );
    DEFINE_TO_CASECOMMAND( NTCAN_CONTR_BUSY );
    DEFINE_TO_CASECOMMAND( NTCAN_CONTR_WARN );
    DEFINE_TO_CASECOMMAND( NTCAN_NO_ID_ENABLED );
    DEFINE_TO_CASECOMMAND( NTCAN_ID_ALREADY_ENABLED );
    DEFINE_TO_CASECOMMAND( NTCAN_ID_NOT_ENABLED );

    DEFINE_TO_CASECOMMAND( NTCAN_INVALID_FIRMWARE );
    DEFINE_TO_CASECOMMAND( NTCAN_MESSAGE_LOST );
    DEFINE_TO_CASECOMMAND( NTCAN_INVALID_HARDWARE );

    DEFINE_TO_CASECOMMAND( NTCAN_PENDING_WRITE );
    DEFINE_TO_CASECOMMAND( NTCAN_PENDING_READ );
    DEFINE_TO_CASECOMMAND( NTCAN_INVALID_DRIVER );

    DEFINE_TO_CASECOMMAND( NTCAN_SOCK_CONN_TIMEOUT );
    DEFINE_TO_CASECOMMAND( NTCAN_SOCK_CMD_TIMEOUT );
    DEFINE_TO_CASECOMMAND( NTCAN_SOCK_HOST_NOT_FOUND );

    DEFINE_TO_CASECOMMAND( NTCAN_INVALID_PARAMETER );
    DEFINE_TO_CASECOMMAND( NTCAN_INVALID_HANDLE );
#ifndef OSNAME_LINUX
// these errors are for Windows only ;-)
    DEFINE_TO_CASECOMMAND( NTCAN_IO_INCOMPLETE );
    DEFINE_TO_CASECOMMAND( NTCAN_IO_PENDING );
    DEFINE_TO_CASECOMMAND( NTCAN_HANDLE_FORCED_CLOSE );
    DEFINE_TO_CASECOMMAND( NTCAN_NOT_IMPLEMENTED );
    DEFINE_TO_CASECOMMAND( NTCAN_NOT_SUPPORTED );
#endif
    DEFINE_TO_CASECOMMAND( NTCAN_NET_NOT_FOUND );
    DEFINE_TO_CASECOMMAND( NTCAN_INSUFFICIENT_RESOURCES );

    DEFINE_TO_CASECOMMAND( NTCAN_OPERATION_ABORTED );
    DEFINE_TO_CASECOMMAND( NTCAN_WRONG_DEVICE_STATE );
    default:
        return "unknown";
    }
}
//-----------------------------------------------------------------


void cCANSerial_ESD::Open( void )
throw (cCANSerial_ESDException*)
{
    if ( pimpl->ntcan_handle == NTCAN_HANDLE(NTCAN_INVALID_HANDLE) )
    {
        // only open if we're not reusing an existing handle:

        //cerr << "opening can with timeout = " << timeout << ", in ms " << int32_t( timeout * 1000.0 ) << "\n";
        DBG( dbg << "Opening ESD CAN net: " << net << ", baudrate: " << baudrate << ", id_read: 0x" << std::hex << id_read << ", id_write: 0x" << id_write << std::dec << "\n" );
        pimpl->rc = canOpen( net,
                             0,           // flags
                             CAN_ESD_TXQUEUESIZE,         // txquesize
                             CAN_ESD_RXQUEUESIZE,         // rxquesize
                             pimpl->timeout_ms,
                             pimpl->timeout_ms,
                             &(pimpl->ntcan_handle) );

        if (pimpl->rc != NTCAN_SUCCESS)
        {
            // open failed, so ensure that pimpl->ntcan_handle is invalid
            pimpl->ntcan_handle = NTCAN_HANDLE(NTCAN_INVALID_HANDLE);
            throw new cCANSerial_ESDException( cMsg( "Could not open ESD CAN net %d: %s", net, GetLastErrorMessage() ) );
        }

        pimpl->rc = canSetBaudrate( pimpl->ntcan_handle, BaudrateToBaudrateCode( baudrate ) );
        if (pimpl->rc != NTCAN_SUCCESS)
            throw new cCANSerial_ESDException( cMsg( "Could not set baudrate to %lu on ESD CAN net %d: %s", baudrate, net, GetLastErrorMessage() ) );
    }

    pimpl->rc = canIdAdd( pimpl->ntcan_handle, id_read );
    if (pimpl->rc != NTCAN_SUCCESS)
        throw new cCANSerial_ESDException( cMsg( "Could not add CAN ID 0x%03x on ESD CAN net %d: %s", (unsigned int) id_read, net, GetLastErrorMessage() ) );

    // (re)init member data:
    pimpl->m_cmsg.len = 0;
    pimpl->m_cmsg.msg_lost = 0;
    pimpl->m_cmsg_next = 0;
}
//----------------------------------------------------------------------


bool cCANSerial_ESD::IsOpen( void )
throw()
{
    return ( pimpl->ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );
}
//----------------------------------------------------------------------


void cCANSerial_ESD::Close( void )
throw (cCANSerial_ESDException*)
{
    if ( pimpl->ntcan_handle == NTCAN_HANDLE(NTCAN_INVALID_HANDLE) )
        throw new cCANSerial_ESDException( cMsg( "Could not close un-opened device" ) );

    canClose( pimpl->ntcan_handle );
    pimpl->ntcan_handle = NTCAN_HANDLE(NTCAN_INVALID_HANDLE);
}
//----------------------------------------------------------------------

unsigned int cCANSerial_ESD::BaudrateToBaudrateCode( unsigned long baudrate )
throw (cCANSerial_ESDException*)
{
    switch (baudrate)
    {
    case 1000000: return NTCAN_BAUD_1000;
    case 800000: return NTCAN_BAUD_800;
    case 500000: return NTCAN_BAUD_500;
    case 250000: return NTCAN_BAUD_250;
    case 125000: return NTCAN_BAUD_125;
    case 100000: return NTCAN_BAUD_100;
    case 50000: return NTCAN_BAUD_50;
    case 20000: return NTCAN_BAUD_20;
    case 10000: return NTCAN_BAUD_10;
    }

    throw new cCANSerial_ESDException( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cCANSerial_ESD::write( char const *ptr, int len )
throw (cCANSerial_ESDException*)
{
    assert( pimpl->ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );

    //cerr << "in cCANSerial_ESD::write\n"; cerr.flush();
    if ( len == 0 )
        len = int( strlen( ptr ) );

    //cerr << "sending " << len << " bytes <" << ptr << "> to CAN net\n"; cerr.flush();

    // calculate number of CMSGS needed (max 8 data bytes per CMSG)
    int len_cmsgs = len/8 + (((len%8)!=0) ? 1 : 0);

    if ( len_cmsgs > CAN_ESD_TXQUEUESIZE )
        throw new cCANSerial_ESDException( cMsg( "len_cmsgs = %d > %d, please adjust CAN_ESD_TXQUEUESIZE!", (int) len_cmsgs, CAN_ESD_TXQUEUESIZE ) );

    //---------------------
    // prepare CMSGs to send:
#if SDH_USE_VCC
    // VCC cannot create variable size arrays on the stack; so use the heap
    CMSG* cmsg = new CMSG[ len_cmsgs ];
#else
    CMSG cmsg[ len_cmsgs ];
#endif
    for ( int i=0; i < len_cmsgs; i++)
    {
        cmsg[i].id = id_write;
        cmsg[i].len = min( 8, len-i*8 );
        for ( int j=0; j<cmsg[i].len; j++ )
            cmsg[i].data[ j ] = *(ptr++);

        DBG( dbg << "cCANSerial_ESD::write writing CAN frame id:0x" << std::hex << cmsg[i].id << " len=" << int(cmsg[i].len) << " DATA (hex):" << cHexByteString( (char const*) cmsg[i].data, cmsg[i].len ) << " bytes_written:" << (i*8+cmsg[i].len) << "/" << len << "\n" );

    }
    //---------------------

    //---------------------
    // now send the cmsgs and check return values (pimpl->rc and len_cmsgs)
    int len_cmsgs_save = len_cmsgs;
    pimpl->rc = canWrite( pimpl->ntcan_handle, cmsg, (int32_t*) &len_cmsgs, NULL );
#if SDH_USE_VCC
    // the code above cannot throw exceptions, so this delete[] will be reached in any case
    delete[] cmsg;
#endif
    if (pimpl->rc != NTCAN_SUCCESS)
        throw new cCANSerial_ESDException( cMsg( "Could not write %d CMSGs on ESD CAN net %d: %s", (int)len_cmsgs, net, GetLastErrorMessage() ) );

    if ( len_cmsgs != len_cmsgs_save )
        throw new cCANSerial_ESDException( cMsg( "Could only send %d/%d CMSGs on ESD CAN net %d", (int)len_cmsgs, (int)len_cmsgs_save, net ) );

    return len;
}
//----------------------------------------------------------------------


ssize_t cCANSerial_ESD::Read( void *_data, ssize_t size, long timeout_us, bool return_on_less_data )
throw (cCANSerial_ESDException*)
{
    assert( pimpl->ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) );

    char* data = (char*) _data;

    //---------------------
    // adjust rx timeout if necessary
    if ( long(pimpl->timeout_ms) * 1000L != timeout_us )
    {
        SetTimeout( double(timeout_us) / 1E6 );
    }
    //---------------------

    //---------------------
    int bytes_read = 0;

    do
    {
        // copy remaining, not yet returned bytes from a previous canRead call to data
        for ( ; pimpl->m_cmsg_next < pimpl->m_cmsg.len  &&  bytes_read < size; pimpl->m_cmsg_next++, bytes_read++ )
            *data++ = pimpl->m_cmsg.data[ pimpl->m_cmsg_next ];

        if ( bytes_read < size )
        {
            // if necessary read one more CMSGs with blocking call
            int len_cmsgs = 1;
            pimpl->m_cmsg.len = 0;
            pimpl->m_cmsg_next = 0;
            if ( timeout_us == 0 )
                pimpl->rc = canTake( pimpl->ntcan_handle, &(pimpl->m_cmsg), (int32_t*) &len_cmsgs );
            else
                pimpl->rc = canRead( pimpl->ntcan_handle, &(pimpl->m_cmsg), (int32_t*) &len_cmsgs, NULL );

            if (pimpl->rc != NTCAN_SUCCESS)
                throw new cCANSerial_ESDException( cMsg( "Could not read CAN messages from ESD CAN net %d: %s", net, GetLastErrorMessage() ) );

            DBG( dbg << "cCANSerial_ESD::Read read CAN frame id:0x" << std::hex << pimpl->m_cmsg.id << " len=" << int(pimpl->m_cmsg.len) << " data (hex):" << cHexByteString( (char const*) pimpl->m_cmsg.data, pimpl->m_cmsg.len ) << " bytes_read:" << bytes_read << "/" << size << "\n" );
            //DBG( dbg << "cCANSerial_ESD::Read read CAN frame id:0x" << std::hex << pimpl->m_cmsg.id << " len=" << int(pimpl->m_cmsg.len) << " data (hex):" << cHexByteString( (char const*) pimpl->m_cmsg.data, pimpl->m_cmsg.len ) << " bytes_read:" << bytes_read << "/" << size << " us_elapsed:" << start_time.Elapsed_us() << "\n );

            if ( len_cmsgs != 1  && timeout_us != 0 )
                throw new cCANSerial_ESDException( cMsg( "Could only read %d/%d CMSGs from ESD CAN net %d", int(len_cmsgs), 1, net ) );
            if ( len_cmsgs > 0 && pimpl->m_cmsg.id != id_read )
                throw new cCANSerial_ESDException( cMsg( "Invalid CAN ID 0x%03x received, expected 0x%03x", (unsigned int) pimpl->m_cmsg.id, (unsigned int) id_read ) );

            for ( ; pimpl->m_cmsg_next < pimpl->m_cmsg.len  &&  bytes_read < size; pimpl->m_cmsg_next++, bytes_read++ )
                *data++ = pimpl->m_cmsg.data[ pimpl->m_cmsg_next ];
        }
    }
    while ( bytes_read < size  &&  !return_on_less_data );

    return bytes_read;
}
//----------------------------------------------------------------------


void cCANSerial_ESD::SetTimeout( double _timeout )
throw (cSerialBaseException*)
{
    if ( _timeout < 0.0 )
        _timeout = 0.0;

    cSerialBase::SetTimeout( _timeout );
    pimpl->timeout_ms = int32_t(_timeout * 1000.0);

    if ( pimpl->ntcan_handle != NTCAN_HANDLE(NTCAN_INVALID_HANDLE) )
    {
        // we already have a handle, so we must forward the timeout to the driver also:
        //cerr << "adjusting timeout = " << _timeout << ", in ms " << pimpl->timeout_ms << "\n";

        pimpl->rc = canIoctl( pimpl->ntcan_handle, NTCAN_IOCTL_SET_RX_TIMEOUT, &(pimpl->timeout_ms) );

        if ( pimpl->rc != NTCAN_SUCCESS )
            throw new cCANSerial_ESDException( cMsg( "Could not set new rx timeout for ESD CAN net %d: %s", net, GetLastErrorMessage() ) );
    }
}
//----------------------------------------------------------------------

char const* cCANSerial_ESD::GetErrorMessage( tErrorCode dw )
{
    static char return_msg[512];

    snprintf( return_msg, 511, "error 0x%x = %d = \"%s\"", dw, dw, ESD_strerror( (NTCAN_RESULT) dw ) );
    return return_msg;

}
//----------------------------------------------------------------------

cSerialBase::tErrorCode cCANSerial_ESD::GetErrorNumber()
{
    return (tErrorCode) pimpl->rc;
}
//----------------------------------------------------------------------

tDeviceHandle cCANSerial_ESD::GetHandle()
{
    return tDeviceHandle( pimpl->ntcan_handle );
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
