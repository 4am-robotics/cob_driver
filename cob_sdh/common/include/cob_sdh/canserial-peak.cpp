//======================================================================
/*!
 \file
 \section sdhlibrary_cpp_canserial_pcan_cpp_general General file information

 \author   Steffen Ruehl, Dirk Osswald
 \date     2009-07-29

 \brief
 Implementation of class #SDH::cCANSerial_PEAK, a class to access a PEAK CAN interface  on cygwin/linux and Visual Studio.
 <HR>
 \internal
*/
//======================================================================

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
#include <iostream>
#include <exception>
#include <stdarg.h>
#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "canserial-peak.h"
#include "simpletime.h"
#include "util.h"
#if defined( OSNAME_LINUX )
# include <libpcan.h>

#elif defined( OSNAME_CYGWIN )
// on cygwin Pcan_usb.h includes windef.h which defines macros named max/min which the compiler confuses with max/min templates
// but defining NOMINMAX prevents those evil macros from being defined
# define NOMINMAX
# include <windows.h>
# include <Pcan_usb.h>

#elif SDH_USE_VCC
# include <windows.h>
# include <Pcan_usb.h>

#else
# error "FIXME: support for PEAK CAN devices in other systems than linux/cygwin is not provided yet!"
        // e.g. the include header from peak is named libpcan.h on linux and peak_usb.h on windows
#endif

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

NAMESPACE_SDH_START
// the Linux version of the PEAK library uses handles,
// while the cygwin/windows version does not. So use a macro
// to keep the code somewhat clearer
#if defined( OSNAME_LINUX )
# define USE_HANDLES( H_ ) (H_),
# define USE_HANDLE( H_ ) (H_)
//! Linux libpcan uses HANDLE where Windows Pcan_usb.h uses no handle at all:
typedef HANDLE PCAN_HANDLE;

#else
# define USE_HANDLES( H_ )
# define USE_HANDLE( H_ )
//! Linux libpcan uses HANDLE where Windows Pcan_usb.h uses no handle at all:
typedef void* PCAN_HANDLE; // dummy definition
#endif
NAMESPACE_SDH_END

/*!
* Flag, if true then code for debug messages is included.
*
* The debug messages must still be enabled at run time by
* setting the \c some_cRS232_object.dbg.SetFlag(1).
*
* This 2 level scheme is used since this is the lowlevel communication,
* so debug outputs might really steal some performance.
*/
#define SDH_CANSERIAL_PEAK_DEBUG 1

/*!
 * instead of guarding every debug output with \#if SDH_CANSERIAL_PEAK_DEBUG / \#endif
 * we use this DBG macro that expands to a stream output to a dbg object or to ";" depending on the value of SDH_CANSERIAL_PEAK_DEBUG
 */
#if SDH_CANSERIAL_PEAK_DEBUG
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

NAMESPACE_SDH_START

//! internal hardware specific implementation details of the lowlevel PEAK CAN interface
/*!
 * private data of cCANSerial_PEAK.
 *
 * Required to keep inclusion of Pcan_usb.h out of public interface of cCANSerial_PEAK.
 */
class cCANSerial_PEAK_Internal
{
public:
    //! the internal handle to the driver
    PCAN_HANDLE peak_handle;

#if defined( OSNAME_LINUX )
    int timeout_us; //!< timeout in micro seconds
#else
    // The cygwin/windows version of the PEAK library/driver cannot handle timeouts...
#endif

    /*!
     * received messages might be split over several CAN messages
     * it might therefore happen that more data is received than
     * can be returned to the user. To not loose that data it is
     * kept here to be be returned in a later call
     */
#if defined( OSNAME_LINUX )
    TPCANRdMsg m_cmsg;
#   define M_CMSG_MSG() m_cmsg.Msg
#else
    TPCANMsg m_cmsg;
#   define M_CMSG_MSG() m_cmsg
#endif
    //! index of next received data byte to return to user in m_cmsg
    int m_cmsg_next;

    //! last return code of calls to Peak functions
    DWORD rc;
};
NAMESPACE_SDH_END


cCANSerial_PEAK::cCANSerial_PEAK( unsigned long _baudrate, double _timeout, int _id_read, int _id_write, const char *device )
   throw (cCANSerial_PEAKException*)
{
    pimpl = NULL;
    assert( sizeof( PCAN_HANDLE ) == sizeof( tDeviceHandle) ); // if this fails, please adjust the tDeviceHandle type

    if ( _timeout < 0.0 )
        throw new cCANSerial_PEAKException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

    pimpl = new cCANSerial_PEAK_Internal();
    pimpl->peak_handle = (PCAN_HANDLE) NULL;
    baudrate = _baudrate;
    SetTimeout( _timeout );
    id_read = _id_read;
    id_write = _id_write;
    strncpy(m_device, device, 64);

    ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_PEAK::cCANSerial_PEAK( PCAN_HANDLE _peak_handle, double _timeout, int _id_read, int _id_write )
   throw (cCANSerial_PEAKException*)
{
   pimpl = NULL;
   if ( _timeout < 0.0 )
       throw new cCANSerial_PEAKException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

#if defined( OSNAME_LINUX )
   if ( _peak_handle ==  NULL  )
       throw new cCANSerial_PEAKException( cMsg( "Cannot reuse invalid PEAK CAN handle" ) );
#endif

   pimpl = new cCANSerial_PEAK_Internal();
   pimpl->peak_handle = _peak_handle;
   baudrate = 0;
   SetTimeout( _timeout );
   id_read = _id_read;
   id_write = _id_write;

   ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_PEAK::~cCANSerial_PEAK()
{
    if ( pimpl )
        delete pimpl;
}
//----------------------------------------------------------------------

char const* PEAK_strerror( DWORD rc )
{
   switch (rc)
   {
       DEFINE_TO_CASECOMMAND( CAN_ERR_OK);
       DEFINE_TO_CASECOMMAND( CAN_ERR_XMTFULL       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_OVERRUN       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSLIGHT      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSHEAVY      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSOFF        );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QRCVEMPTY     );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QOVERRUN      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QXMTFULL      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_REGTEST       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_NOVXD         );
       DEFINE_TO_CASECOMMAND( CAN_ERR_RESOURCE      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_ILLPARAMTYPE  );
       DEFINE_TO_CASECOMMAND( CAN_ERR_ILLPARAMVAL   );
   default:
    return "unknown";
   }
}
//-----------------------------------------------------------------


void cCANSerial_PEAK::Open( void )
   throw (cCANSerial_PEAKException*)
{
    if (pimpl->peak_handle == NULL)
    {
        // only open if we're not reusing an existing handle:
        DBG( dbg << "Opening PEAK CAN baudrate: " << baudrate << ", id_read: 0x" << std::hex << id_read << ", id_write: 0x" << id_write << std::dec << "\n" );

#if defined( OSNAME_LINUX )
        pimpl->peak_handle = LINUX_CAN_Open((char*) m_device, O_RDWR );//| O_NONBLOCK);
        if (pimpl->peak_handle == NULL)
        {
            pimpl->rc = nGetLastError();
            // open failed, so ensure that handle is invalid
            pimpl->peak_handle = NULL;
            throw new cCANSerial_PEAKException( cMsg( "Could not open PEAK CAN device \"%s\": %s", m_device, GetLastErrorMessage() ) );
        }
#else
        pimpl->peak_handle = (PCAN_HANDLE) 1; // nothing more to do on cygwin/windows
#endif

        pimpl->rc = CAN_Init( USE_HANDLES( pimpl->peak_handle ) WORD(BaudrateToBaudrateCode(baudrate)), CAN_INIT_TYPE_ST);
        if ( pimpl->rc )
        {
#if ! defined( OSNAME_LINUX )
            pimpl->peak_handle = NULL;
#endif
            throw new cCANSerial_PEAKException( cMsg( "Could not set baudrate to %lu on Peak CAN device \"%s\": %s",
                                                      baudrate, m_device, GetLastErrorMessage() ) );
        }

        pimpl->rc = CAN_ResetFilter( USE_HANDLE(pimpl->peak_handle) );
        if ( pimpl->rc )
        {
#if ! defined( OSNAME_LINUX )
            pimpl->peak_handle = NULL;
#endif
            throw new cCANSerial_PEAKException( cMsg( "Could not reset CAN ID 0x%03x on Peak CAN device \"%s\": %s",
                                                      (unsigned int) id_read, m_device, GetLastErrorMessage() ) );
        }

        pimpl->rc = CAN_MsgFilter( USE_HANDLES(pimpl->peak_handle) DWORD(id_read), DWORD(id_read), MSGTYPE_STANDARD);
        if ( pimpl->rc )
        {
#if ! defined( OSNAME_LINUX )
            pimpl->peak_handle = NULL;
#endif
            throw new cCANSerial_PEAKException( cMsg( "Could not add CAN ID 0x%03x on Peak CAN device \"%s\": %s",
                                                      (unsigned int) id_read, m_device, GetLastErrorMessage() ) );
        }
    }
    // (re)init member data:
    pimpl->M_CMSG_MSG().LEN = 0;
    pimpl->m_cmsg_next = 0;
}
//----------------------------------------------------------------------


bool cCANSerial_PEAK::IsOpen( void )
   throw()
{
   return ( pimpl->peak_handle!=NULL );
}
//----------------------------------------------------------------------


void cCANSerial_PEAK::Close( void )
   throw (cCANSerial_PEAKException*)
{
   if ( pimpl->peak_handle == NULL )
       throw new cCANSerial_PEAKException( cMsg( "Could not close un-opened device" ) );

   CAN_Close( USE_HANDLE( pimpl->peak_handle ) );
   pimpl->peak_handle = NULL;
}
//----------------------------------------------------------------------

int cCANSerial_PEAK::BaudrateToBaudrateCode( unsigned long baudrate )
   throw (cCANSerial_PEAKException*)
{
   switch (baudrate)
   {
   case 1000000: return CAN_BAUD_1M;
   case 800000:  return CAN_BAUD_500K;
   case 500000:  return CAN_BAUD_500K;
   case 250000:  return CAN_BAUD_250K;
   case 125000:  return CAN_BAUD_125K;
   case 100000:  return CAN_BAUD_100K;
   case 50000:   return CAN_BAUD_50K;
   case 20000:   return CAN_BAUD_20K;
   case 10000:   return CAN_BAUD_10K;
   case 5000:    return CAN_BAUD_5K;
   }

   throw new cCANSerial_PEAKException( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cCANSerial_PEAK::write( char const *ptr, int len )
throw (cCANSerial_PEAKException*)
{
    assert( pimpl->peak_handle != NULL );

    //cerr << "in cCANSerial_PEAK::write\n"; cerr.flush();
    if ( len == 0 )
        len = int( strlen( ptr ) );

    //cerr << "sending " << len << " bytes <" << ptr << "> to CAN device\n"; cerr.flush();

    // calculate number of CMSGS needed (max 8 data bytes per CMSG)
    Int32 len_cmsgs = len/8 + (((len%8)!=0) ? 1 : 0);

    //---------------------
    TPCANMsg cmsg;
    int j;
    for ( int i=0; i < len_cmsgs; i++)
    {
        // prepare message to send:
        cmsg.ID = DWORD(id_write);
        cmsg.LEN = min( 8, len-i*8 );
        cmsg.MSGTYPE = MSGTYPE_STANDARD;
        for ( j=0; j<cmsg.LEN; j++ )
            cmsg.DATA[ j ] = *(ptr++);
        //-----

        //-----
        // now send the cmsgs and check return values
#if defined( OSNAME_LINUX )
        pimpl->rc = LINUX_CAN_Write_Timeout(pimpl->peak_handle, &cmsg, pimpl->timeout_us );
#else
        pimpl->rc = CAN_Write( &cmsg );
#endif
        if ( pimpl->rc )
        {
            throw new cCANSerial_PEAKException( cMsg( "Could not write message %d/%d on PEAK CAN device \"%s\": %s",
                                                      i, (int)len_cmsgs, m_device, GetLastErrorMessage() ) );
        }
        DBG( dbg << "cCANSerial_PEAK::write wrote CAN frame ID:0x" << std::hex << cmsg.ID << " LEN=" << int(cmsg.LEN) << " DATA (hex):" << cHexByteString( (char const*) cmsg.DATA, cmsg.LEN ) << " bytes_written:" << (i*8+cmsg.LEN) << "/" << len << "\n" );

    }
    return len;
}
//----------------------------------------------------------------------


ssize_t cCANSerial_PEAK::Read( void *_data, ssize_t size, long r_timeout_us, bool return_on_less_data )
throw (cCANSerial_PEAKException*)
{
    assert( pimpl->peak_handle != NULL );

    char* data = (char*) _data;

#if ! defined( OSNAME_LINUX )
    // in windows (cygwin and VCC) we must implement our own timeout mechanism
    cSimpleTime start_time;
#endif

    //---------------------
    int bytes_read = 0;

    do
    {
        //------
        // first: copy remaining, not yet returned bytes from a previous canRead call to data
        for ( ; pimpl->m_cmsg_next < pimpl->M_CMSG_MSG().LEN  &&  bytes_read < size; pimpl->m_cmsg_next++, bytes_read++ )
            *data++ = pimpl->M_CMSG_MSG().DATA[ pimpl->m_cmsg_next ];
        //------

        if ( bytes_read < size )
        {
            //-----
            // if necessary read one more messages with blocking call

            pimpl->M_CMSG_MSG().LEN = 0;
            pimpl->m_cmsg_next = 0;
#if defined( OSNAME_LINUX )
            //dbg << __func__ << " r_timeout_us=" << r_timeout_us << "\n";
            if ( r_timeout_us == 0 )
            {
                //pimpl->rc = LINUX_CAN_Read( pimpl->peak_handle, &(pimpl->m_cmsg) ); // will sometimes hang the process here!!!
                if ( return_on_less_data )
                {
                    //dbg << "polling read\n";
                    pimpl->rc = LINUX_CAN_Read_Timeout( pimpl->peak_handle, &(pimpl->m_cmsg),  0 ); // use polling
                }
                else
                {
                    //dbg << "blocking read\n";
                    pimpl->rc = LINUX_CAN_Read_Timeout( pimpl->peak_handle, &(pimpl->m_cmsg),  -1 ); // use blocking read
                }
            }
            else
            {
                //dbg << "timeout read" << r_timeout_us << "us\n";
                pimpl->rc = LINUX_CAN_Read_Timeout( pimpl->peak_handle, &(pimpl->m_cmsg),  r_timeout_us );
            }
#else
            pimpl->rc = CAN_Read( &(pimpl->m_cmsg) );
#endif
            //-----

            //-----
            // check the received message:

            // simple check: return code of the Read call:
            //dbg << "pimpl->rc=" << pimpl->rc << "\n";
            if (pimpl->rc==0xFFFFFFFF)
            {
                // ANOTE: although undocumented this seems to be the "no data available" answer when timeout=0

                //dbg << "***Ignoring error rc=-1  r_timeout_us=" << r_timeout_us <<" return_on_less_data=" << return_on_less_data << " errno=" << errno << "=\"" << strerror( errno ) << "\"\n";
                continue;
            }

            if (pimpl->rc > 0)
            {
                pimpl->M_CMSG_MSG().LEN = 0;

                //if ( (r_timeout_us == 0 || return_on_less_data) &&  pimpl->rc == CAN_ERR_QRCVEMPTY )
                if ( pimpl->rc == CAN_ERR_QRCVEMPTY )
                {
                    // no error, just no more data available
#if ! defined( OSNAME_LINUX )
                    if ( return_on_less_data && bytes_read > 0 )
                    {
                        //DBG( dbg << "cCANSerial_PEAK::Read1 took " << start_time.Elapsed_us() << "us\n" );
                        return bytes_read;
                    }

                    if ( r_timeout_us == 0  || start_time.Elapsed_us() < r_timeout_us)
                        continue;
#endif
                    //DBG( dbg << "cCANSerial_PEAK::Read2 took " << start_time.Elapsed_us() << "us\n" );
                    return bytes_read;
                }
                throw new cCANSerial_PEAKException( cMsg( "Could not read CAN messages from CAN Peak device \"%s\": %s",
                                                          m_device, GetLastErrorMessage() ) );
            }

            // check the actual type of the returned message:
            if ( pimpl->M_CMSG_MSG().MSGTYPE != MSGTYPE_STANDARD )
            {
                pimpl->M_CMSG_MSG().LEN = 0;
                if ( pimpl->M_CMSG_MSG().MSGTYPE == MSGTYPE_EXTENDED || pimpl->M_CMSG_MSG().MSGTYPE == MSGTYPE_RTR )
                {
                    cerr << "Ignoring invalid CAN message of type " << pimpl->M_CMSG_MSG().MSGTYPE << "\n"; cerr.flush();
                    continue;
                }
                // its a MSGTYPE_STATUS indicating an error
                // so the actual error code is in the data bytes in big endian
                pimpl->rc = (DWORD(pimpl->M_CMSG_MSG().DATA[0])<<24) | (DWORD(pimpl->M_CMSG_MSG().DATA[1])<<16) | (DWORD(pimpl->M_CMSG_MSG().DATA[2])<<8) | DWORD(pimpl->M_CMSG_MSG().DATA[3]);
                throw new cCANSerial_PEAKException( cMsg( "Error frame from CAN Peak device \"%s\": %s",
                                                          m_device, GetLastErrorMessage() ) );
            }

            // check the ID:
            if ( pimpl->M_CMSG_MSG().ID != DWORD(id_read) )
            {
                pimpl->M_CMSG_MSG().LEN = 0;
                throw new cCANSerial_PEAKException( cMsg( "Invalid CAN ID 0x%03x received, expected 0x%03x",
                                                          (unsigned int) pimpl->M_CMSG_MSG().ID, (unsigned int) id_read ) );
            }

            // finally copy received bytes to user data:
            for ( ; pimpl->m_cmsg_next < pimpl->M_CMSG_MSG().LEN  &&  bytes_read < size; pimpl->m_cmsg_next++, bytes_read++ )
                *data++ = pimpl->M_CMSG_MSG().DATA[ pimpl->m_cmsg_next ];

            DBG( dbg << "cCANSerial_PEAK::Read read CAN frame ID:0x" << std::hex << pimpl->M_CMSG_MSG().ID << " LEN=" << int(pimpl->M_CMSG_MSG().LEN) << " DATA (hex):" << cHexByteString( (char const*) pimpl->M_CMSG_MSG().DATA, pimpl->M_CMSG_MSG().LEN ) << " bytes_read:" << bytes_read << "/" << size << "\n" );
            //DBG( dbg << "cCANSerial_PEAK::Read read CAN frame ID:0x" << std::hex << pimpl->M_CMSG_MSG().ID << " LEN=" << int(pimpl->M_CMSG_MSG().LEN) << " DATA (hex):" << cHexByteString( (char const*) pimpl->M_CMSG_MSG().DATA, pimpl->M_CMSG_MSG().LEN ) << " bytes_read:" << bytes_read << "/" << size << " us_elapsed:" << start_time.Elapsed_us() << "\n" );
        }
    } while ( bytes_read < size  &&  !return_on_less_data );

    //DBG( dbg << "cCANSerial_PEAK::Read 3 took " << start_time.Elapsed_us() << "us\n" );
    return bytes_read;
}
//----------------------------------------------------------------------


void cCANSerial_PEAK::SetTimeout( double _timeout )
throw (cSerialBaseException*)
{
    cSerialBase::SetTimeout( _timeout );
#if defined( OSNAME_LINUX )
    pimpl->timeout_us = int( _timeout * 1E6 );
#endif
}
//----------------------------------------------------------------------


char const* cCANSerial_PEAK::GetErrorMessage( tErrorCode dw )
{
    static char return_msg[512];

    snprintf( return_msg, 511, "error 0x%x = %d = \"%s\"", (int) dw, (int) dw, PEAK_strerror( (DWORD) pimpl->rc ) );
    return return_msg;
}
//----------------------------------------------------------------------

cSerialBase::tErrorCode cCANSerial_PEAK::GetErrorNumber()
{
    return (tErrorCode) pimpl->rc;
}
//----------------------------------------------------------------------

tDeviceHandle cCANSerial_PEAK::GetHandle()
{
    return tDeviceHandle( pimpl->peak_handle );
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
