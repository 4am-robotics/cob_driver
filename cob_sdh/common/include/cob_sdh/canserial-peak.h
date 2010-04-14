//======================================================================
/*!
 \file
 \section sdhlibrary_cpp_canserial_peak_h_general General file information

 \author   Steffen Ruehl, Dirk Osswald
 \date     2009-07-29

 \brief
 Interface of class #SDH::cCANSerial_PEAK, class to access CAN bus via PEAK card on cygwin/linux.
*/
//======================================================================

#ifndef CANSERIAL_PEAK_H_
#define CANSERIAL_PEAK_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "serialbase.h"
#include "basisdef.h"
#include "sdhlibrary_settings.h"

#if defined( OSNAME_LINUX )
# include <libpcan.h>
// Linux libpcan uses HANDLE where Windows Pcan_usb.h uses no handle at all:
# define PCAN_HANDLE HANDLE

#elif defined( OSNAME_CYGWIN )
// on cygwin Pcan_usb.h includes windef.h which defines macros named max/min which the compiler confuses with max/min templates
// but defining NOMINMAX prevents those evil macros from being defined
# define NOMINMAX
# include <windows.h>
# include <Pcan_usb.h>
// Linux libpcan uses HANDLE where Windows Pcan_usb.h uses no handle at all:
typedef void* PCAN_HANDLE; // dummy definition

#elif SDH_USE_VCC
# include <windows.h>
# include <Pcan_usb.h>
// Linux libpcan uses HANDLE where Windows Pcan_usb.h uses no handle at all:
typedef void* PCAN_HANDLE; // dummy definition

#else
# error "FIXME: support for PEAK CAN devices in other systems than linux/cygwin is not provided yet!"
        // e.g. the include header from peak is named libpcan.h on linux and peak_usb.h on windows
#endif
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
 \brief Derived exception class for low-level CAN PEAK related exceptions.
*/
class cCANSerial_PEAKException: public cSerialBaseException
{
public:
   cCANSerial_PEAKException( cMsg const & _msg )
       : cSerialBaseException( "cCANSerial_PEAKException", _msg )
   {}
};
//======================================================================


/*!
 \brief Low-level communication class to access a CAN port
*/
class cCANSerial_PEAK : public cSerialBase
{

protected:

    //! the baudrate to use in bit/s
    unsigned long baudrate;

    //! the CAN ID used for reading
    DWORD id_read;

    //! the CAN ID used for writing
    DWORD id_write;

    //! the handle to the driver
    PCAN_HANDLE handle;

    //! Translate a baudrate given as unsigned long into a baudrate code for struct termios
    WORD BaudrateToBaudrateCode( unsigned long baudrate )
    throw (cCANSerial_PEAKException*);

    char m_device[64];

private:

#if defined( OSNAME_LINUX )
    int timeout_us; // timeout in micro seconds
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

public:
    /*!
    Constructor: constructs an object to communicate with an SDH via CAN bus using a
    PEAK CAN card.

    \param _baudrate - the baudrate in bit/s. Only some bitrates are valid: (1000000,800000,500000,250000,125000,100000,50000,20000,10000)
    \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
    \param _id_read  - the CAN ID to use for reading (The SDH sends data on this ID)
     \param _id_write - the CAN ID to use for writing (The SDH receives data on this ID)
     \param device    - the name of the char device to communicate with the PEAD driver (Needed on Linux only!)
    */
   cCANSerial_PEAK( unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write, const char *device="/dev/pcanusb0" )
       throw (cCANSerial_PEAKException*);

   /*!
     Constructor: constructs an object to communicate with an SDH via CAN bus using a
     PEAK CAN card by reusing an already existing handle (will work in Linux only).

     \param _handle   - the PEAK CAN handle to reuse (Works on Linux only!)
     \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
     \param _id_read  - the CAN ID to use for reading (The SDH sends data on this ID)
     \param _id_write - the CAN ID to use for writing (The SDH receives data on this ID)
    */
   cCANSerial_PEAK( PCAN_HANDLE _handle, double _timeout, Int32 _id_read, Int32 _id_write )
       throw (cCANSerial_PEAKException*);

   /*!
    * Return the value of the HANDLe to the actual CAN device.
    * Works on Linux only! Only returns a valid handle after a call to Open()!
    *
    * \remark The returned handle can be used to open a connection to a second SDH on the same CAN bus
    * @return the handle to the actual CAN device
    */
   PCAN_HANDLE GetHandle()
   {
       return handle;
   }

   /*!
     Open the device as configured by the parameters given to the constructor
   */
   void Open( void )
       throw (cCANSerial_PEAKException*);

   //! Return true if interface to CAN PEAK is open
   bool IsOpen( void )
       throw();

   //! Close the previously opened CAN PEAK interface port.
   void Close( void )
       throw (cCANSerial_PEAKException*);

   //! Write data to a previously opened port.
   /*!
     Write \a len bytes from \a *ptr to the CAN device

     \param ptr - pointer the byte array to send in memory
     \param len - number of bytes to send

     \return the number of bytes actually written
   */
   int write( char const *ptr, int len=0 )
       throw (cCANSerial_PEAKException*);

   /*!
     Read data from device. This function waits until \a max_time_us us passed or
     the expected number of bytes are received via serial line.
     if (\a return_on_less_data is true (default value), the number of bytes
     that have been received are returned and the data is stored in \a data
     If the \a return_on_less_data is false, data is only read from serial line, if at least
     \a size bytes are available.
   */
   ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
       throw (cCANSerial_PEAKException*);

   //! set the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
   void SetTimeout( double _timeout )
       throw (cSerialBaseException*);
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
