//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_canserial_esd_h_general General file information

  \author   Dirk Osswald
  \date     2008-05-02

  \brief
  Interface of class #SDH::cCANSerial_ESD, class to access CAN bus via ESD card on cygwin/linux.

  \section sdhlibrary_cpp_canserial_esd_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_canserial_esd_h_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2009-12-01 11:41:18 +0100 (Di, 01 Dez 2009) $
  \par SVN file revision:
  $Id: canserial-esd.h 5000 2009-12-01 10:41:18Z Osswald2 $

  \subsection sdhlibrary_cpp_canserial_esd_h_changelog Changelog of this file:
  \include canserial-esd.h.log
*/
//======================================================================

#ifndef CANSERIAL_ESD_H_
#define CANSERIAL_ESD_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"
#include "serialbase.h"
#include "basisdef.h"

// on cygwin ntcan.h includes windef.h which defines macros named max/min which the compiler confuses with max/min templates
// but defining NOMINMAX prevents those evil macros from being defined
#define NOMINMAX
#include "ntcan.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START

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


//! transmit queue size for CAN frames
#define CAN_ESD_TXQUEUESIZE 32

//! receive queue size for CAN frames
#define CAN_ESD_RXQUEUESIZE 512

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member declarations
//----------------------------------------------------------------------





/*!
  \brief Derived exception class for low-level CAN ESD related exceptions.
*/
class cCANSerial_ESDException: public cSerialBaseException
{
public:
    cCANSerial_ESDException( cMsg const & _msg )
        : cSerialBaseException( "cCANSerial_ESDException", _msg )
    {}
};
//======================================================================


/*!
  \brief Low-level communication class to access a CAN port
*/
class cCANSerial_ESD : public cSerialBase
{

protected:

    //! the ESD CAN net to use
    int net;

    //! the baudrate to use in bit/s
    unsigned long baudrate;

    //! the CAN ID used for reading
    Int32 id_read;

    //! the CAN ID used for writing
    Int32 id_write;

    //! the handle to the driver
    NTCAN_HANDLE ntcan_handle;    /* remark: if you get a compiler error here
                                     (e.g. "error: NTCAN_HANDLE does not name a type")
                                     then please consider updating your ESD CAN
                                     driver (and hence ntcan.h). See also the
                                     comment on NTCAN_HANDLE on the beginning
                                     of this file.
                                  */

    //! Translate a baudrate given as unsigned long into a baudrate code for struct termios
    UInt32 BaudrateToBaudrateCode( unsigned long baudrate )
        throw (cCANSerial_ESDException*);

    int status;

private:
    UInt32 timeout_ms;

    /*!
    * received messages might be split over several CAN messages
    * it might therefore happen that more data is received than
    * can be returned to the user. To not loose that data it is
    * kept here to be be returned in a later call
    */
    CMSG m_cmsg;
    //! index of next received data byte to return to user in m_cmsg
    int m_cmsg_next;

public:
    /*!
      Constructor: constructs an object to communicate with an %SDH via CAN bus using an
      ESD CAN card.

      \param _net      - the ESD CAN net to use
      \param _baudrate - the baudrate in bit/s. Only some bitrates are valid: (1000000,800000,500000,250000,125000,100000,50000,20000,10000)
      \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
      \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID)
      \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID)
     */
    cCANSerial_ESD( int _net, unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write )
        throw (cCANSerial_ESDException*);

    /*!
      Constructor: constructs an object to communicate with an %SDH via CAN bus using an
      ESD CAN card by reusing an already existing handle.

      \param _ntcan_handle - the ESD CAN handle to reuse
      \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
      \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID)
      \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID)
     */
    cCANSerial_ESD( NTCAN_HANDLE _ntcan_handle, double _timeout, Int32 _id_read, Int32 _id_write )
        throw (cCANSerial_ESDException*);

    /*!
      Open the device as configured by the parameters given to the constructor
    */
    void Open( void )
        throw (cCANSerial_ESDException*);

    //! Return true if interface to CAN ESD is open
    bool IsOpen( void )
        throw();

    //! Close the previously opened CAN ESD interface port.
    void Close( void )
        throw (cCANSerial_ESDException*);

    //! Write data to a previously opened port.
    /*!
      Write \a len bytes from \a *ptr to the CAN device

      \param ptr - pointer the byte array to send in memory
      \param len - number of bytes to send

      \return the number of bytes actually written
    */
    int write( char const *ptr, int len=0 )
        throw (cCANSerial_ESDException*);

    /*!
      Read data from device. This function waits until \a max_time_us us passed or
      the expected number of bytes are received via serial line.
      if (\a return_on_less_data is true (default value), the number of bytes
      that have been received are returned and the data is stored in \a data
      If the \a return_on_less_data is false, data is only read from serial line, if at least
      \a size bytes are available.
    */
    ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
        throw (cCANSerial_ESDException*);

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
