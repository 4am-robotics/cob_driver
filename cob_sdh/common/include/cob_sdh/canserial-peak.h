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


//! forward declaration of internal implementation specific data (Pimpl idiom)
class cCANSerial_PEAK_Internal;



/*!
 \brief Derived exception class for low-level CAN PEAK related exceptions.
*/
class VCC_EXPORT cCANSerial_PEAKException: public cSerialBaseException
{
public:
   cCANSerial_PEAKException( cMsg const & _msg )
       : cSerialBaseException( "cCANSerial_PEAKException", _msg )
   {}
};
//======================================================================


/*!
 \brief Low-level communication class to access a CAN port from company PEAK (http://www.peak-system.com)

  Since SDHLibrary-C++ release 0.0.2.0 implementation specific
  parts of the access to PEAK CAN devices have been removed from
  the header file here in order to get rid of dependencies from
  the Pcan_usb.h.
  Specifically the peak_handle of type PEAK_HANDLE member was removed.
  You can still provide an existing handle for reuse on construction,
  but you must cast your HANDLE to a tDeviceHandle.
  You can get the internally used PEAK_HANDLE cast to a tDeviceHandle
  with GetHandle().
*/
class VCC_EXPORT cCANSerial_PEAK : public cSerialBase
{

protected:

    //! the baudrate to use in bit/s
    unsigned long baudrate;

    //! the CAN ID used for reading
    int id_read;

    //! the CAN ID used for writing
    int id_write;

    // handle was removed from here, see class comment and GetHandle()

    //! Translate a baudrate given as unsigned long into a baudrate code for struct termios
    int BaudrateToBaudrateCode( unsigned long baudrate )
    throw (cCANSerial_PEAKException*);

    char m_device[64];

private:
    //! ptr to private, implementation specific members (using the 'Pimpl' (pointer to implementatino) design pattern)
    cCANSerial_PEAK_Internal* pimpl;

    //! private copy constructor without implementation, since copying of cCANSerial_PEAK objects makes no sense
    cCANSerial_PEAK( cCANSerial_PEAK const& other );

    //! private copy assignment operator without implementation, since copying of cCANSerial_PEAK objects makes no sense
    cCANSerial_PEAK& operator=( cCANSerial_PEAK const& rhs );

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
   cCANSerial_PEAK( unsigned long _baudrate, double _timeout, int _id_read, int _id_write, const char *device="/dev/pcanusb0" )
       throw (cCANSerial_PEAKException*);

   /*!
     Constructor: constructs an object to communicate with an SDH via CAN bus using a
     PEAK CAN card by reusing an already existing handle (will work in Linux only).

     \param _peak_handle   - the PEAK CAN handle to reuse (Works on Linux only!)
     \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
     \param _id_read  - the CAN ID to use for reading (The SDH sends data on this ID)
     \param _id_write - the CAN ID to use for writing (The SDH receives data on this ID)
    */
   cCANSerial_PEAK( tDeviceHandle _peak_handle, double _timeout, int _id_read, int _id_write )
       throw (cCANSerial_PEAKException*);

   //! destructor: clean up
   ~cCANSerial_PEAK();

   /*!
    * Return the value of the HANDLE to the actual CAN device.
    * Works on Linux only! Only returns a valid handle after a call to Open()!
    *
    * \remark The returned handle can be used to open a connection to a second SDH on the same CAN bus
    * @return the handle to the actual CAN device
    */
   tDeviceHandle GetHandle();

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

   /*!
    * Overloaded helper function that returns the last Peak error number.
    */
   virtual tErrorCode GetErrorNumber();

   /*!
    * Overloaded helper function that returns a PEAK error message for error code dw.
    *
    * \remark The string returned will be overwritten by the next call to the function
    */
   virtual char const* GetErrorMessage( tErrorCode dw );

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
