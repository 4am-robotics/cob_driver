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
  $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
  \par SVN file revision:
  $Id: canserial-esd.h 6526 2011-03-09 10:55:11Z Osswald2 $

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
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START

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

//! forward declaration of internal implementation specific data (Pimpl idiom)
class cCANSerial_ESD_Internal;


/*!
  \brief Derived exception class for low-level CAN ESD related exceptions.
*/
class VCC_EXPORT cCANSerial_ESDException: public cSerialBaseException
{
public:
    cCANSerial_ESDException( cMsg const & _msg )
        : cSerialBaseException( "cCANSerial_ESDException", _msg )
    {}
};
//======================================================================

/*!
  \brief Low-level communication class to access a CAN port from company ESD (http://www.esd.eu/)

  Since SDHLibrary-C++ release 0.0.2.0 implementation specific
  parts of the access to ESD CAN devices have been removed from
  the header file here in order to get rid of dependencies from
  the weird ntcan.h.
  Specifically the ntcan_handle of type NTCAN_HANDLE member was removed.
  You can still provide an existing handle for reuse on construction,
  but you must cast your NTCAN_HANDLE to a tDeviceHandle.
  You can get the internally used NTCAN_HANDLE cast to a tDeviceHandle
  with GetHandle().
*/
class VCC_EXPORT cCANSerial_ESD : public cSerialBase
{

protected:

    //! the ESD CAN net to use
    int net;

    //! the baudrate to use in bit/s
    unsigned long baudrate;

    //! the CAN ID used for reading
    int id_read;

    //! the CAN ID used for writing
    int id_write;

    // ntcan_handle was removed from here, see class comment and GetHandle()

    //! Translate a baudrate given as unsigned long into a baudrate code for struct termios
    unsigned int BaudrateToBaudrateCode( unsigned long baudrate )
        throw (cCANSerial_ESDException*);

    int status;

private:
    //! ptr to private, implementation specific members (using the 'Pimpl' (pointer to implementatino) design pattern)
    cCANSerial_ESD_Internal* pimpl;

    //! private copy constructor without implementation, since copying of cCANSerial_ESD objects makes no sense
    cCANSerial_ESD( cCANSerial_ESD const& other );

    //! private copy assignment operator without implementation, since copying of cCANSerial_ESD objects makes no sense
    cCANSerial_ESD& operator=( cCANSerial_ESD const& rhs );

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
    cCANSerial_ESD( int _net, unsigned long _baudrate, double _timeout, int _id_read, int _id_write )
        throw (cCANSerial_ESDException*);

    /*!
      Constructor: constructs an object to communicate with an %SDH via CAN bus using an
      ESD CAN card by reusing an already existing handle.

      \param _ntcan_handle - the ESD CAN handle to reuse (please cast your NTCAN_HANDLE to tDeviceHandle! It is save to do that!)
      \param _timeout  - the timeout in seconds (0 for no timeout = wait for ever)
      \param _id_read  - the CAN ID to use for reading (The %SDH sends data on this ID)
      \param _id_write - the CAN ID to use for writing (The %SDH receives data on this ID)
     */
    cCANSerial_ESD( tDeviceHandle _ntcan_handle, double _timeout, int _id_read, int _id_write )
        throw (cCANSerial_ESDException*);

    //! destructor: clean up
    ~cCANSerial_ESD();

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

    /*!
     * Overloaded helper function that returns the last ESD error number.
     */
    virtual tErrorCode GetErrorNumber();

    /*!
     * Overloaded helper function that returns an ESD error message for ESD error code dw.
     *
     * \remark The string returned will be overwritten by the next call to the function
     */
    virtual char const* GetErrorMessage( tErrorCode dw );

    //! return the internally used NTCAN_HANDLE cast to a tDeviceHandle
    tDeviceHandle GetHandle();
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
