//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_serialbase_h_general General file information

  \author   Dirk Osswald
  \date     2008-05-02

  \brief
  Interface of class #SDH::cSerialBase, a virtal base class to access serial communication channels like RS232 or CAN

  \section sdhlibrary_cpp_serialbase_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_serialbase_h_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
  \par SVN file revision:
  $Id: serialbase.h 3686 2008-10-13 15:07:24Z Osswald2 $

  \subsection sdhlibrary_cpp_serialbase_h_changelog Changelog of this file:
  \include serialbase.h.log
*/
//======================================================================

#ifndef SERIALBASE_H_
#define SERIALBASE_H_

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4290)
#endif

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"

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
  \brief Derived exception class for low-level serial communication related exceptions.
*/
class cSerialBaseException: public cSDHErrorCommunication
{
 public:
    cSerialBaseException( cMsg const & _msg )
        : cSDHErrorCommunication( "cSerialBaseException", _msg )
    {}

    cSerialBaseException( char const* _type, cMsg const & _msg )
        : cSDHErrorCommunication( _type, _msg )
    {}
};
//======================================================================


/*!
  \brief Low-level communication class to access a serial port

  (This is an abstract base class with pure virtual functions)
*/
class cSerialBase
{
 protected:

    //! an already read data byte of the next line
    char ungetch;

    //! Flag, true if ungetch is valid
    bool ungetch_valid;

    //! timeout in seconds
    double timeout;

 public:
    virtual ~cSerialBase( void )
    {
        // do nothing
    }

    //! Open rs232 port \a port
    /*!
      Open the device with the parameters provided in the constructor
    */
    virtual void Open( void )
        throw (cSerialBaseException*) = 0;

    //! Return true if communication channel is open
    virtual bool IsOpen( void )
        throw() = 0;

    //! Close the previously opened communication channel.
    virtual void Close( void )
        throw (cSerialBaseException*) = 0;

    //! set the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
    virtual void SetTimeout( double _timeout )
        throw (cSerialBaseException*)
    {
        timeout = _timeout;
    }

    //! set the timeout for next #readline() calls (negative value means: no timeout, wait for ever)
    virtual double GetTimeout()
    {
        return timeout;
    }

    //! Write data to a previously opened port.
    /*!
      Write \a len bytes from \a *ptr to the rs232 device

      \param ptr - pointer the byte array to send in memory
      \param len - number of bytes to send

      \return the number of bytes actually written
    */
    virtual int write( char const *ptr, int len=0 )
        throw (cSerialBaseException*) = 0;

    /*!
      Read data from device. This function waits until \a max_time_us us passed or
      the expected number of bytes are received via serial line.
      if (\a return_on_less_data is true (default value), the number of bytes
      that have been received are returned and the data is stored in \a data
      If the \a return_on_less_data is false, data is only read from serial line, if at least
      \a size bytes are available.
    */
    virtual ssize_t Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
        throw (cSerialBaseException*) = 0;

    //! Read a line from the device.
    /*!
      A line is terminated with one of the end-of-line (eol)
      characters ('\n' by default) or until timeout

      \param line - ptr to where to store the read line
      \param size - space available in line (bytes)
      \param eol  - a string containing all the chars that mark an end of line
      \param return_on_less_data - if (\a return_on_less_data is true (default value), the number of bytes
                                   that have been received are returned and the data is stored in \a data
                                   If the \a return_on_less_data is false, data is only read from serial line, if at least
                                   \a size bytes are available.

      A pointer to the line read is returned.

    */
    virtual char* readline( char* line, int size, char* eol = "\n", bool return_on_less_data = false )
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

