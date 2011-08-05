//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhexception_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-22

  \brief
    Interface of the exception base class #SDH::cSDHLibraryException and #SDH::cMsg.

  \section sdhlibrary_cpp_sdhexception_h_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhexception_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-06-17 10:55:45 +0200 (Mi, 17 Jun 2009) $
      \par SVN file revision:
        $Id: sdhexception.h 4605 2009-06-17 08:55:45Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhexception_h_changelog Changelog of this file:
      \include sdhexception.h.log
*/
//======================================================================

#ifndef SDHEXCEPTION_H_
#define SDHEXCEPTION_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <cstdio>     // needed in gcc-4.4 (as reported by Hannes Saal)

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class declaration
//----------------------------------------------------------------------

/*!
    \brief Class for short, fixed maximum length text messages

    Simple message objects for short, fixed maximum length text messages,
    but with printf like initialization.

    An object of type #SDH::cMsg contains an ASCII-Z string of maximum
    length #eMAX_MSG. It can be initialized with a 'printf-style'
    format string by the constructor. It is used in the SDHLibrary to
    further specify the cause of an exception in human readable form.

    See #SDH::cSDHLibraryException::cSDHLibraryException() for exemplary use.
*/
class cMsg
{
protected:

  //! anonymous enum instead of define macros
  enum {
    eMAX_MSG = 256 //!< maximum length in bytes of a message to store
  };

  char msg[ eMAX_MSG ];

public:
  //! Default constructor, init message to empty string.
  cMsg();


  //! Copy constructor, copy message content of other object to this object
  cMsg( cMsg const & other );


  //! Constructor with printf like format, argument parameters
  cMsg( char const* fmt, ... ) SDH__attribute__((format (printf, 2, 3)));

  /*
    Remark:
      Since non-static C++ methods have an implicit `this' argument,
      the arguments of such methods should be counted from two, not
      one, when giving values for STRING-INDEX and FIRST-TO-CHECK
      parameter of the format __attribute__.)
  */


  //! Return the C-string representation of the messag in this object
  char const *c_str() const;

};

std::ostream &operator<<(std::ostream &stream, cMsg const &msg);


//======================================================================

/*!
   \brief Base class for exceptions in the SDHLibrary-CPP

   At construction time a cMsg object is stored in the #msg member of
   the #cSDHLibraryException object. The cMsg object should contain a
   string wich further describes the actual cause of the exception
   thrown. The string in the cMsg object can be queried with the
   overloaded #what() member function, just like in the std::exception
   class.

   See the verbose description of the constructor
   #SDH::cSDHLibraryException::cSDHLibraryException() for examplary use.
*/
class cSDHLibraryException: public std::exception
{

protected:
  //! The message object
  cMsg msg;

public:
  /*!
      Constructor of sdh exception base class.

      \param _type - the type name of the exception. By convention this is the class name of the exception
      \param _msg  - a reference to a cMsg object that further describes the exception.

      \remark
      - The \a _type parameter is mainly usefull in derived classes
      - The \a _msg given as parameter is copied to the #msg member. Thus
        the given \a _msg object can be an anonymous object, like in:

      \code
      ...
      if ( v > v_max )
        throw new cSDHLibraryException( "cSDHLibraryException", cMsg( "Failed since v is invalid (v=%d > %d=v_max)", v, v_max ) );
      \endcode

      - But exceptions of the base will hardly ever be thrown. Instead
        objects of derived, more specific classes will be thrown. This
        looks like:
      \code
      // Derived exception class for more specific exceptions:
      class cDerivedException : public cSDHLibraryException
      {
      public:
        cDerivedException( cMsg const & _msg )
          : cSDHLibraryException( "cDerivedException", _msg )
        {}
      }
      // (Yes that is really all that must be done here!)

      ...

      try
      {
        ...
        if ( v > v_max )
          throw new cDerivedException( cMsg( "Failed since v is invalid (v=%d > %d=v_max)", v, v_max ) );
        ...
      }
      catch ( cDerivedException *e )
      {
        cerr << "Caught exception " << e->what() << "\n";
        // handle exception
        ...

        // finally delete the caught exception
        delete e;
      }

      \endcode

  */
  cSDHLibraryException( char const * _type, cMsg const & _msg );

  /*!
      Return the #msg member
   */
  virtual const char* what() const throw();

}; // cSDHLibraryException
//----------------------------------------------------------------------

/*!
   \brief Derived exception class for exceptions related to communication between the SDHLibrary and the %SDH.
*/
class cSDHErrorCommunication: public cSDHLibraryException
{
public:
  cSDHErrorCommunication( cMsg const & _msg )
    : cSDHLibraryException( "cSDHErrorCommunication", _msg )
  {}

  cSDHErrorCommunication( char const* _type, cMsg const & _msg )
      : cSDHLibraryException( _type, _msg )
  {}
};
//-----------------------------------------------------------------

std::ostream &operator<<(std::ostream &stream, cSDHLibraryException const &e);

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
