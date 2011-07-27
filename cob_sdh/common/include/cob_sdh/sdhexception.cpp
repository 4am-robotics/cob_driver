//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhexception_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-22

  \brief
    Implementation of the exception base class #SDH::cSDHLibraryException and #SDH::cMsg.

  \section sdhlibrary_cpp_sdhexception_h_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhexception_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-03-18 09:39:27 +0100 (Mi, 18 Mrz 2009) $
      \par SVN file revision:
        $Id: sdhexception.cpp 4191 2009-03-18 08:39:27Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhexception_h_changelog Changelog of this file:
      \include sdhexception.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"
#include <cstring>    // needed in gcc-4.3 for prototypes like strcmp according to http://gcc.gnu.org/gcc-4.3/porting_to.html

#if SDH_USE_VCC
# pragma warning(disable : 4996)
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

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


cMsg::cMsg()
{
  msg[0] = '\0';
}
//----------------------------------------------------------------------


cMsg::cMsg( cMsg const & other )
{
  strncpy( msg, other.msg, eMAX_MSG );
}
//----------------------------------------------------------------------


cMsg::cMsg( char const* fmt, ... )
{
  va_list arglist;
  va_start( arglist, fmt );
  vsnprintf( msg, eMAX_MSG, fmt, arglist );
  va_end( arglist );
}
//----------------------------------------------------------------------


char const *cMsg::c_str() const
{
  return msg;
}
//----------------------------------------------------------------------

NAMESPACE_SDH_START

std::ostream &operator<<(std::ostream &stream, cMsg const &msg)
{
  return stream << msg.c_str();
}

NAMESPACE_SDH_END

//======================================================================


cSDHLibraryException::cSDHLibraryException( char const * _type, cMsg const & _msg )
  :
  // call constructor of base class:
  std::exception(),

  // call constructor of members:
  msg( "%s: %s", _type, _msg.c_str() )
{
  // nothing more to do here
}
//----------------------------------------------------------------------

const char* cSDHLibraryException::what() const throw()
{
  return msg.c_str();
}
//----------------------------------------------------------------------


NAMESPACE_SDH_START

std::ostream &operator<<(std::ostream &stream, cSDHLibraryException const &e)
{
  return stream << e.what();
}

NAMESPACE_SDH_END

//======================================================================



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
