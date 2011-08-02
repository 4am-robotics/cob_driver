//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhlibrary_settings_h_general General file information
  \author   Dirk Osswald
  \date     2008-05-20


  \brief This file contains settings to make the SDHLibrary compile on differen systems:
  - gcc/Cygwin/Windows
  - gcc/Linux
  - VisualC++/Windows


  \section sdhlibrary_cpp_sdhlibrary_settings_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_sdhlibrary_settings_h_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
  \par SVN file revision:
  $Id: sdhlibrary_settings.h 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhlibrary_settings_h_changelog Changelog of this file:
  \include sdhlibrary_settings.h.log
*/
//======================================================================

#ifndef SDHLIBRARY_SETTINGS_h_
#define SDHLIBRARY_SETTINGS_h_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

/*!
  \defgroup sdhlibrary_cpp_sdhlibrary_settings_h_settings_group Compile time settings

  Primary settings to be adjusted by the user.

  @{
*/

//! Flag, if 1 then all classes are put into a namespace called #SDH. If 0 then the classes are left outside any namespace.
#define  SDH_USE_NAMESPACE  1

//! Flag, if 1 then binary communication is used where possible for better performance (requires at least firmware 0.0.2.15)
#define  SDH_USE_BINARY_COMMUNICATION 1

//! @}   // end of doxygen module group sdhlibrary_cpp_sdhlibrary_settings_h_settings_group
//-----------------------------------------------------------------


/*!
  \defgroup sdhlibrary_cpp_sdhlibrary_settings_h_derived_settings_group Derived compile time settings

  Derived settings that users should not have to adjust. Adjustments should
  be done in \ref sdhlibrary_cpp_sdhlibrary_settings_h_settings_group "Compile time settings"

  @{
*/


#if SDH_USE_NAMESPACE
# define NAMESPACE_SDH_START namespace SDH {
# define NAMESPACE_SDH_END   }
# define USING_NAMESPACE_SDH using namespace SDH;
# define NS_SDH              SDH::
#else
# define NAMESPACE_SDH_START
# define NAMESPACE_SDH_END
# define USING_NAMESPACE_SDH
# define NS_SDH
#endif


//---------------------
#if defined( OSNAME_CYGWIN ) || defined( OSNAME_LINUX )
//#warning "using settings for Cygwin and Linux"

// settings when run on Windows/cygwin or Linux:

//! Flag, 0 (Visual C Compiler) is not used
# define SDH_USE_VCC 0

//! gcc knows how to check printf like format strings
# define SDH__attribute__( ... ) __attribute__(__VA_ARGS__)

//! gcc checks for NAN (Not a number) with isnan
# define SDH_ISNAN( V ) isnan( (V) )

# define VCC_EXPORT
# define VCC_EXPORT_TEMPLATE

//---------------------
// WIN32 might be defined by ntcan.h
#elif defined( WIN32 ) && ( ! defined( OSNAME_CYGWIN ) ) && ( ! defined( OSNAME_LINUX ) )
//#warning "using settings for VCC"

//! Flag, 1 if VCC (Visual C Compiler) is used
# define SDH_USE_VCC 1

//! VCC does not know how to check for printf like format strings
# define SDH__attribute__( ... )

//! VCC checks for NAN (Not a number) with _isnan
# define SDH_ISNAN( V ) _isnan( (V) )

typedef long          ssize_t;

/*!
 * \def SDH_IN_SDHLIBRARY_DLL
 * The SDH_IN_SDHLIBRARY_DLL macro must be defined when compiling the
 * SDHLibrary-C++ as a DLL with VCC, since w have to 'flatter' VCC to
 * actually export some functions into a DLL... Sigh!
 *
 * See http://support.microsoft.com/default.aspx?scid=kb;EN-US;q168958
 * but be aware of http://www.codesynthesis.com/~boris/blog/2010/01/18/dll-export-cxx-templates/
 * as well.
 */

//disable warnings on extern before template instantiation
# pragma warning (disable : 4231)
# ifdef SDH_IN_SDHLIBRARY_DLL
#  define VCC_EXPORT           __declspec( dllexport )
#  define VCC_EXPORT_TEMPLATE
# else
#  define VCC_EXPORT           __declspec( dllimport )
#  define VCC_EXPORT_TEMPLATE  extern
# endif


//---------------------
#else
# error "Unknown compiler, please adjust settings!"
#endif


//! @}   // end of doxygen module group sdhlibrary_cpp_sdhlibrary_settings_h_derived_settings

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions and classes (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


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
