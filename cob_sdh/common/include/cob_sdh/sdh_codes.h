//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdh_codes_h_general General file information
    \author   Dirk Osswald
    \date     2011-02-04


  \brief
    This file contains function to convert the binary command codes of the SDH.
    To use this from a non gcc compiler you might have to
    define SDH__attribute__ to nothing and SDH_USE_VCC to 1.


  \section sdh_components_sdh_codes_h_copyright Copyright

  - Copyright (c) 2003 IPR, University of Karlsruhe (TH), all rights reserved
  - Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdh_components_sdh_codes_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-02-04 19:26:19 +0100 (Fr, 04 Feb 2011) $
      \par SVN file revision:
        $Id: sdh_codes.h 6420 2011-02-04 18:26:19Z Osswald2 $

  \subsection sdh_components_sdh_codes_h_changelog Changelog of this file:
      \include sch_codes.h.log
*/
//======================================================================


#ifndef SDH_CODES_h
#define SDH_CODES_h

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

NAMESPACE_SDH_START

#include "sdh_command_codes.h"
#include "sdh_return_codes.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External functions (function declarations)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------

char const* SDHCommandCodeToString( eCommandCode cc );
char const* SDHReturnCodeToString( eReturnCode rc );

NAMESPACE_SDH_END

#endif


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
*/
//======================================================================
