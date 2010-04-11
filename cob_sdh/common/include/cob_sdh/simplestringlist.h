//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simplestirnglist_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSimpleStringList.

  \section sdhlibrary_cpp_simplestirnglist_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simplestirnglist_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-08-31 15:46:47 +0200 (Mo, 31 Aug 2009) $
      \par SVN file revision:
        $Id: simplestringlist.h 4766 2009-08-31 13:46:47Z Osswald2 $

  \subsection sdhlibrary_cpp_simplestirnglist_h_changelog Changelog of this file:
      \include simplestringlist.h.log
*/
//======================================================================

#ifndef SIMPLESTRINGLIST_H_
#define SIMPLESTRINGLIST_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <iosfwd>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------



//! A simple string list. (Fixed maximum number of strings of fixed maximum length)
class cSimpleStringList
{
public:

    //! the index of the current line. For empty cSimpleStringLists this is -1.
    int current_line;


    //! anonymous enum instead of define macros
    enum {
        eMAX_LINES = 256,
        eMAX_CHARS = 256,
    };

    //! Default constructor: init members
    cSimpleStringList();


    //! Return the current line
    char* CurrentLine();


    //! Return the next line, this increases current_line
    char* NextLine();


    //! Return number of lines stored
    int Length() const;


    //! return ptr to line with index.
    /*!
      if index < 0 then the numbering starts from the end,
      thus [-1] gives the last line, [-2] the next to last, ...
    */
    char* operator[]( int index );

    //! return ptr to line with index.
    /*!
      if index < 0 then the numbering starts from the end,
      thus [-1] gives the last line, [-2] the next to last, ...
    */
    char const* operator[]( int index ) const;


    //! reset list
    void Reset();

protected:
    //! a fixed length array of lines with fixed length
    char line[ eMAX_LINES ][ eMAX_CHARS ];

}; // cSimpleStringList
//-----------------------------------------------------------------


//! Output of cSimpleStringList objects in 'normal' output streams.
std::ostream& operator<<( std::ostream& stream, cSimpleStringList const& ssl );


//-----------------------------------------------------------------

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

