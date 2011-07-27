//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_serialbase_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-20

  \brief
    Implementation of class #SDH::cSerialBase, a virtual base class to access serial interfaces like RS232 or CAN.

  \section sdhlibrary_cpp_serialbase_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_serialbase_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-08 13:36:01 +0100 (Di, 08 Mrz 2011) $
      \par SVN file revision:
        $Id: serialbase.cpp 6521 2011-03-08 12:36:01Z Osswald2 $

  \subsection sdhlibrary_cpp_serialbase_cpp_changelog Changelog of this file:
      \include serialbase.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <termios.h>
# include <unistd.h>
# include <sys/select.h>
# include <sys/ioctl.h>
#endif
#include <errno.h>
//#include <string.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <cstring>    // needed in gcc-4.3 for prototypes like strcmp according to http://gcc.gnu.org/gcc-4.3/porting_to.html

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "serialbase.h"
#include "simpletime.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;
USING_NAMESPACE_SDH

char* cSerialBase::readline( char* line, int size, char const* eol, bool return_on_less_data )
    throw(cSerialBaseException*)
{
    line[0] = '\0';
    int len = 0;
    int n;
    char c;

    if (ungetch_valid)
    {
        line[len++] = ungetch;
        ungetch_valid = false;
    }

    ////cSimpleTime start;
    long timeout_us;
    if ( timeout >= 0.0 )
        timeout_us = (long) (timeout*1000000.0);
    else
        timeout_us = -1L;
    while (true)
    {
        //    n = ::read(fd, line + len, 1);
        n = Read( line + len, 1, timeout_us, return_on_less_data );

        if (n>0)
        {
            c = line[ len ];
            len += n;

            //cout << "read '" << c << "' from device\n"; cout.flush();

            if (strchr( eol, c ) != NULL)
                break;


            if (size > 0  &&  len >= size-1)
                break;
        }
        else
        {
            throw new cSerialBaseException( cMsg( "Timeout while reading line from device (timeout_us=%ld line=\"%s\")", timeout_us, line ) );
        }
    }
    // mark end of line
    line[ len ] = '\0';

    ////cerr << "reading line took " << start.Elapsed() << "s\n";
    return line;
}
//----------------------------------------------------------------------

char const* cSerialBase::GetErrorMessage( tErrorCode dw )
{
    static char return_msg[512];
#if SDH_USE_VCC
    LPVOID lpMsgBuf;

    FormatMessageA( FORMAT_MESSAGE_ALLOCATE_BUFFER |
                    FORMAT_MESSAGE_FROM_SYSTEM |
                    FORMAT_MESSAGE_IGNORE_INSERTS,
                    NULL,
                    dw,
                    MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                    (LPSTR) &lpMsgBuf,
                    0,
                    NULL );

    StringCchPrintfA( return_msg,
                      511,
                      "error 0x%x = %d = \"%s\"",
                      dw, dw, lpMsgBuf );
    LocalFree(lpMsgBuf);
#else
    snprintf( return_msg, 511, "error 0x%x = %d = \"%s\"", dw, dw, strerror( dw ) );
#endif
    return return_msg;
}
//----------------------------------------------------------------------


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
