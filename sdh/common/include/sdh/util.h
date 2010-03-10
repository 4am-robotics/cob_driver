//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_util_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of auxilliary utility functions for SDHLibrary-CPP.

  \section sdhlibrary_cpp_util_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_util_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
      \par SVN file revision:
        $Id: util.h 3659 2008-10-08 08:48:38Z Osswald2 $

  \subsection sdhlibrary_cpp_util_h_changelog Changelog of this file:
      \include util.h.log
*/
//======================================================================

#ifndef UTIL_H_
#define UTIL_H_


//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>

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

#ifndef M_PI
static double M_PI = 4.0*atan(1.0);
#endif

//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------



/*!
  \anchor sdhlibrary_cpp_util_h_auxiliary
  \name   Auxiliary functions

  @{
*/
//-----------------------------------------------------------------

/*!
    Return True if v is in range [0 .. max[
*/
bool InIndex( int v, int max );


/*!
    Return True if v is in range [min .. max]
*/
bool InRange( double v, double min, double max );


/*!
    Return True if in list/tuple/array v=(v1,v2,...)  each
    v_i is in range [min_i..max_i] with
    min = (min1, min2,...) max = (max1, max2, ..)
*/
bool InRange( int n, double const* v, double const* min, double const* max );


/*!
    Return v limited to range [min .. max]. I.e. if v is < min then
    min is returned, or if v > max then max is returned, else v is
    returned
*/
double ToRange( double v, double min, double max );


/*!
    Limit each v_i in v to range [min_i..max_i] with
    min = (min1, min2,...) max = (max1, max2, ..)
    This modifies \a *v!
*/
void ToRange( int n, double* v, double const* min, double const* max );


/*!
    Limit each v_i in v to range [min_i..max_i] with
    min = (min1, min2,...) max = (max1, max2, ..)
    This modifies \a v!
*/
void ToRange( std::vector<double>& v, std::vector<double> const& min, std::vector<double> const& max );


/*!
    Return True if a is approximately the same as b. I.E. |a-b| < eps
*/
double Approx( double a, double b, double eps );


/*!
    Return True if list/tuple/array a=(a1,a2,...) is approximately
    the same as b=(b1,b2,...). I.E. |a_i-b_i| < eps[i]
*/
bool Approx( int n, double* a, double* b, double* eps );


/*!
    Return d in deg converted to rad
*/
double DegToRad( double d );


/*!
    Return r in rad converted to deg
*/
double RadToDeg( double r );


/*!
    Sleep for t seconds. (t is a double!)
*/
void SleepSec( double t );


/*!
   Apply a function to every element of a sequence, the elements of the sequence are modified by \a f

   \param  f        A unary function object.
   \param  sequence The iterable sequence to modify.

   Applies the function object \p f to each element of the \a sequence.
 */
template<typename Function, typename Tp>
void apply( Function f, Tp& sequence )
{
    // concept requirements

    for ( typename Tp::iterator it = sequence.begin();
          it < sequence.end();
          it++ )
    {
        *it = f( *it );
    }
}
//----------------------------------------------------------------------


/*!
    Apply a function to every element of a sequence.

    \param  first  An input iterator.
    \param  last   An input iterator.
    \param  f      A unary function object.
    \return   \p f.

    Applies the function object \p f to each element in the range
    \p [first,last).  \p f must not modify the order of the sequence.
    If \p f has a return value it is ignored.
 */
template<typename Function, typename InputIterator>
Function
apply( Function f, InputIterator first, InputIterator last)
{
    // concept requirements (not understood by gcc 3.2, thus commented out)
    //__glibcxx_function_requires(_InputIteratorConcept<InputIterator>)
    //  __glibcxx_requires_valid_range(first, last);
  for ( ; first != last; ++first)
        *first = f(*first);
}
//----------------------------------------------------------------------

/*!
    map a function to every element of a sequence, returning a copy with the mapped elements

    \param  f      - A unary function object.
    \param  sequence - An iterable object.
    \return copy of \a sequence with the mapped elements
 */
template<typename Function, typename Tp>
Tp map(Function f, Tp sequence)
{
    Tp result (sequence);

    apply( f, result );

    return result;
}
//----------------------------------------------------------------------

/*!
 * Overloaded insertion operator for vectors: a comma and space separated list of the vector elements of \a v is inserted into \a stream
 *
 * @param stream - the output stream to insert into
 * @param v      - the vector of objects to insert into \a stream
 * @return the stream with the inserted objects
 *
 * \attention
 * If you use the SDH namespace then you should be aware that using this overloaded insertion operator can get tricky:
 * - If you use a "using namespace SDH" directive then things are easy and intuitive:
 *   \code
 *     #include <sdh/util.h>
 *     using namespace SDH;
 *     std::vector<int> v;
 *     std::cout << "this is a std::vector: " << v << "\n";
 *   \endcode
 * - But without the \c using \c namespace \c SDH accessing the operator is tricky,
 *   you have to use the 'functional' access \c operator<<(s,v) in order to be
 *   able to apply the scope resolution operator \c :: correctly:
 *   \code
 *     #include <sdh/util.h>
 *     std::vector<int> v;
 *     SDH::operator<<( std::cout << "this is a std::vector: ", v ) << "\n";
 *   \endcode
 * - The more intuitive approaches do not work:
 *   \code
 *     std::cout << faa ;                // obviously fails with "no match for >>operator<<<< in >>std::cout << faa<<", as expected
 *     std::cout SDH::<< faa ;           // is a syntax error
 *     std::cout SDH::operator<< faa ;   // is a syntax error
 *     std::cout operator SDH::<< faa ;  // is a syntax error
 *   \endcode
 *
 */
template<typename T>
std::ostream& operator<<(std::ostream& stream, std::vector<T> v)
{
    char* sep = "";

    typename std::vector<T>::const_iterator it;
    for ( it = v.begin();
          it != v.end();
          it++ )
    {
        stream << sep << *it ;
        sep = ", ";
    }

    return stream;
}
//----------------------------------------------------------------------


//! @}   // end of doxygen name group sdhlibrary_cpp_util_h_auxiliary

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
