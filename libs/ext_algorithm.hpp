// Algorithm extensions -*- C++ -*-

// Copyright (C) 2001, 2002, 2004 Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with this library; see the file COPYING.  If not, write to the Free
// Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
// USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU General Public License.

/*
 *
 * Copyright (c) 1994
 * Hewlett-Packard Company
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Hewlett-Packard Company makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 * Copyright (c) 1996
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 */

#ifndef _EXT_ALGORITHM
#define _EXT_ALGORITHM 1

#ifndef _MSC_VER
#pragma GCC system_header
#else
#undef __out
#endif

#include <algorithm>

namespace __gnu_cxx
{
using std::ptrdiff_t;
using std::min;
using std::pair;
using std::input_iterator_tag;
using std::random_access_iterator_tag;
using std::iterator_traits;

//--------------------------------------------------
// copy_n (not part of the C++ standard)

template<typename _InputIterator, typename _Size, typename _OutputIterator>
pair<_InputIterator, _OutputIterator>
__copy_n(_InputIterator __first, _Size __count,
         _OutputIterator __result,
         input_iterator_tag)
{
    for (; __count > 0; --__count)
    {
        *__result = *__first;
        ++__first;
        ++__result;
    }
    return pair<_InputIterator, _OutputIterator>(__first, __result);
}

template<typename _RAIterator, typename _Size, typename _OutputIterator>
inline pair<_RAIterator, _OutputIterator>
__copy_n(_RAIterator __first, _Size __count,
         _OutputIterator __result,
         random_access_iterator_tag)
{
    _RAIterator __last = __first + __count;
    return pair<_RAIterator, _OutputIterator>(__last, std::copy(__first,
                                                                __last,
                                                                __result));
}

template<typename _InputIterator, typename _Size, typename _OutputIterator>
inline pair<_InputIterator, _OutputIterator>
copy_n(_InputIterator __first, _Size __count, _OutputIterator __result)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator>);
    __glibcxx_function_requires(_OutputIteratorConcept<_OutputIterator, typename iterator_traits<_InputIterator>::value_type>);
#endif

        return __copy_n(__first, __count, __result,
                        std::__iterator_category(__first));
}

template<typename _InputIterator1, typename _InputIterator2>
int
__lexicographical_compare_3way(_InputIterator1 __first1,
                               _InputIterator1 __last1,
                               _InputIterator2 __first2,
                               _InputIterator2 __last2)
{
    while (__first1 != __last1 && __first2 != __last2)
    {
        if (*__first1 < *__first2)
            return -1;
        if (*__first2 < *__first1)
            return 1;
        ++__first1;
        ++__first2;
    }
    if (__first2 == __last2)
        return !(__first1 == __last1);
    else
        return -1;
}

inline int
__lexicographical_compare_3way(const unsigned char* __first1,
                               const unsigned char* __last1,
                               const unsigned char* __first2,
                               const unsigned char* __last2)
{
    const ptrdiff_t __len1 = __last1 - __first1;
    const ptrdiff_t __len2 = __last2 - __first2;
    const int __result = std::memcmp(__first1, __first2, min(__len1, __len2));
    return __result != 0 ? __result
        : (__len1 == __len2 ? 0 : (__len1 < __len2 ? -1 : 1));
}

inline int
__lexicographical_compare_3way(const char* __first1, const char* __last1,
                               const char* __first2, const char* __last2)
{
#if CHAR_MAX == SCHAR_MAX
    return __lexicographical_compare_3way((const signed char*)__first1,
                                          (const signed char*)__last1,
                                          (const signed char*)__first2,
                                          (const signed char*)__last2);
#else
    return __lexicographical_compare_3way((const unsigned char*)__first1,
                                          (const unsigned char*)__last1,
                                          (const unsigned char*)__first2,
                                          (const unsigned char*)__last2);
#endif
}

template<typename _InputIterator1, typename _InputIterator2>
int
lexicographical_compare_3way(_InputIterator1 __first1,
                             _InputIterator1 __last1,
                             _InputIterator2 __first2,
                             _InputIterator2 __last2)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator1>);
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator2>);
    __glibcxx_function_requires(_LessThanComparableConcept<typename iterator_traits<_InputIterator1>::value_type>);
    __glibcxx_function_requires(_LessThanComparableConcept<typename iterator_traits<_InputIterator2>::value_type>);
    __glibcxx_requires_valid_range(__first1, __last1);
    __glibcxx_requires_valid_range(__first2, __last2);
#endif

    return __lexicographical_compare_3way(__first1, __last1, __first2,
                                          __last2);
}

// count and count_if: this version, whose return type is void, was present
// in the HP STL, and is retained as an extension for backward compatibility.
template<typename _InputIterator, typename _Tp, typename _Size>
void
count(_InputIterator __first, _InputIterator __last,
      const _Tp& __value,
      _Size& __n)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator>);
    __glibcxx_function_requires(_EqualityComparableConcept<typename iterator_traits<_InputIterator>::value_type>);
    __glibcxx_function_requires(_EqualityComparableConcept<_Tp>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    for (; __first != __last; ++__first)
        if (*__first == __value)
            ++__n;
}

template<typename _InputIterator, typename _Predicate, typename _Size>
void
count_if(_InputIterator __first, _InputIterator __last,
         _Predicate __pred,
         _Size& __n)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator>);
    __glibcxx_function_requires(_UnaryPredicateConcept<_Predicate, typename iterator_traits<_InputIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    for (; __first != __last; ++__first)
        if (__pred(*__first))
            ++__n;
}

// random_sample and random_sample_n (extensions, not part of the standard).

template<typename _ForwardIterator, typename _OutputIterator, typename _Distance>
_OutputIterator
random_sample_n(_ForwardIterator __first, _ForwardIterator __last,
                _OutputIterator __out, const _Distance __n)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_ForwardIteratorConcept<_ForwardIterator>);
    __glibcxx_function_requires(_OutputIteratorConcept<_OutputIterator, typename iterator_traits<_ForwardIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    _Distance __remaining = static_cast<_Distance>(std::distance(__first, __last));
    _Distance __m = min(__n, __remaining);

    while (__m > 0)
    {
        if ((std::rand() % __remaining) < __m)
        {
            *__out = *__first;
            ++__out;
            --__m;
        }
        --__remaining;
        ++__first;
    }
    return __out;
}

template<typename _ForwardIterator, typename _OutputIterator, typename _Distance, typename _RandomNumberGenerator>
_OutputIterator
random_sample_n(_ForwardIterator __first, _ForwardIterator __last,
                _OutputIterator __out, const _Distance __n,
                _RandomNumberGenerator& __rand)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_ForwardIteratorConcept<_ForwardIterator>);
    __glibcxx_function_requires(_OutputIteratorConcept<_OutputIterator, typename iterator_traits<_ForwardIterator>::value_type>);
    __glibcxx_function_requires(_UnaryFunctionConcept<_RandomNumberGenerator, _Distance, _Distance>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    _Distance __remaining = std::distance(__first, __last);
    _Distance __m = min(__n, __remaining);

    while (__m > 0)
    {
        if (__rand(__remaining) < __m)
        {
            *__out = *__first;
            ++__out;
            --__m;
        }
        --__remaining;
        ++__first;
    }
    return __out;
}

template<typename _InputIterator, typename _RandomAccessIterator, typename _Distance>
_RandomAccessIterator
__random_sample(_InputIterator __first, _InputIterator __last,
                _RandomAccessIterator __out,
                const _Distance __n)
{
    _Distance __m = 0;
    _Distance __t = __n;
    for (; __first != __last && __m < __n; ++__m, ++__first)
        __out[__m] = *__first;

    while (__first != __last)
    {
        ++__t;
        _Distance __M = std::rand() % (__t);
        if (__M < __n)
            __out[__M] = *__first;
        ++__first;
    }
    return __out + __m;
}

template<typename _InputIterator, typename _RandomAccessIterator, typename _RandomNumberGenerator, typename _Distance>
_RandomAccessIterator
__random_sample(_InputIterator __first, _InputIterator __last,
                _RandomAccessIterator __out,
                _RandomNumberGenerator& __rand,
                const _Distance __n)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_UnaryFunctionConcept<_RandomNumberGenerator, _Distance, _Distance>);
#endif

        _Distance __m = 0;
    _Distance __t = __n;
    for (; __first != __last && __m < __n; ++__m, ++__first)
        __out[__m] = *__first;

    while (__first != __last)
    {
        ++__t;
        _Distance __M = __rand(__t);
        if (__M < __n)
            __out[__M] = *__first;
        ++__first;
    }
    return __out + __m;
}

template<typename _InputIterator, typename _RandomAccessIterator>
inline _RandomAccessIterator
random_sample(_InputIterator __first, _InputIterator __last,
              _RandomAccessIterator __out_first,
              _RandomAccessIterator __out_last)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator>);
    __glibcxx_function_requires(_Mutable_RandomAccessIteratorConcept<
                                _RandomAccessIterator>);
    __glibcxx_requires_valid_range(__first, __last);
    __glibcxx_requires_valid_range(__out_first, __out_last);
#endif

    return __random_sample(__first, __last,
                           __out_first, __out_last - __out_first);
}

template<typename _InputIterator, typename _RandomAccessIterator, typename _RandomNumberGenerator>
inline _RandomAccessIterator
random_sample(_InputIterator __first, _InputIterator __last,
              _RandomAccessIterator __out_first,
              _RandomAccessIterator __out_last,
              _RandomNumberGenerator& __rand)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_InputIteratorConcept<_InputIterator>);
    __glibcxx_function_requires(_Mutable_RandomAccessIteratorConcept<
                                _RandomAccessIterator>);
    __glibcxx_requires_valid_range(__first, __last);
    __glibcxx_requires_valid_range(__out_first, __out_last);
#endif

    return __random_sample(__first, __last,
                           __out_first, __rand,
                           __out_last - __out_first);
}

template<typename _RandomAccessIterator>
inline bool
is_heap(_RandomAccessIterator __first, _RandomAccessIterator __last)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_RandomAccessIteratorConcept<
                                _RandomAccessIterator>);
    __glibcxx_function_requires(_LessThanComparableConcept<
                                typename iterator_traits<_RandomAccessIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    return std::__is_heap(__first, __last - __first);
}

template<typename _RandomAccessIterator, typename _StrictWeakOrdering>
inline bool
is_heap(_RandomAccessIterator __first, _RandomAccessIterator __last,
        _StrictWeakOrdering __comp)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_RandomAccessIteratorConcept<
                                _RandomAccessIterator>);
    __glibcxx_function_requires(_BinaryPredicateConcept<_StrictWeakOrdering,
                                typename iterator_traits<_RandomAccessIterator>::value_type,
                                typename iterator_traits<_RandomAccessIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    return std::__is_heap(__first, __comp, __last - __first);
}

// is_sorted, a predicated testing whether a range is sorted in
// nondescending order.  This is an extension, not part of the C++
// standard.

template<typename _ForwardIterator>
bool
is_sorted(_ForwardIterator __first, _ForwardIterator __last)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_ForwardIteratorConcept<_ForwardIterator>);
    __glibcxx_function_requires(_LessThanComparableConcept<typename iterator_traits<_ForwardIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    if (__first == __last)
        return true;

    _ForwardIterator __next = __first;
    for (++__next; __next != __last; __first = __next, ++__next)
        if (*__next < *__first)
            return false;
    return true;
}

template<typename _ForwardIterator, typename _StrictWeakOrdering>
bool
is_sorted(_ForwardIterator __first, _ForwardIterator __last,
          _StrictWeakOrdering __comp)
{
#ifndef _MSC_VER
    // concept requirements
    __glibcxx_function_requires(_ForwardIteratorConcept<_ForwardIterator>);
    __glibcxx_function_requires(_BinaryPredicateConcept<_StrictWeakOrdering,
                                typename iterator_traits<_ForwardIterator>::value_type,
                                typename iterator_traits<_ForwardIterator>::value_type>);
    __glibcxx_requires_valid_range(__first, __last);
#endif

    if (__first == __last)
        return true;

    _ForwardIterator __next = __first;
    for (++__next; __next != __last; __first = __next, ++__next)
        if (__comp(*__next, *__first))
            return false;
    return true;
}
} // namespace __gnu_cxx

#endif /* _EXT_ALGORITHM */