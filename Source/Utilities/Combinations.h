#include "StdAfx.h"

#ifndef  _COMB_H_
#define  _COMB_H_

// 25.3.9, combinatorial enumeration:
// ...
/*
A permutation of size r of a range of size n is a (not necessarily) sorted subsequence
of size r of the total range, i.e., a subsequence of elements at r positions among
the n positions in the range. 

A combination of size r of a range of size n is a sorted subsequence of size r of the
total range, i.e., the ordered (possibly multi-)set of the elements at r positions
among the n positions in the range.

A permutation or combination is without repetition if the r indices in the respective
definition are distinct (and necessarily r <= n), and with repetition otherwise.
*/

template  <class  BidirectionalIterator>
bool  next_partial_permutation (BidirectionalIterator  first,
                                BidirectionalIterator  middle,
                                BidirectionalIterator  last)
{
  reverse (middle, last);
  return  next_permutation (first, last);
}
template <class  BidirectionalIterator>
bool  prev_partial_permutation (BidirectionalIterator  first,
                                BidirectionalIterator  middle,
                                BidirectionalIterator  last)
{
  bool  result = prev_permutation (first, last);
  reverse (middle, last);
  return  result;
}

// -----------------------
namespace
{
  template <class  BidirectionalIterator>
  bool  next_combination (BidirectionalIterator  first1,
                          BidirectionalIterator  last1,
                          BidirectionalIterator  first2,
                          BidirectionalIterator  last2)
  {
    if ( (first1 == last1) || (first2 == last2) )
    {
      return  false;
    }
    BidirectionalIterator  m1 = last1;
    BidirectionalIterator  m2 = last2; --m2;
    while ( --m1 != first1  && !(*m1 < *m2) )
    {
    }
    bool  result = (m1 == first1) && !(*first1  < *m2);
    if ( !result )
    {
      while ( first2 != m2 && !(*m1 < *first2) )
      {
        ++first2;
      }
      first1 = m1;
      std::iter_swap (first1, first2);
      ++first1;
      ++first2;
    }
    if ( (first1 != last1) && (first2 != last2) )
    {
      m1 = last1;   m2 = first2;
      while ( (m1 != first1) && (m2 != last2) )
      {
        std::iter_swap (--m1, m2);
        ++m2;
      }
      std::reverse (first1, m1);
      std::reverse (first1, last1);
      std::reverse (m2, last2);
      std::reverse (first2, last2);
    }
    return !result;
  }
} // namespace detail

template <class  BidirectionalIterator>
inline bool  next_combination (BidirectionalIterator  first,
                               BidirectionalIterator  middle,
                               BidirectionalIterator  last)
{ return next_combination (first, middle, middle, last); }

template <class  BidirectionalIterator>
inline bool  prev_combination (BidirectionalIterator  first,
                               BidirectionalIterator  middle,
                               BidirectionalIterator  last)
{ return next_combination (middle, last, first, middle); }

template <class     BidirectionalIterator, class T>
bool  next_mapping (BidirectionalIterator  first,
                    BidirectionalIterator  last,
                    T first_value, T last_value)
{
  if ( last == first )
  { return  false; }
  do
  {
    if ( ++(*(--last)) != last_value )
    { return  true; }
    *last = first_value;
  } while ( last != first );
  return  false;
}

template  <class    BidirectionalIterator, class T>
bool  prev_mapping (BidirectionalIterator  first,
                    BidirectionalIterator  last,
                    T first_value, T last_value)
{
  if ( last == first )
  { return  false; }
  --last_value;
  do
  {
    if ( *(--last) != first_value )
    {
      --(*last);
      return  true;
    }
    *last = last_value;
  } while ( last != first );
  return  true;
}

template  <class  BidirectionalIterator>
bool  next_repeat_combination_counts (BidirectionalIterator  first,
                                      BidirectionalIterator  last)
{
  BidirectionalIterator  current = last;
  while ( current != first && *(--current) == 0 )
  {
  }
  if ( current == first )
  {
    if ( first != last && *first != 0 )
      std::iter_swap (--last, first);
    return  false;
  }
  --(*current);
  std::iter_swap (--last, current);
  ++(*(--current));
  return  true;
}

template  <class  BidirectionalIterator >
bool  prev_repeat_combination_counts (BidirectionalIterator  first,
                                      BidirectionalIterator  last)
{
  if ( first == last )
    return  false;
  BidirectionalIterator  current = --last;
  while ( current != first && *(--current) == 0 )
  {}

  if ( current == last || current == first && *current == 0 )
  {
    if ( first != last )
      std::iter_swap (first, last);
    return  false;
  }
  --(*current);
  ++current;
  if ( 0 != *last )
  { std::iter_swap (current, last); }
  ++(*current);
  return  true;
}






template  <typename  RandomAccessIter, typename  Functor >
void  for_each_tuple (RandomAccessIter  first,
                      RandomAccessIter  middle,
                      RandomAccessIter  last, Functor f)
{
  std::sort (first, last);
  do
  {
    f (first, middle);
  } while ( next_partial_permutation (first, middle, last) );
}

template  <typename  RandomAccessIter, typename  Functor >
void  for_each_subset (RandomAccessIter  first,
                       RandomAccessIter  middle,
                       RandomAccessIter  last, Functor f)
{
  std::sort (first, last);
  do
  {
    f (first, middle);
  } while ( next_combination (first, middle, last) );
}

/*
These are standard concepts in combinatorics.
- partial permutations of a range,
- combinations (i.e., selection of a subset of a given
size without replacement (repetitions), where order matters or not).
*/

template  <typename  RandomAccessIter, typename  Functor >
void  for_each_triplet (RandomAccessIter  first,
                        RandomAccessIter  last,
                        Functor f)
{
  for ( RandomAccessIter i = first; i != last; ++i )
  {
    for ( RandomAccessIter j = first; j != last; ++j )
    {
      if ( i == j ) continue;
      for ( RandomAccessIter k = first; k != last; ++k )
      {
        if ( i == k || j == k ) continue;
        f (i, j, k);
      }
    }
  }
}

/* a triplet should be enumerated once, not once for each of its six permutations) */
template  <typename  RandomAccessIter, typename  Functor >
void  for_each_3_subset (RandomAccessIter  first,
                         RandomAccessIter  last,
                         Functor f)
{
  for ( RandomAccessIter i = first; i != last; ++i )
  {
    for ( RandomAccessIter j = i + 1; j != last; ++j )
    {
      for ( RandomAccessIter k = j + 1; k != last; ++k )
      {
        f (i, j, k);
      }
    }
  }
}


#endif // _COMB_H_
