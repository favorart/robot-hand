#include "StdAfx.h"

#ifndef  _COMB_H_
#define  _COMB_H_

// 25.3.9, combinatorial enumeration:
// ...
template <class  BidirectionalIterator>
bool  next_partial_permutation (BidirectionalIterator  first,
                                BidirectionalIterator  middle,
                                BidirectionalIterator  last);

// template <class  BidirectionalIterator, class  Compare>
// bool  next_partial_permutation (BidirectionalIterator  first,
//                                 BidirectionalIterator  middle,
//                                 BidirectionalIterator  last, Compare  comp);
template <class  BidirectionalIterator>
bool  prev_partial_permutation (BidirectionalIterator  first,
                                BidirectionalIterator  middle,
                                BidirectionalIterator  last);

// template <class  BidirectionalIterator, class  Compare>
// bool  prev_partial_permutation (BidirectionalIterator  first,
//                                 BidirectionalIterator  middle,
//                                 BidirectionalIterator  last, Compare  comp);

template <class  BidirectionalIterator>
inline bool    next_combination (BidirectionalIterator  first,
                                 BidirectionalIterator  middle,
                                 BidirectionalIterator  last);

// template <class  BidirectionalIterator, class  Compare>
// bool  next_combination (BidirectionalIterator  first,
//                         BidirectionalIterator  middle,
//                         BidirectionalIterator  last, Compare  comp);

template <class  BidirectionalIterator>
inline bool    prev_combination (BidirectionalIterator  first,
                        BidirectionalIterator  middle,
                        BidirectionalIterator  last);

// template <class  BidirectionalIterator, class  Compare>
// bool  prev_combination (BidirectionalIterator  first,
//                         BidirectionalIterator  middle,
//                         BidirectionalIterator  last, Compare  comp);

template <class  BidirectionalIterator, class T>
bool  next_mapping (BidirectionalIterator  first,
                    BidirectionalIterator  last,
                    T first_value, T last_value);

// template <class  BidirectionalIterator, class T, class  Incrementor>
// bool  next_mapping (BidirectionalIterator  first,
//                     BidirectionalIterator  last,
//                     T first_value, T last_value, Incrementor  increment);

template <class  BidirectionalIterator, class T>
bool  prev_mapping (BidirectionalIterator  first,
                    BidirectionalIterator  last,
                    T first_value, T last_value);

// template <class  BidirectionalIterator, class T, class  Decrementor>
// bool  prev_mapping (BidirectionalIterator  first,
//                     BidirectionalIterator  last,
//                     T first_value, T last_value, Decrementor  decrement);

template <class  BidirectionalIterator>
bool  next_repeat_combination_counts (BidirectionalIterator  first,
                                      BidirectionalIterator  last);

template <class  BidirectionalIterator>
bool  prev_repeat_combination_counts (BidirectionalIterator  first,
                                      BidirectionalIterator  last);


template <class  BidirectionalIterator>
bool  next_partial_permutation (BidirectionalIterator  first,
                                BidirectionalIterator  middle,
                                BidirectionalIterator  last);

 template <class  BidirectionalIterator, class  Compare>
 bool  next_partial_permutation (BidirectionalIterator  first,
                                 BidirectionalIterator  middle,
                                 BidirectionalIterator  last, Compare  comp);

#endif // _COMB_H_
