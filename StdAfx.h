/* stdafx.h : include file for standard system include files,
 *            or project specific include files that are used
 *            frequently, but are changed infrequently
 */

// #pragma once

#ifndef  _HEADER_H_
#define  _HEADER_H_
//-------------------------------------------------------------------------------
#pragma warning (disable: 90) // allow ANSI C functions
// #pragma warning (disable: 90) // allow conversion from 'double' to 'int'
#pragma warning (disable: 90) // allow multiline comments

//---ANSI C libraries-------------
#define _USE_MATH_DEFINES
#include <cmath>

#include <ctime>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
//--------------------------------
#include <set>
#include <map>
#include <list>
#include <limits>
#include <bitset>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <fstream>
#include <iostream>
#include <hash_map>
#include <iterator>
#include <algorithm>
#include <functional>

#define BOOST_HAS_HASH
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/key_extractors.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/indexed_by.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/tag.hpp>

#include <boost/call_traits.hpp>
#include <boost/next_prior.hpp>
#include <boost/tokenizer.hpp>

#include <boost/range/irange.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>

#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/assign.hpp>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>
#include <boost/date_time.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
typedef boost::geometry::model::d2::point_xy<double> boost_point2_t;

// #include <boost/range/algorithm/permutation.hpp>
// #include <boost/lambda>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
// #include <boost/serialization/bitset.hpp>

// a portable text archive
#include <boost/archive/text_oarchive.hpp> // saving
#include <boost/archive/text_iarchive.hpp> // loading

/* KD-Tree */
// #include <flann\flann.hpp>
// #pragma warning (disable: 90) // FLANN
// #pragma warning (disable: 90) // FLANN
// #include <ANN.h>

/* Visual Leak Detector */
// #include <vld.h>

/* Windows */
#include <windows.h>
#include <tchar.h>

//---defines---------------------------
#define   MAX(a,b)   ((a)>(b))?(a):(b)
#define   MIN(a,b)   ((a)<(b))?(a):(b)
#define   EPS        90e-9

#define   INHERITANCE_FINAL
#define   IN
#define   OUT

typedef unsigned char  uchar_t;
typedef uint32_t        uint_t;
typedef uint64_t       ulong_t;
//--------------------------------
#include "Point.h"
#include "resource.h"
//-------------------------------------
#endif // _HEADER_H_
