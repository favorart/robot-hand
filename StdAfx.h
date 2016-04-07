/* stdafx.h : include file for standard system include files,
 *            or project specific include files that are used
 *            frequently, but are changed infrequently
 */
#ifndef  _HEADER_H_
#define  _HEADER_H_

 // #pragma once
//-------------------------------------------------------------------------------
#pragma warning (disable: 4010) // allow multiline comments

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
#include <string>
#include <vector>
#include <memory>
#include <numeric>
#include <utility>
#include <fstream>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <unordered_map>

// #if !defined(NDEBUG)
// #define BOOST_MULTI_INDEX_ENABLE_INVARIANT_CHECKING
// #define BOOST_MULTI_INDEX_ENABLE_SAFE_MODE
// #endif

// #include <boost/config.hpp> /* keep it first to prevent nasty warns in MSVC */
//#define BOOST_HAS_HASH
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/key_extractors.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
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

#include <boost/functional/hash.hpp>

#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/algorithm/cxx11/one_of.hpp>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/algorithm/cxx11/copy_n.hpp>
#include <boost/algorithm/cxx11/iota.hpp>
#include <boost/algorithm/cxx11/find_if_not.hpp>
#include <boost/algorithm/cxx11/is_sorted.hpp>
// #include <boost/algorithm/cxx11/is_partitioned.hpp>
// #include <boost/algorithm/cxx11/is_permutation.hpp>
//#include <boost/algorithm/cxx11/partition_copy.hpp>
//#include <boost/algorithm/cxx11/partition_point.hpp>

#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>

#include <boost/assign.hpp>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include <boost/date_time.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/linestring.hpp>

typedef boost::geometry::model::d2::point_xy<double> boost_point2_t;
inline double  boost_distance (boost_point2_t a, boost_point2_t b)
{ return boost::geometry::distance<boost_point2_t> (a,b); }

// #include <boost/range/algorithm/permutation.hpp>
// #include <boost/lambda>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/list.hpp>
// #include <boost/serialization/map.hpp>
// #include <boost/serialization/bitset.hpp>

// a portable text archive
#include <boost/archive/text_oarchive.hpp> // saving
#include <boost/archive/text_iarchive.hpp> // loading
// a binary archive
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

// #include <boost/interprocess/allocators/allocator.hpp>
// #include <boost/interprocess/containers/string.hpp>
// #include <boost/interprocess/managed_mapped_file.hpp>
// #include <boost/interprocess/sync/named_mutex.hpp>
// #include <boost/interprocess/sync/scoped_lock.hpp>

/* Visual Leak Detector */
// #include <vld.h>

/* Windows */
#include <windows.h>
#include <tchar.h>

// TCHAR based std::string
typedef std::basic_string<TCHAR> tstring;
// TCHAR based std::istream
typedef std::basic_istream<TCHAR> tistream;
// TCHAR based std::ostream
typedef std::basic_ostream<TCHAR> tostream;
// TCHAR based std::fstream;
typedef std::basic_fstream<TCHAR> tfstream;
// TCHAR based std::stringstream
typedef std::basic_stringstream<TCHAR> tstringstream;

#if defined(UNICODE) || defined(_UNICODE)
#define tcout std::wcout
#else
#define tcout std::cout
#endif

//---defines---------------------------
#define   EPS        1e-4
#define   EPS_VIS    1e-2

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
