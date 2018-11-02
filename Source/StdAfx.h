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
#include <stack>
#include <limits> // std::numeric_limits<T>
#include <string>
#include <bitset>
#include <vector>
#include <memory> // for std::allocator
#include <numeric>
#include <utility>
#include <fstream>
#include <codecvt> // utf converter
#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <unordered_map>
//--------------------------------

#ifdef _MSC_VER
#include <boost/config/compiler/visualc.hpp>
#endif
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
namespace pt = boost::property_tree;

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

#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext/is_sorted.hpp>
#include <boost/range/combine.hpp>
#include <boost/range/iterator.hpp>
#include <boost/range/irange.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>

#include <boost/math/special_functions/sign.hpp>
#include <boost/assign.hpp>
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
//#include <boost/algorithm/cxx11/is_partitioned.hpp>
//#include <boost/algorithm/cxx11/is_permutation.hpp>
//#include <boost/algorithm/cxx11/partition_copy.hpp>
//#include <boost/algorithm/cxx11/partition_point.hpp>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include <boost/date_time.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> boost_point2_t;
inline double boost_distance(boost_point2_t a, boost_point2_t b)
{ return boost::geometry::distance<boost_point2_t>(a, b); }

namespace ba = boost::algorithm;
namespace br = boost::range;
namespace bg = boost::geometry;


//#include <boost/range/algorithm/permutation.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/function.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/export.hpp> 

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/property_tree/ptree_serialization.hpp>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

//#include <boost/interprocess/allocators/allocator.hpp>
//#include <boost/interprocess/containers/string.hpp>
//#include <boost/interprocess/managed_mapped_file.hpp>
//#include <boost/interprocess/sync/named_mutex.hpp>
//#include <boost/interprocess/sync/scoped_lock.hpp>

/* Visual Leak Detector */
//#include <vld.h>

/* Windows */
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tchar.h>
//-------------------------------------------------------------------------------
#if defined(UNICODE) || defined(_UNICODE)
#define tcin  std::wcin
#define tcout std::wcout
#define tcerr std::wcerr

#define to_tstring std::to_wstring

using tstring =  std::wstring;
using tistream = std::wistream;
using tostream = std::wostream;
using tfstream = std::wfstream;
using tstringstream = std::wstringstream;

using tptree = pt::wptree;
#else
#define tcin  std::cin
#define tcout std::cout
#define tcerr std::cerr

#define to_tstring std::to_string

using tstring  = std::string;
using tistream = std::istream;
using tostream = std::ostream;
using tfstream = std::fstream;
using tstringstream = std::stringstream;

using tptree = pt::ptree;
#endif

//---defines---------------------------
#define   IN
#define   OUT

#include "Utils.h"
#include "verbose.h"
#include "Point.h"
#include "PointAdapter.h"
#include "resource.h"
//-------------------------------------
inline Point::value_type boost_distance(const Point &a, const Point &b)
{ return bg::distance(a, b); }
//-------------------------------------
#endif // _HEADER_H_
