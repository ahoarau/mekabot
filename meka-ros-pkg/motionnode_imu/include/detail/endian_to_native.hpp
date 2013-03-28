/**
  @file    tools/sdk/cpp/detail/endian_to_native.hpp
  @author  Luke Tokheim, luke@motionnode.com
  @version 1.2

  (C) Copyright GLI Interactive LLC 2009. All rights reserved.

  The coded instructions, statements, computer programs, and/or related
  material (collectively the "Data") in these files contain unpublished
  information proprietary to GLI Interactive LLC, which is protected by
  US federal copyright law and by international treaties.

  The Data may not be disclosed or distributed to third parties, in whole
  or in part, without the prior written consent of GLI Interactive LLC.

  The Data is provided "as is" without express or implied warranty, and
  with no claim as to its suitability for any purpose.
*/
#ifndef __MOTION_NODE_SDK_ENDIAN_HPP_
#define __MOTION_NODE_SDK_ENDIAN_HPP_

#include "endian.hpp"

#include <algorithm>

#ifdef BOOST_BIG_ENDIAN
#  define MOTION_NODE_SDK_BIG_ENDIAN 1
#else
#  ifndef BOOST_LITTLE_ENDIAN
#    error Unsupported endian platform
#  endif
#endif


namespace MotionNode { namespace SDK { namespace detail {

/**
  Generic byte swapping function for little-endian data to native type.
  This is certainly not the fastest way to handle byte-swapping, but
  consider it to be a fall-back implementation with minimal dependencies.

  Example usage:
  @code
  std::vector<int> buffer;
  
  // ... Read some binary data into the int buffer ...

  // Element-wise transformation from little-endian to native byte ordering.
  std::transform(
    buffer.begin(), buffer.end(),
    buffer.begin(), &detail::little_endian_to_native<int>);
  @endcode
*/
template <typename T>
inline T little_endian_to_native(T & value)
{
#ifdef MOTION_NODE_SDK_BIG_ENDIAN
  T result = T();

  std::reverse_copy(
    reinterpret_cast<const char *>(&value), reinterpret_cast<const char *>(&value) + sizeof(T),
    reinterpret_cast<char *>(&result));

  return result;
#else
  return value;
#endif // MOTION_NODE_SDK_BIG_ENDIAN
}

}}} // namespace MotionNode::SDK::detail

#endif // __MOTION_NODE_SDK_ENDIAN_HPP_
