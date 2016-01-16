// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <boost/operators.hpp>

namespace amber {
namespace rendering {

/** Object pointer.
 */
class ObjectPointer
: private boost::equality_comparable<ObjectPointer>
{
public:
  /** Constructor.
   */
  template <typename Object>
  ObjectPointer(const Object* pointer) noexcept;

  /** Dereference.
   */
  template <typename Object>
  operator const Object&() const noexcept;

  /** Comparator.
   */
  bool operator==(const ObjectPointer& rhs) const noexcept;

private:
  const void* pointer_;
};





template <typename Object>
ObjectPointer::ObjectPointer(const Object* object) noexcept
: pointer_(object)
{}

template <typename Object>
ObjectPointer::operator const Object&() const noexcept
{
  return *static_cast<const Object*>(pointer_);
}

inline bool
ObjectPointer::operator==(const ObjectPointer& rhs) const noexcept
{
  return pointer_ == rhs.pointer_;
}

}
}
