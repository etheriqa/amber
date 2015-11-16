// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include <algorithm>
#include <array>

#include <boost/operators.hpp>

#include "core/writer.h"

namespace amber {
namespace core {

template <typename T>
class RGB
: public Writer,
  private boost::field_operators<RGB<T>>
{
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  RGB() noexcept : RGB(T()) {}
  RGB(const T& value) noexcept : RGB(value, value, value) {}
  RGB(const T& r, const T& g, const T& b) noexcept : values_({{r, g, b}}) {}

  T const& r() const noexcept { return values_[0]; }
  T const& g() const noexcept { return values_[1]; }
  T const& b() const noexcept { return values_[2]; }

  RGB<T>& operator+=(RGB<T> const& c) noexcept
  {
    return this->operator=(RGB<T>(r() + c.r(), g() + c.g(), b() + c.b()));
  }

  RGB<T>& operator-=(RGB<T> const& c) noexcept
  {
    return this->operator=(RGB<T>(r() - c.r(), g() - c.g(), b() - c.b()));
  }

  RGB<T>& operator*=(RGB<T> const& c) noexcept
  {
    return this->operator=(RGB<T>(r() * c.r(), g() * c.g(), b() * c.b()));
  }

  RGB<T>& operator/=(RGB<T> const& c) noexcept
  {
    return this->operator=(RGB<T>(r() / c.r(), g() / c.g(), b() / c.b()));
  }

  void Write(std::ostream& os) const noexcept
  {
    os << "RGB(" << r() << " " << g() << " " << b() << ")";
  }
};

template <typename T>
T const
Max(RGB<T> const& c) noexcept
{
  return std::max({c.r(), c.g(), c.b()});
}

template <typename T>
T const
Sum(RGB<T> const& c) noexcept
{
  return c.r() + c.g() + c.b();
}

}
}
