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

#include "writer.h"

namespace amber {

template <typename T>
class RGB : public Writer
{
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  RGB() noexcept : RGB(T()) {}
  explicit RGB(const T& value) noexcept { values_.fill(value); }
  RGB(const T& r, const T& g, const T& b) noexcept : values_({{r, g, b}}) {}

  T& r() noexcept { return values_[0]; }
  const T& r() const noexcept { return values_[0]; }
  T& g() noexcept { return values_[1]; }
  const T& g() const noexcept { return values_[1]; }
  T& b() noexcept { return values_[2]; }
  const T& b() const noexcept { return values_[2]; }

  void Write(std::ostream& os) const noexcept {
    os << "RGB(" << r() << " " << g() << " " << b() << ")";
  }

  T Min() const noexcept { return std::min({r(), g(), b()}); }
  T Max() const noexcept { return std::max({r(), g(), b()}); }
  T Sum() const noexcept { return r() + g() + b(); }
  T Mean() const noexcept { return Sum() / 3; }

  template <typename U>
  RGB<T>& operator+=(const U& u) noexcept { return *this = *this + u; }

  template <typename U>
  RGB<T>& operator-=(const U& u) noexcept { return *this = *this - u; }

  template <typename U>
  RGB<T>& operator*=(const U& u) noexcept { return *this = *this * u; }

  template <typename U>
  RGB<T>& operator/=(const U& u) noexcept { return *this = *this / u; }
};

template <typename T>
RGB<T> operator+(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() + y.r(), x.g() + y.g(), x.b() + y.b());
}

template <typename T, typename U>
RGB<T> operator+(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() + k, x.g() + k, x.b() + k);
}

template <typename T>
RGB<T> operator-(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() - y.r(), x.g() - y.g(), x.b() - y.b());
}

template <typename T, typename U>
RGB<T> operator-(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() - k, x.g() - k, x.b() - k);
}

template <typename T>
RGB<T> operator*(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() * y.r(), x.g() * y.g(), x.b() * y.b());
}

template <typename T, typename U>
RGB<T> operator*(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() * k, x.g() * k, x.b() * k);
}

template <typename T>
RGB<T> operator/(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() / y.r(), x.g() / y.g(), x.b() / y.b());
}

template <typename T, typename U>
RGB<T> operator/(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() / k, x.g() / k, x.b() / k);
}

}
