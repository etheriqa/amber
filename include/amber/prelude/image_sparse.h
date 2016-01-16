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

#include <vector>

#include <boost/operators.hpp>

#include "amber/prelude/pixel.h"

namespace amber {
namespace prelude {

template <typename T>
class SparseImage
: private boost::multiplicative2<SparseImage<T>, T>
{
public:
  using Iterator      = typename std::vector<PixelValue<T>>::iterator;
  using ConstIterator = typename std::vector<PixelValue<T>>::const_iterator;

  SparseImage() noexcept;

  Iterator begin() noexcept;
  ConstIterator begin() const noexcept;
  Iterator end() noexcept;
  ConstIterator end() const noexcept;

  SparseImage<T>& operator*=(const T& multiplier) noexcept;
  SparseImage<T>& operator/=(const T& divisor) noexcept;

  template <typename... Args> void Emplace(Args&&... args);
  void Clear() noexcept;

private:
  std::vector<PixelValue<T>> values_;
};



template <typename T>
SparseImage<T>::SparseImage() noexcept
: values_()
{}

template <typename T>
auto
SparseImage<T>::begin() noexcept
-> Iterator
{
  return values_.begin();
}

template <typename T>
auto
SparseImage<T>::begin() const noexcept
-> ConstIterator
{
  return values_.begin();
}

template <typename T>
auto
SparseImage<T>::end() noexcept
-> Iterator
{
  return values_.end();
}

template <typename T>
auto
SparseImage<T>::end() const noexcept
-> ConstIterator
{
  return values_.end();
}

template <typename T>
SparseImage<T>&
SparseImage<T>::operator*=(const T& multiplier) noexcept
{
  for (auto& value : values_) {
    value *= multiplier;
  }
  return *this;
}

template <typename T>
SparseImage<T>&
SparseImage<T>::operator/=(const T& divisor) noexcept
{
  for (auto& value : values_) {
    value /= divisor;
  }
  return *this;
}

template <typename T>
template <typename... Args>
void
SparseImage<T>::Emplace(Args&&... args)
{
  values_.emplace_back(args...);
}

template <typename T>
void
SparseImage<T>::Clear() noexcept
{
  values_.clear();
}

}
}
