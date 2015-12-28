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

namespace amber {
namespace core {

template <typename T>
class Accumulator
{
private:
  std::size_t n_;
  std::vector<T> buffer_;

public:
  Accumulator() noexcept;

  operator T const() const noexcept;

  Accumulator<T>& operator+=(T const& value);
  Accumulator<T>& operator+=(T&& value);
};

template <typename T>
Accumulator<T>::Accumulator() noexcept
: n_(0)
, buffer_()
{}

template <typename T>
Accumulator<T>::operator T const() const noexcept
{
  T sum(0);

  for (std::size_t i = 0; i < buffer_.size(); i++) {
    if (n_ & (static_cast<std::size_t>(1) << i)) {
      sum += buffer_.at(i);
    }
  }

  return sum;
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(T const& value)
{
  auto copy = value;
  return operator+=(copy);
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(T&& value)
{
  n_++;

  for (std::size_t i = 0;; i++) {
    if (n_ & (static_cast<std::size_t>(1) << i)) {
      if (n_ == (n_ & -n_)) {
        buffer_.emplace_back(std::move(value));
      } else {
        buffer_.at(i) = std::move(value);
      }
      break;
    }

    value += buffer_.at(i);
  }

  return *this;
}

}
}
