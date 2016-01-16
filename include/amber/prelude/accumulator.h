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

#include <mutex>
#include <vector>

#include <boost/operators.hpp>

namespace amber {
namespace prelude {

// TODO probably more sophisticated methods that avoid a precision problem would
// be possible
template <typename T>
class Accumulator
: private boost::addable<Accumulator<T>>
{
public:
  Accumulator(T&& initial);

  const T Sum() const;
  const T Mean() const;

  Accumulator<T>& operator+=(const Accumulator<T>& accumulator);
  Accumulator<T>& operator+=(Accumulator<T>&& accumulator);
  Accumulator<T>& operator+=(const T& value);
  Accumulator<T>& operator+=(T&& value);

private:
  using Mutex = std::recursive_mutex;
  using Lock  = std::lock_guard<Mutex>;

  mutable Mutex mutex_;
  std::size_t size_;
  std::vector<T> buffer_;
  T initial_;

  void Add(std::size_t pos, T&& value);
};



template <typename T>
Accumulator<T>::Accumulator(T&& initial)
: mutex_()
, size_(0)
, buffer_()
, initial_(std::move(initial))
{}

template <typename T>
const T
Accumulator<T>::Sum() const
{
  Lock lock(mutex_);

  auto sum = initial_;

  for (std::size_t i = 0; i < buffer_.size(); i++) {
    const auto mask = static_cast<std::size_t>(1) << i;
    if (size_ & mask) {
      sum += buffer_.at(i);
    }
  }

  return sum;
}

template <typename T>
const T
Accumulator<T>::Mean() const
{
  Lock lock(mutex_);

  return Sum() / size_;
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(const Accumulator<T>& accumulator)
{
  return *this += static_cast<Accumulator<T>>(accumulator);
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(Accumulator<T>&& accumulator)
{
  Lock lock0(&mutex_ < &accumulator.mutex_ ? mutex_ : accumulator.mutex_);
  Lock lock1(&mutex_ < &accumulator.mutex_ ? accumulator.mutex_ : mutex_);

  if (accumulator.size_ > size_) {
    std::swap(size_, accumulator.size_);
    std::swap(buffer_, accumulator.buffer_);
  }

  for (std::size_t i = 0; i < accumulator.buffer_.size(); i++) {
    const auto mask = static_cast<std::size_t>(1) << i;
    if (accumulator.size_ & mask) {
      Add(i, std::move(accumulator.buffer_[i]));
    }
  }

  accumulator.size_ = 0;
  accumulator.buffer_.clear();

  return *this;
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(const T& value)
{
  return *this += static_cast<T>(value);
}

template <typename T>
Accumulator<T>&
Accumulator<T>::operator+=(T&& value)
{
  Lock lock(mutex_);

  Add(0, std::move(value));

  return *this;
}

template <typename T>
void
Accumulator<T>::Add(std::size_t pos, T&& value)
{
  size_ += static_cast<std::size_t>(1) << pos;

  for (std::size_t i = pos;; i++) {
    const auto mask = static_cast<std::size_t>(1) << i;
    if (size_ & mask) {
      if (i == buffer_.size()) {
        buffer_.emplace_back(std::move(value));
      } else {
        buffer_[i] = std::move(value);
      }
      break;
    }

    value += std::move(buffer_[i]);
  }
}

}
}
