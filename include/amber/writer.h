/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <ostream>
#include <type_traits>

namespace amber {

class Writer
{
public:
  virtual void Write(std::ostream& os) const noexcept = 0;
};

template <
  typename W,
  typename = typename std::enable_if_t<std::is_base_of<Writer, W>::value>
>
std::ostream& operator<<(std::ostream& os, W const& w)
{
  w.Write(os);
  return os;
}

}
