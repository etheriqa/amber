/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/sampler.h"
#include "base/writer.h"

namespace amber {
namespace camera {
namespace aperture {

template <typename RealType>
class Aperture : public Writer {
public:
  using vector3_type  = geometry::Vector3<RealType>;

  virtual vector3_type samplePoint(Sampler*) const = 0;
};

}
}
}
