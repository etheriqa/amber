/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera/aperture/aperture.h"
#include "camera/lens/lens.h"

namespace amber {
namespace camera {
namespace lens {

template <typename RealType>
class Thin : public Lens<RealType> {
public:
  using ray_type      = typename Lens<RealType>::ray_type;
  using vector3_type  = typename Lens<RealType>::vector3_type;

  using aperture_type = aperture::Aperture<RealType>;

private:
  aperture_type* aperture_;
  RealType focal_length_, target_distance_, sensor_distance_, magnifier_;

public:
  Thin(aperture_type* aperture, RealType target_distance) noexcept
    : Thin(aperture, target_distance, Lens<RealType>::kFocalLength) {}

  Thin(aperture_type* aperture,
       RealType target_distance,
       RealType focal_length) noexcept
    : aperture_(aperture),
      focal_length_(focal_length),
      target_distance_(target_distance),
      sensor_distance_(1 / (1 / focal_length_ - 1 / target_distance_)),
      magnifier_(target_distance_ / sensor_distance_) {}

  void write(std::ostream& os) const noexcept {
    os
      << "Thin(focal_length=" << focal_length_
      << ", target_distance=" << target_distance_
      << ", sensor_distance=" << sensor_distance_
      << ")" << std::endl
      << "Aperture: " << *aperture_;
  }

  ray_type sampleRay(vector3_type const& sensor_point,
                      Sampler* sampler) const {
    const auto lens_point = aperture_->samplePoint(sampler);
    const auto focal_point =
      (vector3_type(0, 0, -sensor_distance_) - sensor_point) * magnifier_;
    return ray_type(lens_point, focal_point - lens_point);
  }
};

}
}
}
