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

#include <immintrin.h>

#include "amber/prelude/aabb.h"

namespace amber {
namespace prelude {

template <>
std::tuple<bool, std::float_t, std::float_t>
Intersect(
  const AABB<std::float_t>& bb,
  const Ray<std::float_t>& ray,
  std::float_t t_max
) noexcept
{
  std::float_t t_min = 0;

  const auto ray_origin =
    _mm_setr_ps(ray.origin.X(), ray.origin.Y(), ray.origin.Z(), 0);
  const auto inverse_ray_direction = _mm_div_ps(
    _mm_setr_ps(1, 1, 1, 1),
    _mm_setr_ps(ray.direction.X(), ray.direction.Y(), ray.direction.Z(), 1)
  );
  const auto aabb_min =
    _mm_setr_ps(bb.Min().X(), bb.Min().Y(), bb.Min().Z(), 0);
  const auto aabb_max =
    _mm_setr_ps(bb.Max().X(), bb.Max().Y(), bb.Max().Z(), 0);

  const auto t0 =
    _mm_mul_ps(_mm_sub_ps(aabb_min, ray_origin), inverse_ray_direction);
  const auto t1 =
    _mm_mul_ps(_mm_sub_ps(aabb_max, ray_origin), inverse_ray_direction);

  std::float_t t_mins[4], t_maxs[4];
  _mm_store_ps(t_mins, _mm_min_ps(t0, t1));
  _mm_store_ps(t_maxs, _mm_max_ps(t0, t1));

  t_min = std::max({t_min, t_mins[0], t_mins[1], t_mins[2]});
  t_max = std::min({t_max, t_maxs[0], t_maxs[1], t_maxs[2]});

  return std::make_tuple(t_min <= t_max, t_min, t_max);
}

template <>
std::tuple<bool, std::double_t, std::double_t>
Intersect(
  const AABB<std::double_t>& bb,
  const Ray<std::double_t>& ray,
  std::double_t t_max
) noexcept
{
  std::double_t t_min = 0;

  const auto ray_origin =
    _mm256_setr_pd(ray.origin.X(), ray.origin.Y(), ray.origin.Z(), 0);
  const auto inverse_ray_direction = _mm256_div_pd(
    _mm256_setr_pd(1, 1, 1, 1),
    _mm256_setr_pd(ray.direction.X(), ray.direction.Y(), ray.direction.Z(), 1)
  );
  const auto aabb_min =
    _mm256_setr_pd(bb.Min().X(), bb.Min().Y(), bb.Min().Z(), 0);
  const auto aabb_max =
    _mm256_setr_pd(bb.Max().X(), bb.Max().Y(), bb.Max().Z(), 0);

  const auto t0 =
    _mm256_mul_pd(_mm256_sub_pd(aabb_min, ray_origin), inverse_ray_direction);
  const auto t1 =
    _mm256_mul_pd(_mm256_sub_pd(aabb_max, ray_origin), inverse_ray_direction);

  std::double_t t_mins[4], t_maxs[4];
  _mm256_store_pd(t_mins, _mm256_min_pd(t0, t1));
  _mm256_store_pd(t_maxs, _mm256_max_pd(t0, t1));

  t_min = std::max({t_min, t_mins[0], t_mins[1], t_mins[2]});
  t_max = std::min({t_max, t_maxs[0], t_maxs[1], t_maxs[2]});

  return std::make_tuple(t_min <= t_max, t_min, t_max);
}

}
}
