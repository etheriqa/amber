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

#include "core/material/lambertian.h"
#include "core/material/light.h"
#include "core/material/phong.h"
#include "core/material/refraction.h"
#include "core/material/specular.h"
#include "core/primitive/convex_polygon.h"
#include "core/primitive/sphere.h"
#include "scene/cornel_box_complex.h"

namespace amber {
namespace scene {

std::vector<core::Object<core::RGB<std::float_t>, std::double_t>>
CornelBoxComplex()
{
  using real_type    = std::double_t;
  using radiant_type = core::RGB<std::float_t>;

  using object_type  = core::Object<radiant_type, real_type>;
  using vector3_type = core::Vector3<real_type>;

  using ConvexPolygon = core::primitive::ConvexPolygon<real_type>;
  using Sphere        = core::primitive::Sphere<real_type>;

  using Lambertian = core::material::Lambertian<radiant_type, real_type>;
  using Light      = core::material::Light<radiant_type, real_type>;
  using Phong      = core::material::Phong<radiant_type, real_type>;
  using Refraction = core::material::Refraction<radiant_type, real_type>;
  using Specular   = core::material::Specular<radiant_type, real_type>;

  std::vector<object_type> objects;

  // light source
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 0.001, 1,  0.001),
      vector3_type( 0.001, 1, -0.001),
      vector3_type(-0.001, 1, -0.001),
      vector3_type(-0.001, 1,  0.001),
    }),
    new Light(radiant_type(10, 10, 10))
  );
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 0.25, 3,  0.25),
      vector3_type( 0.25, 3, -0.25),
      vector3_type(-0.25, 3, -0.25),
      vector3_type(-0.25, 3,  0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // left
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type(-1,  1,  1),
      vector3_type(-1, -1,  1),
      vector3_type(-1, -1, -1),
      vector3_type(-1,  1, -1),
    }),
    new Lambertian(radiant_type(.5, .0, .0))
  );
  // right
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type(1,  1,  1),
      vector3_type(1,  1, -1),
      vector3_type(1, -1, -1),
      vector3_type(1, -1,  1),
    }),
    new Lambertian(radiant_type(.0, .5, .0))
  );
  // back
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 1,  1, -1),
      vector3_type(-1,  1, -1),
      vector3_type(-1, -1, -1),
      vector3_type( 1, -1, -1),
    }),
    new Phong(radiant_type(.1), radiant_type(.5), 256)
  );
  // floor
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 1, -1,  1),
      vector3_type( 1, -1, -1),
      vector3_type(-1, -1, -1),
      vector3_type(-1, -1,  1),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // ceiling
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 1.00, 1,  1.00),
      vector3_type( 1.00, 1, -1.00),
      vector3_type( 0.25, 1, -1.00),
      vector3_type( 0.25, 1,  1.00),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type(-1.00, 1,  1.00),
      vector3_type(-1.00, 1, -1.00),
      vector3_type(-0.25, 1, -1.00),
      vector3_type(-0.25, 1,  1.00),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 1.00, 1,  1.00),
      vector3_type(-1.00, 1,  1.00),
      vector3_type(-1.00, 1,  0.25),
      vector3_type( 1.00, 1,  0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  objects.emplace_back(
    new ConvexPolygon({
      vector3_type( 1.00, 1, -1.00),
      vector3_type(-1.00, 1, -1.00),
      vector3_type(-1.00, 1, -0.25),
      vector3_type( 1.00, 1, -0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // diffuse Sphere
  objects.emplace_back(
    new Sphere(vector3_type( 0.4, -0.6, -0.5), 0.4),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // specular Sphere
  objects.emplace_back(
    new Sphere(vector3_type(-0.4, -0.7,  0.1), 0.3),
    new Specular(radiant_type(.95, .95, .95))
  );
  // refraction Sphere
  objects.emplace_back(
    new Sphere(vector3_type( 0.1, -0.8,  0.6), 0.2),
    new Refraction(1.5)
  );

  return objects;
}

}
}