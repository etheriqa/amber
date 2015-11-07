/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "geometry/primitive/convex_polygon.h"
#include "geometry/primitive/sphere.h"
#include "material/lambertian.h"
#include "material/light.h"
#include "material/phong.h"
#include "material/refraction.h"
#include "material/specular.h"

namespace amber {
namespace scene {

template <typename OutputIterator,
          typename Object = typename OutputIterator::container_type::value_type>
void cornel_box_complex(OutputIterator output) {
  using radiant_type  = typename Object::radiant_type;
  using real_type     = typename Object::real_type;
  using vector3_type  = typename Object::vector3_type;

  using ConvexPolygon = geometry::primitive::ConvexPolygon<real_type>;
  using Sphere        = geometry::primitive::Sphere<real_type>;

  using Lambertian    = material::Lambertian<radiant_type, real_type>;
  using Light         = material::Light<radiant_type, real_type>;
  using Phong         = material::Phong<radiant_type, real_type>;
  using Refraction    = material::Refraction<radiant_type, real_type>;
  using Specular      = material::Specular<radiant_type, real_type>;

  // light source
  output = Object(
    new ConvexPolygon({
      vector3_type( 0.001, 1,  0.001),
      vector3_type( 0.001, 1, -0.001),
      vector3_type(-0.001, 1, -0.001),
      vector3_type(-0.001, 1,  0.001),
    }),
    new Light(radiant_type(10, 10, 10))
  );
  output = Object(
    new ConvexPolygon({
      vector3_type( 0.25, 3,  0.25),
      vector3_type( 0.25, 3, -0.25),
      vector3_type(-0.25, 3, -0.25),
      vector3_type(-0.25, 3,  0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // left
  output = Object(
    new ConvexPolygon({
      vector3_type(-1,  1,  1),
      vector3_type(-1, -1,  1),
      vector3_type(-1, -1, -1),
      vector3_type(-1,  1, -1),
    }),
    new Lambertian(radiant_type(.5, .0, .0))
  );
  // right
  output = Object(
    new ConvexPolygon({
      vector3_type(1,  1,  1),
      vector3_type(1,  1, -1),
      vector3_type(1, -1, -1),
      vector3_type(1, -1,  1),
    }),
    new Lambertian(radiant_type(.0, .5, .0))
  );
  // back
  output = Object(
    new ConvexPolygon({
      vector3_type( 1,  1, -1),
      vector3_type(-1,  1, -1),
      vector3_type(-1, -1, -1),
      vector3_type( 1, -1, -1),
    }),
    new Phong(radiant_type(.1), radiant_type(.5), 256)
  );
  // floor
  output = Object(
    new ConvexPolygon({
      vector3_type( 1, -1,  1),
      vector3_type( 1, -1, -1),
      vector3_type(-1, -1, -1),
      vector3_type(-1, -1,  1),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // ceiling
  output = Object(
    new ConvexPolygon({
      vector3_type( 1.00, 1,  1.00),
      vector3_type( 1.00, 1, -1.00),
      vector3_type( 0.25, 1, -1.00),
      vector3_type( 0.25, 1,  1.00),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  output = Object(
    new ConvexPolygon({
      vector3_type( 1.00, 1,  1.00),
      vector3_type( 1.00, 1, -1.00),
      vector3_type( 0.25, 1, -1.00),
      vector3_type( 0.25, 1,  1.00),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  output = Object(
    new ConvexPolygon({
      vector3_type(-1.00, 1,  1.00),
      vector3_type(-1.00, 1, -1.00),
      vector3_type(-0.25, 1, -1.00),
      vector3_type(-0.25, 1,  1.00),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  output = Object(
    new ConvexPolygon({
      vector3_type( 1.00, 1,  1.00),
      vector3_type(-1.00, 1,  1.00),
      vector3_type(-1.00, 1,  0.25),
      vector3_type( 1.00, 1,  0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  output = Object(
    new ConvexPolygon({
      vector3_type( 1.00, 1, -1.00),
      vector3_type(-1.00, 1, -1.00),
      vector3_type(-1.00, 1, -0.25),
      vector3_type( 1.00, 1, -0.25),
    }),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // diffuse sphere
  output = Object(
    new Sphere(vector3_type( 0.4, -0.6, -0.5), 0.4),
    new Lambertian(radiant_type(.5, .5, .5))
  );
  // specular sphere
  output = Object(
    new Sphere(vector3_type(-0.4, -0.7,  0.1), 0.3),
    new Specular(radiant_type(.95, .95, .95))
  );
  // refraction sphere
  output = Object(
    new Sphere(vector3_type( 0.1, -0.8,  0.6), 0.2),
    new Refraction(1.5)
  );
}

}
}
