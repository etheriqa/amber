#pragma once

#include "material/lambertian.h"
#include "material/light.h"
#include "material/phong.h"
#include "material/refraction.h"
#include "material/specular.h"
#include "shape/convex_polygon.h"
#include "shape/sphere.h"
#include "vector.h"

namespace amber {
namespace scene {

template <typename Acceleration>
typename Acceleration::object_buffer_type cornel_box()
{
  using RealType      = typename Acceleration::object_type::shape_type::real_type;

  using Vector3       = Vector3<RealType>;

  using ConvexPolygon = shape::ConvexPolygon<RealType>;
  using Sphere        = shape::Sphere<RealType>;

  using RGB           = typename Acceleration::object_type::material_type::flux_type;

  using Lambertian    = material::Lambertian<RGB>;
  using Light         = material::Light<RGB>;
  using Phong         = material::Phong<RGB>;
  using Refraction    = material::Refraction<RGB>;
  using Specular      = material::Specular<RGB>;

  using ObjectBuffer  = typename Acceleration::object_buffer_type;

  ObjectBuffer objects;

  // light source
  objects.emplace_back(
    new ConvexPolygon({
      Vector3( 0.25, 0.99,  0.25),
      Vector3(-0.25, 0.99,  0.25),
      Vector3(-0.25, 0.99, -0.25),
      Vector3( 0.25, 0.99, -0.25),
    }),
    new Light(RGB(10, 10, 10))
  );
  // left
  objects.emplace_back(
    new ConvexPolygon({
      Vector3(-1,  1,  1),
      Vector3(-1, -1,  1),
      Vector3(-1, -1, -1),
      Vector3(-1,  1, -1),
    }),
    new Lambertian(RGB(.5, .0, .0))
  );
  // right
  objects.emplace_back(
    new ConvexPolygon({
      Vector3(1,  1,  1),
      Vector3(1,  1, -1),
      Vector3(1, -1, -1),
      Vector3(1, -1,  1),
    }),
    new Lambertian(RGB(.0, .5, .0))
  );
  // back
  objects.emplace_back(
    new ConvexPolygon({
      Vector3( 1,  1, -1),
      Vector3(-1,  1, -1),
      Vector3(-1, -1, -1),
      Vector3( 1, -1, -1),
    }),
    new Lambertian(RGB(.5, .5, .5))
  );
  // floor
  objects.emplace_back(
    new ConvexPolygon({
      Vector3( 1, -1,  1),
      Vector3( 1, -1, -1),
      Vector3(-1, -1, -1),
      Vector3(-1, -1,  1),
    }),
    new Phong(RGB(.1, .1, .1), RGB(.5, .5, .5), 100)
  );
  // ceiling
  objects.emplace_back(
    new ConvexPolygon({
      Vector3( 1, 1,  1),
      Vector3(-1, 1,  1),
      Vector3(-1, 1, -1),
      Vector3( 1, 1, -1),
    }),
    new Lambertian(RGB(.5, .5, .5))
  );
  // diffuse sphere
  objects.emplace_back(new Sphere(Vector3( 0.4, -0.6, -0.5), 0.4), new Lambertian(RGB(.5, .5, .5)));
  // specular sphere
  objects.emplace_back(new Sphere(Vector3(-0.4, -0.7,  0.1), 0.3), new Specular(RGB(.95, .95, .95)));
  // refraction sphere
  objects.emplace_back(new Sphere(Vector3( 0.1, -0.8,  0.6), 0.2), new Refraction(1.5));

  return objects;
}

}
}
