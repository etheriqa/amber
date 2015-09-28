#pragma once

#include "material/lambertian.h"
#include "material/light.h"
#include "material/phong.h"
#include "material/refraction.h"
#include "material/specular.h"
#include "rgb.h"
#include "shape/convex_polygon.h"
#include "shape/sphere.h"
#include "vector.h"

namespace amber {
namespace scene {

template <class Container>
Container cornel_box()
{
  using Vector3 = Vector3<typename Container::real_type>;

  using ConvexPolygon = shape::ConvexPolygon<typename Container::real_type>;
  using Sphere = shape::Sphere<typename Container::real_type>;

  using RGB = RGB<typename Container::real_type>;

  using Lambertian = material::Lambertian<RGB>;
  using Light = material::Light<RGB>;
  using Phong = material::Phong<RGB>;
  using Refraction = material::Refraction<RGB>;
  using Specular = material::Specular<RGB>;

  using ObjectList = container::ObjectList<RGB>;

  ObjectList objects;

  // light source
  objects.insert(
    new ConvexPolygon({
      Vector3(0.25, 0.25, 0.99),
      Vector3(0.25, -0.25, 0.99),
      Vector3(-0.25, -0.25, 0.99),
      Vector3(-0.25, 0.25, 0.99),
    }),
    new Light(RGB(10, 10, 10))
  );
  // left
  objects.insert(
    new ConvexPolygon({
      Vector3(-1, 1, 1),
      Vector3(-1, -1, 1),
      Vector3(-1, -1, -1),
      Vector3(-1, 1, -1),
    }),
    new Lambertian(RGB(.5, .0, .0))
  );
  // right
  objects.insert(
    new ConvexPolygon({
      Vector3(1, 1, 1),
      Vector3(1, 1, -1),
      Vector3(1, -1, -1),
      Vector3(1, -1, 1),
    }),
    new Lambertian(RGB(.0, .5, .0))
  );
  // back
  objects.insert(
    new ConvexPolygon({
      Vector3(1, 1, 1),
      Vector3(-1, 1, 1),
      Vector3(-1, 1, -1),
      Vector3(1, 1, -1),
    }),
    new Lambertian(RGB(.5, .5, .5))
  );
  // floor
  objects.insert(
    new ConvexPolygon({
      Vector3(1, 1, -1),
      Vector3(-1, 1, -1),
      Vector3(-1, -1, -1),
      Vector3(1, -1, -1),
    }),
    new Phong(RGB(.1, .1, .1), RGB(.5, .5, .5), 100)
  );
  // ceiling
  objects.insert(
    new ConvexPolygon({
      Vector3(1, 1, 1),
      Vector3(1, -1, 1),
      Vector3(-1, -1, 1),
      Vector3(-1, 1, 1),
    }),
    new Lambertian(RGB(.5, .5, .5))
  );
  // diffuse sphere
  objects.insert(new Sphere(Vector3(0.4, 0.5, -0.6), 0.4), new Lambertian(RGB(.5, .5, .5)));
  // specular sphere
  objects.insert(new Sphere(Vector3(-0.4, -0.1, -0.7), 0.3), new Specular(RGB(.95, .95, .95)));
  // refraction sphere
  objects.insert(new Sphere(Vector3(0.1, -0.6, -0.8), 0.2), new Refraction(1.5));

  return Container(std::move(objects));
}

}
}
