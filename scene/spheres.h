#pragma once

#include <cmath>
#include "constant.h"
#include "material/lambertian.h"
#include "material/light.h"
#include "material/refraction.h"
#include "shape/circle.h"
#include "shape/cylinder.h"
#include "shape/sphere.h"
#include "vector.h"

namespace amber {
namespace scene {

template <class Acceleration>
typename Acceleration::object_buffer_type spheres()
{
  using RealType     = typename Acceleration::object_type::shape_type::real_type;

  using Vector3      = Vector3<RealType>;

  using Circle       = shape::Circle<RealType>;
  using Cylinder     = shape::Cylinder<RealType>;
  using Sphere       = shape::Sphere<RealType>;

  using RGB          = typename Acceleration::object_type::material_type::flux_type;

  using Lambertian   = material::Lambertian<RGB>;
  using Light        = material::Light<RGB>;
  using Refraction   = material::Refraction<RGB>;

  using ObjectBuffer = typename Acceleration::object_type;

  ObjectBuffer objects;

  const auto a = Vector3(std::sin((0. * 2 / 3) * kPI), -1, std::cos((0. * 2 / 3) * kPI));
  const auto b = Vector3(std::sin((1. * 2 / 3) * kPI), -1, std::cos((1. * 2 / 3) * kPI));
  const auto c = Vector3(std::sin((2. * 2 / 3) * kPI), -1, std::cos((2. * 2 / 3) * kPI));

  // floor
  objects.emplace_back(
    new Circle({
      Vector3(0, -1, 0),
      Vector3(0,  1, 0),
      5
    }),
    new Lambertian(RGB(.5, .5, .5))
  );

  // lights
  objects.emplace_back(
    new Cylinder(a + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.emplace_back(
    new Cylinder(b + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.emplace_back(
    new Cylinder(c + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.emplace_back(
    new Circle(a + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25),
    new Light(RGB(1, 0, 0))
  );
  objects.emplace_back(
    new Circle(b + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25),
    new Light(RGB(0, 1, 0))
  );
  objects.emplace_back(
    new Circle(c + Vector3(0, 4, 0), Vector3(0, -1, 0), 0.25),
    new Light(RGB(0, 0, 1))
  );

  // spheres
  const auto radius = 0.25;
  const auto material = new Refraction(1.5);
  objects.emplace_back(
    new Sphere(a + Vector3(0, radius, 0), radius),
    material
  );
  objects.emplace_back(
    new Sphere(b + Vector3(0, radius, 0), radius),
    material
  );
  objects.emplace_back(
    new Sphere(c + Vector3(0, radius, 0), radius),
    material
  );

  return objects;
}

}
}
