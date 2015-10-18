#include <iostream>
#include <thread>
#include "acceleration/kdtree.h"
#include "aperture/circle.h"
#include "aperture/polygon.h"
#include "camera.h"
#include "image.h"
#include "io/ppm.h"
#include "io/rgbe.h"
#include "lens/pinhole.h"
#include "lens/thin.h"
#include "object.h"
#include "post_process/gamma.h"
#include "post_process/reinhard.h"
#include "radiometry/rgb.h"
#include "render.h"
#include "scene/cornel_box.h"
#include "shader/bdpt.h"
#include "shader/pt.h"
#include "vector.h"

int main()
{
  std::cerr
    << R"(                   __             )" << std::endl
    << R"(  ____ _____ ___  / /_  ___  _____)" << std::endl
    << R"( / __ `/ __ `__ \/ __ \/ _ \/ ___/)" << std::endl
    << R"(/ /_/ / / / / / / /_/ /  __/ /    )" << std::endl
    << R"(\__,_/_/ /_/ /_/_.___/\___/_/     )" << std::endl
    << std::endl;

  using RealType = double;

  using Vector3 = amber::Vector3<RealType>;
  using Radiant = amber::radiometry::RGB<RealType>;

  using Primitive = amber::primitive::Primitive<RealType>;
  using Material = amber::material::Material<Radiant>;
  using Acceleration = amber::acceleration::KDTree<amber::Object<Primitive, Material>>;

  const auto n_thread = std::thread::hardware_concurrency();
  const auto ssaa_factor = 4;
  const auto pt_spp = 1024;
  const auto bpt_spp = 128;

  const auto scene = amber::scene::cornel_box<Acceleration>();
  //const auto shader = amber::shader::PathTracing<Acceleration>(n_thread, pt_spp / ssaa_factor / ssaa_factor);
  const auto shader = amber::shader::BidirectionalPathTracing<Acceleration>(n_thread, bpt_spp / ssaa_factor / ssaa_factor);
  const auto lens   = new amber::lens::Pinhole<RealType>();
  const auto image  = new amber::Image<Radiant>(512 * ssaa_factor, 512 * ssaa_factor);
  const auto sensor = new amber::Sensor<Radiant>(image);
  const auto camera = amber::Camera<Radiant>(lens, sensor, Vector3(0, 0, 4), Vector3(0, 0, 0), Vector3(0, 1, 0));

  amber::render(shader, scene, camera);

  amber::post_process::Reinhard<RealType> reinhard;
  amber::post_process::Gamma<RealType> gamma;
  amber::io::export_rgbe("output.hdr", image->down_sample(ssaa_factor));
  amber::io::export_ppm("output.ppm", gamma(reinhard(image->down_sample(ssaa_factor))));

  return 0;
}
