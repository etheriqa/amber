#include <iostream>
#include <thread>
#include "aperture/circle.h"
#include "aperture/polygon.h"
#include "camera.h"
#include "container/list.h"
#include "lens/pinhole.h"
#include "lens/thin.h"
#include "render.h"
#include "rgb.h"
#include "scene/cornel_box.h"
#include "scene/spheres.h"
#include "shader/bpt.h"
#include "shader/pt.h"
#include "tonemap/reinhard.h"
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
  using Spectrum = amber::RGB<RealType>;

  using Scene = amber::container::List<Spectrum>;

  const auto ssaa_factor = 4;
  const auto pt_spp = 1024;
  const auto bpt_spp = 128;

  const auto scene = amber::scene::cornel_box<Scene>();
  //const auto scene  = amber::scene::spheres<Scene>();
  //const auto shader = amber::shader::PathTracing<Scene>(std::thread::hardware_concurrency(), pt_spp / ssaa_factor / ssaa_factor);
  const auto shader = amber::shader::BidirectionalPathTracing<Scene>(std::thread::hardware_concurrency(), bpt_spp / ssaa_factor / ssaa_factor);
  const auto lens   = new amber::lens::Pinhole<RealType>();
  const auto image  = new amber::Image<Spectrum>(512 * ssaa_factor, 512 * ssaa_factor);
  const auto sensor = new amber::Sensor<Spectrum>(image);
  const auto camera = amber::Camera<Spectrum>(lens, sensor, Vector3(0, -4, 0), Vector3(0, 0, 0), Vector3(0, 0, 1));

  amber::render(shader, scene, camera);

  amber::tonemap::Reinhard<RealType> tonemapper;
  amber::save_ppm("output.ppm", tonemapper(image->down_sample(ssaa_factor)));

  return 0;
}
