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

#pragma once

#include <mutex>
#include <vector>

#include "core/component/bidirectional_path_tracing.h"
#include "core/component/image_average_buffer.h"
#include "core/component/multiple_importance_sampling.h"
#include "core/shader.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class BidirectionalPathTracing : public Shader<Object>
{
private:
  using typename Shader<Object>::camera_type;
  using typename Shader<Object>::image_type;
  using typename Shader<Object>::scene_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using unit_vector3_type  = typename Object::unit_vector3_type;

  using path_buffer_type       = component::PathBuffer<radiant_type, real_type>;
  using path_contribution_type = component::PathContribution<radiant_type>;

public:
  BidirectionalPathTracing() noexcept {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "BidirectionalPathTracing()";
  }

  image_type
  operator()(
    scene_type const& scene,
    camera_type const& camera,
    Context& ctx
  )
  {
    component::ImageAverageBuffer<radiant_type>
      image(camera.ImageWidth(), camera.ImageHeight());
    {
      std::mutex mtx;

      IterateParallel(ctx, [&](auto const&){
        MTSampler sampler((std::random_device()()));
        path_buffer_type path_buffer;

        image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
        for (std::size_t y = 0; y < camera.ImageHeight(); y++) {
          for (std::size_t x = 0; x < camera.ImageWidth(); x++) {
            radiant_type measurement;
            std::vector<path_contribution_type> light_image;
            std::tie(measurement, light_image) = component::Combine(
              scene,
              camera,
              component::GenerateLightSubpath(scene, camera, sampler),
              component::GenerateEyeSubpath(scene, camera, x, y, sampler),
              path_buffer,
              component::PowerHeuristic<radiant_value_type>()
            );
            for (auto const& contribution : light_image) {
              auto const& x_light_image = std::get<0>(*contribution.pixel);
              auto const& y_light_image = std::get<1>(*contribution.pixel);
              image_buffer.at(x_light_image, y_light_image) +=
                contribution.measurement;
            }
            image_buffer.at(x, y) += measurement;
          }
        }

        std::lock_guard<std::mutex> lock(mtx);
        image.Buffer(std::move(image_buffer));
      });
    }

    return image;
  }
};

}
}
}
