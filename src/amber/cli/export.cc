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

#include <opencv2/opencv.hpp>

#include "cli/export.h"
#include "post_process/filmic.h"
#include "post_process/gamma.h"
#include "post_process/normalizer.h"

namespace amber {
namespace cli {

void
ExportEXR(
  core::Image<core::RGB<std::float_t>> const& image,
  std::string const& filename
)
{
  post_process::Normalizer<core::RGB<std::float_t>> normalizer;

  auto const hdr_image = normalizer(image);

  cv::Mat mat(hdr_image.height(), hdr_image.width(), CV_32FC3);
  for (std::size_t i = 0; i < hdr_image.height(); i++) {
    for (std::size_t j = 0; j < hdr_image.width(); j++) {
      auto& rgb = mat.at<cv::Vec3f>(i, j);
      rgb[0] = hdr_image.at(j, i).b();
      rgb[1] = hdr_image.at(j, i).g();
      rgb[2] = hdr_image.at(j, i).r();
    }
  }

  cv::imwrite(filename, mat);
}

void
ExportPNG(
  core::Image<core::RGB<std::float_t>> const& image,
  std::string const& filename,
  std::double_t const exposure
)
{
  post_process::Filmic<core::RGB<std::float_t>> filmic(exposure);
  post_process::Gamma<core::RGB<std::float_t>> gamma;

  auto const ldr_image = gamma(filmic(image));

  cv::Mat mat(ldr_image.height(), ldr_image.width(), CV_8UC3);
  for (std::size_t i = 0; i < ldr_image.height(); i++) {
    for (std::size_t j = 0; j < ldr_image.width(); j++) {
      auto& rgb = mat.at<cv::Vec3b>(i, j);
      rgb[0] = ldr_image.at(j, i).b();
      rgb[1] = ldr_image.at(j, i).g();
      rgb[2] = ldr_image.at(j, i).r();
    }
  }
  cv::imwrite(filename, mat);
}

}
}
