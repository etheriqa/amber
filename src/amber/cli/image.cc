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

#include <opencv2/highgui/highgui.hpp>

#include "amber/cli/image.h"
#include "amber/prelude/image.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace cli {

HDRImage ImportEXR(const std::string& filename)
{
  const auto mat =
    cv::imread(filename, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

  HDRImage image(mat.rows, mat.cols);
  for (pixel_size_type j = 0; j < image.Height(); j++) {
    for (pixel_size_type i = 0; i < image.Width(); i++) {
      auto& rgb = mat.at<cv::Vec3f>(j, image.Width() - i - 1);
      image[Pixel(i, j)] = HDR(rgb[2], rgb[1], rgb[0]);
    }
  }
  return image;
}

void ExportEXR(const HDRImage& image, const std::string& filename)
{
  cv::Mat mat(image.Height(), image.Width(), CV_32FC3);
  for (pixel_size_type j = 0; j < image.Height(); j++) {
    for (pixel_size_type i = 0; i < image.Width(); i++) {
      auto& rgb = mat.at<cv::Vec3f>(j, image.Width() - i - 1);
      rgb[0] = image[Pixel(i, j)].Z();
      rgb[1] = image[Pixel(i, j)].Y();
      rgb[2] = image[Pixel(i, j)].X();
    }
  }
  cv::imwrite(filename, mat);
}

void ExportPNG(const LDRImage& image, const std::string& filename)
{
  cv::Mat mat(image.Height(), image.Width(), CV_8UC3);
  for (pixel_size_type j = 0; j < image.Height(); j++) {
    for (pixel_size_type i = 0; i < image.Width(); i++) {
      auto& rgb = mat.at<cv::Vec3b>(j, image.Width() - i - 1);
      rgb[0] = image[Pixel(i, j)].Z();
      rgb[1] = image[Pixel(i, j)].Y();
      rgb[2] = image[Pixel(i, j)].X();
    }
  }
  cv::imwrite(filename, mat);
}

}
}
