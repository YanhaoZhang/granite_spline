/**
MIT License

This file is part of the Granite project which is based on Basalt.
https://github.com/DLR-RM/granite

Copyright (c) Martin Wudenka, Deutsches Zentrum für Luft- und Raumfahrt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
Original license of Basalt:
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DATASET_IO_EUROC_H
#define DATASET_IO_EUROC_H

#include <granite/io/dataset_io.h>
#include <granite/utils/common_types.h>
#include <granite/utils/filesystem.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace granite {

class EurocVioDataset : public VioDataset {
  size_t num_cams;

  std::string path;

  std::vector<int64_t> image_timestamps;
  std::unordered_map<int64_t, std::string> image_path;

  // vector of images for every timestamp
  // assumes vectors size is num_cams for every timestamp with null pointers for
  // missing frames
  // std::unordered_map<int64_t, std::vector<ImageData>> image_data;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d>
      gt_pose_data;  // TODO: change to eigen aligned

  Eigen::aligned_vector<Eigen::Vector3d> gt_velocities;

  int64_t mocap_to_imu_offset_ns = 0;

  std::vector<std::unordered_map<int64_t, double>> exposure_times;

 public:
  ~EurocVioDataset(){};

  size_t get_num_cams() const { return num_cams; }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }

  const Eigen::aligned_vector<AccelData> &get_accel_data() const {
    return accel_data;
  }
  const Eigen::aligned_vector<GyroData> &get_gyro_data() const {
    return gyro_data;
  }
  const std::vector<int64_t> &get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    return gt_pose_data;
  }
  const Eigen::aligned_vector<Eigen::Vector3d> &get_gt_velocities() const {
    return gt_velocities;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t t_ns) {
    std::vector<ImageData> res(num_cams);

    // const std::vector<std::string> folder = {"/mav0/cam0/", "/mav0/cam1/"};

    for (size_t i = 0; i < num_cams; i++) {
      std::string full_image_path =
          path + "/mav0/cam" + std::to_string(i) + "/data/" + image_path[t_ns];

      if (fs::exists(full_image_path)) {
        cv::Mat img = cv::imread(full_image_path, cv::IMREAD_UNCHANGED);


        //yh undistort image
//        cv::Mat distCoeffs = (cv::Mat_<double>(1,4) << -0.11106452180588008, 0.17858247476424663, -0.0006877836355554949, -0.00013122812418964706);
//        cv::Mat K = (cv::Mat_<double>(3,3) << 897.4764841120259, 0, 516.5816593053271, 0, 900.131314815222, 377.03405845062315, 0, 0, 1);
//        cv::Mat img;
//        cv::undistort(img_, img, K, distCoeffs);



        res[i].img.reset(
            new ManagedImage<PixelType>(img.cols, img.rows));

        PixelType *data_out = res[i].img->ptr;

        if (img.type() == CV_8UC1) {
          const uint8_t *data_in = img.ptr();
          convert_copy_gray_image(img.cols, img.rows, data_in, data_out);
        } else if (img.type() == CV_8UC3) {
          const uint8_t *data_in = img.ptr();
          convert_copy_color_image(img.cols, img.rows, data_in, data_out);
        } else if (img.type() == CV_16UC1) {
          const uint16_t *data_in =
              reinterpret_cast<const uint16_t *>(img.ptr());
          convert_copy_gray_image(img.cols, img.rows, data_in, data_out);
        } else {
          std::cerr << "img.fmt.bpp " << img.type() << std::endl;
          std::abort();
        }

        auto exp_it = exposure_times[i].find(t_ns);
        if (exp_it != exposure_times[i].end()) {
          res[i].exposure = exp_it->second;
        }
      } else {
        std::cerr << "image '" << full_image_path << "' does not exist"
                  << std::endl;
        std::abort();
      }
    }

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class EurocIO;
};

class EurocIO : public DatasetIoInterface {
 public:
  EurocIO(bool load_mocap_as_gt) : load_mocap_as_gt(load_mocap_as_gt) {}

  bool read(const std::string &path, size_t num_cams) {
    if (!fs::exists(path)) {
      std::cerr << "No dataset found in " << path << std::endl;
      return false;
    }

    std::cout << "reading Euroc data set in folder " << path << std::endl;
    std::cout << "num_cams = " << num_cams << std::endl;

    data.reset(new EurocVioDataset);

    data->num_cams = num_cams;
    data->path = path;

    read_image_timestamps(path + "/mav0/cam0/");

    read_imu_data(path + "/mav0/imu0/");

    if (!load_mocap_as_gt &&
        fs::exists(path + "/mav0/state_groundtruth_estimate0/data.csv")) {
      read_gt_data_state(path + "/mav0/state_groundtruth_estimate0/");
    } else if (!load_mocap_as_gt && fs::exists(path + "/mav0/gt/data.csv")) {
      read_gt_data_pose(path + "/mav0/gt/");
    } else if (fs::exists(path + "/mav0/mocap0/data.csv")) {
      read_gt_data_pose(path + "/mav0/mocap0/");
    }

    data->exposure_times.resize(data->num_cams);
    if (fs::exists(path + "/mav0/cam0/exposure.csv")) {
      std::cout << "Loading exposure times for cam0" << std::endl;
      read_exposure(path + "/mav0/cam0/", data->exposure_times[0]);
    }
    if (fs::exists(path + "/mav0/cam1/exposure.csv")) {
      std::cout << "Loading exposure times for cam1" << std::endl;
      read_exposure(path + "/mav0/cam1/", data->exposure_times[1]);
    }

    return true;
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  void read_exposure(const std::string &path,
                     std::unordered_map<int64_t, double> &exposure_data) {
    exposure_data.clear();

    std::ifstream f(path + "exposure.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      int64_t timestamp, exposure_int;
      Eigen::Vector3d gyro, accel;

      ss >> timestamp >> tmp >> exposure_int;

      exposure_data[timestamp] = exposure_int * 1e-9;
    }
  }

  void read_image_timestamps(const std::string &path) {
//    std::ifstream f(path + "data.csv");
    std::ifstream f(path + "data.csv");   //yh just for debug
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;
      std::stringstream ss(line);
      char tmp;
      int64_t t_ns;
      std::string path;
      ss >> t_ns >> tmp >> path;

      data->image_timestamps.emplace_back(t_ns);
      data->image_path[t_ns] = path;
    }
  }

  void read_imu_data(const std::string &path) {
    data->accel_data.clear();
    data->gyro_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Vector3d gyro, accel;

      ss >> timestamp >> tmp >> gyro[0] >> tmp >> gyro[1] >> tmp >> gyro[2] >>
          tmp >> accel[0] >> tmp >> accel[1] >> tmp >> accel[2];

      data->accel_data.emplace_back();
      data->accel_data.back().timestamp_ns = timestamp;
      data->accel_data.back().data = accel;

      data->gyro_data.emplace_back();
      data->gyro_data.back().timestamp_ns = timestamp;
      data->gyro_data.back().data = gyro;
    }
  }

  void read_gt_data_state(const std::string &path) {
    data->gt_timestamps.clear();
    data->gt_pose_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Quaterniond q;
      Eigen::Vector3d pos, vel, accel_bias, gyro_bias;

      ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
          tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z() >> tmp >>
          vel[0] >> tmp >> vel[1] >> tmp >> vel[2] >> tmp >> accel_bias[0] >>
          tmp >> accel_bias[1] >> tmp >> accel_bias[2] >> tmp >> gyro_bias[0] >>
          tmp >> gyro_bias[1] >> tmp >> gyro_bias[2];

      data->gt_timestamps.emplace_back(timestamp);
      data->gt_pose_data.emplace_back(q, pos);
      data->gt_velocities.emplace_back(vel);
    }
  }

  void read_gt_data_pose(const std::string &path) {
    data->gt_timestamps.clear();
    data->gt_pose_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Quaterniond q;
      Eigen::Vector3d pos;

      ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
          tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z();

      data->gt_timestamps.emplace_back(timestamp);
      data->gt_pose_data.emplace_back(q, pos);
    }
  }

  std::shared_ptr<EurocVioDataset> data;
  bool load_mocap_as_gt;
};  // namespace granite

}  // namespace granite

#endif  // DATASET_IO_H
