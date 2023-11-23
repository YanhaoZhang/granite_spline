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

#pragma once

#include <thread>

#include <sophus/se2.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <granite/optical_flow/optical_flow.h>
#include <granite/optical_flow/patch.h>

#include <granite/image/image_pyr.h>
#include <granite/utils/keypoints.h>

namespace granite {

//
template <typename Scalar, template <typename> typename Pattern>
class FrameToFrameOpticalFlow : public OpticalFlowBase {
 public:
  typedef OpticalFlowPatch<Scalar, Pattern<Scalar>> PatchT;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;

  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

  typedef Sophus::SE2<Scalar> SE2;

  FrameToFrameOpticalFlow(const VioConfig& config,
                          const granite::Calibration<double>& calib)
      : t_ns(-1), frame_counter(0), last_keypoint_id(0), config(config) {


    input_queue.set_capacity(10);   //yh set input queue size

    this->calib = calib.cast<Scalar>();  //yh intrinsic

    patch_coord = PatchT::pattern2.template cast<float>();

    //yh for stereo (not my case)
    for (const auto& stereo_pair : calib.stereo_pairs) {
      Eigen::Matrix4d Ed;
      Sophus::SE3d T_i_j = calib.T_i_c[stereo_pair.first].inverse() *
                           calib.T_i_c[stereo_pair.second];
      computeEssential(T_i_j, Ed);
      this->E.emplace(stereo_pair, Ed.cast<Scalar>());
    }

    output_queue.set_capacity(10);

    //yh start processing ptr
    processing_thread.reset(
        new std::thread(&FrameToFrameOpticalFlow::processingLoop, this));
  }

  ~FrameToFrameOpticalFlow() { join(); }

  void processingLoop() {
    OpticalFlowInput::Ptr input_ptr;

    while (!should_quit) {
      try {
        input_queue.pop(input_ptr);       //yh get image from the input_queue. the data is put in vio.cpp->feedImagesLoop()
      } catch (const tbb::user_abort&) {
        input_ptr = nullptr;
      };

      if (!input_ptr.get()) {
        try {
          output_queue.push(nullptr);
        } catch (const tbb::user_abort&) {
        };

        break;
      }

      //yh if the retrived input_ptr is not empty, calculate optical flow
      bool cont = processFrame(input_ptr->t_ns, input_ptr);   //yh main function to calculate optical flow

      if (!cont) {
        break;
      }
    }

    quit();

    std::cout << "Finished OpticalFlow" << std::endl;
  }

   //yh https://blog.csdn.net/qq_39266065/article/details/106175701
  bool processFrame(int64_t curr_t_ns, OpticalFlowInput::Ptr& new_img_vec) {
    //yh steps:
    // init: calculate FAST feat on image, store pts in transforms->observations
    // next frame: calculate forward and backward optical flow for feature matching
    //              just solve a SE2 for each patch (each extracted keypts on previous frame)
    // after matching: store matched pts to transforms->observations
    // add new pts on current frame


    //yh directly return if the image is empty
    for (const auto& v : new_img_vec->img_data) {
      if (!v.img.get()) {
        std::cout << "Image for " << curr_t_ns << " not present!" << std::endl;
        return true;
      }
    }

    //yh initialize
    if (t_ns < 0) {
      t_ns = curr_t_ns;

      //yh step1: init feature coordinate container
      transforms.reset(new OpticalFlowResult);                     //yh init container
      transforms->observations.resize(calib.intrinsics.size());    // set size
      transforms->pyramid_levels.resize(calib.intrinsics.size());  // new for granite
      transforms->t_ns = t_ns;

      //yh step2: initialize pyramid
      pyramid.reset(new std::vector<granite::ManagedImagePyr<PixelType>>); //yh init container
      pyramid->resize(calib.intrinsics.size());                            // num of camera (=1 for my case)

      //yh multi-ptr to construct pyramid
      tbb::parallel_for(tbb::blocked_range<size_t>(0, calib.intrinsics.size()),
                        [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = r.begin(); i != r.end(); ++i) {
                            pyramid->at(i).setFromImage(
                                *new_img_vec->img_data[i].img,
                                config.optical_flow_levels);
                          }
                        });

      //yh step3: to visualize
      transforms->input_images = new_img_vec;

      //yh step4: add now pts
      addPoints();

      //yh step5: remove outliers by epipolar geometry (stereo only)
      filterPoints();
    } else {
      //yh normal tracking.
      t_ns = curr_t_ns;

      old_pyramid = pyramid;

      //yh the new pyramid is by current image, the old pyramid is by previous image
      // (calculated last time when call this fucntion)
      pyramid.reset(new std::vector<granite::ManagedImagePyr<PixelType>>);
      pyramid->resize(calib.intrinsics.size());
      tbb::parallel_for(tbb::blocked_range<size_t>(0, calib.intrinsics.size()),
                        [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = r.begin(); i != r.end(); ++i) {
                            pyramid->at(i).setFromImage(
                                *new_img_vec->img_data[i].img,
                                config.optical_flow_levels);
                          }
                        });

      OpticalFlowResult::Ptr new_transforms;
      new_transforms.reset(new OpticalFlowResult);
      new_transforms->observations.resize(calib.intrinsics.size());
      new_transforms->pyramid_levels.resize(calib.intrinsics.size());
      new_transforms->t_ns = t_ns;

      //yh transforms->observations[i] stores the previous extracted pts
      for (size_t i = 0; i < calib.intrinsics.size(); i++) {
        trackPoints(old_pyramid->at(i), pyramid->at(i),
                    transforms->observations[i], transforms->pyramid_levels[i],
                    new_transforms->observations[i],
                    new_transforms->pyramid_levels[i]);
      }

      // std::cout << t_ns << ": Could track "
      //           << new_transforms->observations.at(0).size() << " points."
      //           << std::endl;

      transforms = new_transforms;
      transforms->input_images = new_img_vec;

      addPoints();
      filterPoints();
    }

    //yh output the tracking result into output_queue
    if (frame_counter % config.optical_flow_skip_frames == 0) {
      try {
        output_queue.push(transforms);
      } catch (const tbb::user_abort&) {
        return false;
      };
    }

    //yh
    frame_counter++;
    return true;
  }

  //yh frame to frame tracking
  void trackPoints(
      const granite::ManagedImagePyr<PixelType>& pyr_1,
      const granite::ManagedImagePyr<PixelType>& pyr_2,
      const Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>&
          transform_map_1,
      const std::map<KeypointId, size_t>& pyramid_levels_1,
      Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>& transform_map_2,
      std::map<KeypointId, size_t>& pyramid_levels_2) const {
    size_t num_points = transform_map_1.size();

    std::vector<KeypointId> ids;
    Eigen::aligned_vector<Eigen::AffineCompact2f> init_vec;
    std::vector<size_t> pyramid_level;

    ids.reserve(num_points);
    init_vec.reserve(num_points);
    pyramid_level.reserve(num_points);

    //yh map->vector (for forward tracking, transform_map_1 stores the existing pts by previous frame)
    for (const auto& kv : transform_map_1) {
      ids.push_back(kv.first);
      init_vec.push_back(kv.second);
      pyramid_level.push_back(pyramid_levels_1.at(kv.first));
    }



    // pure debug
//    Eigen::AffineCompact2f transform = init_vec[100];
//    std::cout<<"before: "<<transform.matrix()<<std::endl;
//    transform.linear().setIdentity();
//    std::cout<<"after: "<<transform.matrix()<<std::endl;
//    std::cout<<"set 0: "<<transform.linear()<<std::endl;
//    std::cout<<"set: "<<transform.linear().setIdentity()<<std::endl;


    //yh container to save rst
    tbb::concurrent_unordered_map<KeypointId, Eigen::AffineCompact2f,
                                  std::hash<KeypointId>>
        result_transforms;
    tbb::concurrent_unordered_map<KeypointId, size_t, std::hash<KeypointId>>
        result_pyramid_level;

    //yh lambda function to define a ptr
    auto compute_func = [&](const tbb::blocked_range<size_t>& range) {
      //yh loop over all keypoints
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const KeypointId id = ids[r];

        //yh feature's coordinate in the reference frame transform_1
        const Eigen::AffineCompact2f& transform_1 = init_vec[r];
        Eigen::AffineCompact2f transform_2 = transform_1;  //yh initialize current coordinate transform_2

        //yh forward optical flow tracking (for each pts, we calculate a SE2?)
        bool valid = trackPoint(pyr_1, pyr_2, transform_1, pyramid_level[r],
                                transform_2);
        //yh if forward optical flow valid, conduct backward optical flow tracking
        if (valid) {
          Eigen::AffineCompact2f transform_1_recovered = transform_2;

          //yh patch to patch tracking
          valid = trackPoint(pyr_2, pyr_1, transform_2, pyramid_level[r],
                             transform_1_recovered);

          if (valid) {
            const Scalar scale = 1 << pyramid_level[r];
            Scalar dist2 = (transform_1.translation() / scale -
                            transform_1_recovered.translation() / scale)
                               .squaredNorm();

            //yh store rst if the distance between backward tracking and
            // the original coordinate is smaller than optical_flow_max_recovered_dist2
            // this is where the correspondence is checked!!!
            if (dist2 < config.optical_flow_max_recovered_dist2) {
              result_transforms[id] = transform_2;
              result_pyramid_level[id] = pyramid_level[r];
            }
          }
        }
      }
    };


    tbb::blocked_range<size_t> range(0, num_points);  //yh feature range

    //yh multi-ptr to run tracking features
    tbb::parallel_for(range, compute_func);  //yh tbb parallel_for(range and fun)
    // compute_func(range);

    transform_map_2.clear();
    transform_map_2.insert(result_transforms.begin(), result_transforms.end());
    pyramid_levels_2.clear();
    pyramid_levels_2.insert(result_pyramid_level.begin(),
                            result_pyramid_level.end());
  }

  //yh track points between patches
  inline bool trackPoint(const granite::ManagedImagePyr<PixelType>& old_pyr,
                         const granite::ManagedImagePyr<PixelType>& pyr,
                         const Eigen::AffineCompact2f& old_transform,
                         const size_t pyramid_level,
                         Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    transform.linear().setIdentity();    //yh rotation part as identity

    //yh from top-to-bottom pyramid level
    for (ssize_t level = config.optical_flow_levels;
         level >= static_cast<ssize_t>(pyramid_level); level--) {

      //yh calculate scale
      const Scalar scale = 1 << level;

      Eigen::AffineCompact2f transform_tmp = transform;

      transform_tmp.translation() /= scale;  //yh suppose the feature motion in level0 is 4 pix, it's motion in level1 is 2 pix.

      //yh patch's initial coordinate at current level
      // input: pyr level and patch original pixel coordinate w.r.t. current level (old_transform.translation())
      PatchT p(old_pyr.lvl(level), old_transform.translation() / scale);  //yh call function OpticalFlowPatch and setFromImage

      // Perform tracking on current level
      patch_valid = trackPointAtLevel(pyr.lvl(level), p, transform_tmp);

      if (level == static_cast<ssize_t>(pyramid_level) + 1 && !patch_valid) {
        return false;
      }

      transform_tmp.translation() *= scale;   // yh rescale the rst back to level0

      if (patch_valid) {
        transform = transform_tmp;
      }
    }

    transform.linear() = old_transform.linear() * transform.linear();

    return patch_valid;
  }

  //yh patch tracking at current level.
  // key method implmentation
  inline bool trackPointAtLevel(const Image<const PixelType>& img_2,
                                const PatchT& dp,
                                Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    //yh set iteration number and patch is valid
    for (int iteration = 0;
         patch_valid && iteration < config.optical_flow_max_iterations;
         iteration++) {
      typename PatchT::VectorP res;

      //yh patch translate and rotate to current frame's patch
      // transformed_pattern is initialized using the previous frame, delta transform = 0
      typename PatchT::Matrix2P transformed_pat =
          transform.linear().matrix() * PatchT::pattern2;
      transformed_pat.colwise() += transform.translation();

      //yh calculate diff_coordinate between reference and current frame
      bool valid = dp.residual(img_2, transformed_pat, res);

      if (valid) {
        //yh calculate delta
        Vector3 inc = -dp.H_se2_inv_J_se2_T * res;
        transform *= SE2::exp(inc).matrix();

        const int filter_margin = 2;

        if (!img_2.InBounds(transform.translation(), filter_margin))
          patch_valid = false;
      } else {
        patch_valid = false;
      }
    }

    return patch_valid;
  }

  void addPoints() {
    for (const auto cam_idx : calib.main_cam_idx) {
      KeypointsData kd;

      Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f> new_poses_main,
          new_poses_stereo;
      std::map<KeypointId, size_t> new_pyramid_levels_main,
          new_pyramid_levels_stereo;

      for (ssize_t level = 0;
           level < static_cast<ssize_t>(config.optical_flow_levels) - 1;
           level++) {
        // for (ssize_t level = 0;
        //      level < 1;
        //      level++) {
        Eigen::aligned_vector<Eigen::Vector2d> pts;

        for (const auto& kv : transforms->observations.at(cam_idx)) {
          const ssize_t point_level =
              transforms->pyramid_levels.at(cam_idx).at(kv.first);   //yh kv.first: KeypointId

          // do not create points were already points at similar levels are
          if (point_level <= level + 1 && point_level >= level - 1) {
            // if (point_level == level) {
            const Scalar scale = 1 << point_level;  //yh Scalar must be float or double
            pts.emplace_back(
                (kv.second.translation() / scale).template cast<double>());
          }
        }

        detectKeypoints(pyramid->at(cam_idx).lvl(level), kd,
                        config.optical_flow_detection_grid_size, 1, pts);  //yh pts stores the points that we do not want to detect

        const Scalar scale = 1 << level;

        //yh add new feature measurement
        for (size_t i = 0; i < kd.corners.size(); i++) {
          Eigen::AffineCompact2f transform;
          transform.setIdentity();        //yh rotation
          transform.translation() =
              kd.corners[i].cast<Scalar>() * scale;  // TODO cast float?   //yh translation new on granite. initialize each point translation as the pix coord

          //yh store by map
          transforms->observations.at(cam_idx)[last_keypoint_id] = transform;
          transforms->pyramid_levels.at(cam_idx)[last_keypoint_id] = level;
          new_poses_main[last_keypoint_id] = transform;
          new_pyramid_levels_main[last_keypoint_id] = level;

          last_keypoint_id++;
        }
      }

      for (const auto stereo_pair : calib.stereo_pairs) {
        if (stereo_pair.first == cam_idx) {
          trackPoints(pyramid->at(cam_idx), pyramid->at(stereo_pair.second),
                      new_poses_main, new_pyramid_levels_main, new_poses_stereo,
                      new_pyramid_levels_stereo);

          for (const auto& kv : new_poses_stereo) {
            transforms->observations.at(stereo_pair.second).emplace(kv);
            transforms->pyramid_levels.at(stereo_pair.second)[kv.first] =
                new_pyramid_levels_stereo.at(kv.first);
          }
        }
      }
    }
  }

  void filterPoints() {
    //
    for (const auto& stereo_pair : calib.stereo_pairs) {
      std::set<KeypointId> lm_to_remove;

      std::vector<KeypointId> kpid;
      Eigen::aligned_vector<Eigen::Vector2f> proj0, proj1;

      for (const auto& kv : transforms->observations.at(stereo_pair.second)) {
        auto it = transforms->observations.at(stereo_pair.first).find(kv.first);

        if (it != transforms->observations.at(stereo_pair.first).end()) {
          proj0.emplace_back(it->second.translation());
          proj1.emplace_back(kv.second.translation());
          kpid.emplace_back(kv.first);
        }
      }

      Eigen::aligned_vector<Eigen::Vector4f> p3d_main, p3d_stereo;
      std::vector<bool> p3d_main_success, p3d_stereo_success;

      calib.intrinsics[stereo_pair.first].unproject(proj0, p3d_main,
                                                    p3d_main_success);
      calib.intrinsics[stereo_pair.second].unproject(proj1, p3d_stereo,
                                                     p3d_stereo_success);

      for (size_t i = 0; i < p3d_main_success.size(); i++) {
        if (p3d_main_success[i] && p3d_stereo_success[i]) {
          const double epipolar_error = std::abs(
              p3d_main[i].transpose() * E.at(stereo_pair) * p3d_stereo[i]);

          const Scalar scale =
              1 << transforms->pyramid_levels.at(stereo_pair.first).at(kpid[i]);

          if (epipolar_error > config.optical_flow_epipolar_error * scale) {
            lm_to_remove.emplace(kpid[i]);
          }
        } else {
          lm_to_remove.emplace(kpid[i]);
        }
      }

      for (int id : lm_to_remove) {
        transforms->observations.at(stereo_pair.second).erase(id);
      }
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  int64_t t_ns;

  size_t frame_counter;

  KeypointId last_keypoint_id;

  VioConfig config;
  granite::Calibration<Scalar> calib;

  OpticalFlowResult::Ptr transforms;
  std::shared_ptr<std::vector<granite::ManagedImagePyr<PixelType>>> old_pyramid,
      pyramid;

  // map from stereo pair -> essential matrix
  Eigen::aligned_unordered_map<CamPair, Matrix4> E;
};

}  // namespace granite
