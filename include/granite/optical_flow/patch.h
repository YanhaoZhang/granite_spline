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

#include <Eigen/Dense>

#include <granite/image/image.h>
#include <granite/optical_flow/patterns.h>

namespace granite {

template <typename Scalar, typename Pattern>
struct OpticalFlowPatch {
  static constexpr int PATTERN_SIZE = Pattern::PATTERN_SIZE;

  typedef Eigen::Matrix<int, 2, 1> Vector2i;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 1, 2> Vector2T;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 1> VectorP;

  typedef Eigen::Matrix<Scalar, 2, PATTERN_SIZE> Matrix2P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 2> MatrixP2;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 3> MatrixP3;
  typedef Eigen::Matrix<Scalar, 3, PATTERN_SIZE> Matrix3P;
  typedef Eigen::Matrix<Scalar, PATTERN_SIZE, 4> MatrixP4;
  typedef Eigen::Matrix<int, 2, PATTERN_SIZE> Matrix2Pi;

  static const Matrix2P pattern2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OpticalFlowPatch() { mean = 0; }

  OpticalFlowPatch(const Image<const PixelType> &img, const Vector2 &pos) {
    setFromImage(img, pos);
  }

  //yh key method idea of calculate Jacobian of optical flow
  // refer to https://www.bilibili.com/video/BV1rV4y1y7BQ?p=1&vd_source=b8a3c79bdb6454e08dd84dd50816b460 (09.55)
  void setFromImage(const Image<const PixelType> &img, const Vector2 &pos) {
    this->pos = pos;

    int num_valid_points = 0;
    Scalar sum = 0;
    Vector2 grad_sum(0, 0);

    MatrixP2 grad;    //yh the grad is fixed. The main idea is very similar as zhehua's miccai.

    for (int i = 0; i < PATTERN_SIZE; i++) {
      Vector2 p = pos + pattern2.col(i);    //yh pattern2 is set before, e.g., patent50 uses surrouding 50 pixels?
      if (img.InBounds(p, 2)) {
        Vector3 valGrad = img.interpGrad<Scalar>(p);
        data[i] = valGrad[0];
        sum += valGrad[0];
        grad.row(i) = valGrad.template tail<2>();
        grad_sum += valGrad.template tail<2>();
        num_valid_points++;
      } else {
        data[i] = -1;
      }
    }

    mean = sum / num_valid_points;

    Scalar mean_inv = num_valid_points / sum;

    Eigen::Matrix<Scalar, 2, 3> Jw_se2;     //yh the rotation and translation of patch is in SE2.
    Jw_se2.template topLeftCorner<2, 2>().setIdentity();

    MatrixP3 J_se2;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (data[i] >= 0) {
        const Scalar data_i = data[i];
        const Vector2 grad_i = grad.row(i);
        grad.row(i) =
            num_valid_points * (grad_i * sum - grad_sum * data_i) / (sum * sum);   //yh first term

        data[i] *= mean_inv;
      } else {
        grad.row(i).setZero();
      }

      // Fill jacobians with respect to SE2 warp
      Jw_se2(0, 2) = -pattern2(1, i);   //yh second term
      Jw_se2(1, 2) = pattern2(0, i);
      J_se2.row(i) = grad.row(i) * Jw_se2;  //yh first term * second term
    }

    Matrix3 H_se2 = J_se2.transpose() * J_se2;
    Matrix3 H_se2_inv;
    H_se2_inv.setIdentity();
    H_se2.ldlt().solveInPlace(H_se2_inv);

    H_se2_inv_J_se2_T = H_se2_inv * J_se2.transpose();
  }

  inline bool residual(const Image<const PixelType> &img,
                       const Matrix2P &transformed_pattern,
                       VectorP &residual) const {
    Scalar sum = 0;
    Vector2 grad_sum(0, 0);
    int num_valid_points = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (img.InBounds(transformed_pattern.col(i), 2)) {
        //yh here not the residual but the new intensity of the image is retrived
        // transformed_pattern is initialized using the previous frame, delta transform = 0
        residual[i] = img.interp<Scalar>(transformed_pattern.col(i));
        sum += residual[i];
        num_valid_points++;
      } else {
        residual[i] = -1;
      }
    }

    int num_residuals = 0;

    bool iszerosum = false;
    if (fabs(sum) < 1.e-8) {
      iszerosum = true;
    }
    if (iszerosum && num_valid_points > 0) {
      std::cout << "sum = " << sum << std::endl;
      std::cout << "num_valid_points = " << num_valid_points << std::endl;
      for (int i = 0; i < PATTERN_SIZE; i++) {
        std::cout << i << ", res = " << residual[i] << std::endl;
      }
      return false;
    }
    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (residual[i] >= 0 && data[i] >= 0) {
        const Scalar val = residual[i];
        residual[i] = num_valid_points * val / sum - data[i];    //yh actual residual is calculated mean(It+1) - mean(It)
        num_residuals++;

      } else {
        residual[i] = 0;
      }
    }

    return num_residuals > PATTERN_SIZE / 2;
  }

  Vector2 pos;
  VectorP data;  // negative if the point is not valid

  // MatrixP3 J_se2;  // total jacobian with respect to se2 warp
  // Matrix3 H_se2_inv;
  Matrix3P H_se2_inv_J_se2_T;

  Scalar mean;
};

template <typename Scalar, typename Pattern>
const typename OpticalFlowPatch<Scalar, Pattern>::Matrix2P
    OpticalFlowPatch<Scalar, Pattern>::pattern2 = Pattern::pattern2; //todo where the value is given?

}  // namespace granite
