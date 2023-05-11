/**
 * Copyright(c)2017 ChangSha XingShen Technology Ltd .
 * All rights reserved
 * @projectName   LocalizerInit
 * @fileName      transform.h
 * @brief
 * @author        fuxiang
 * @date          2019-10-29
 */
#ifndef TRANSFORM_H
#define TRANSFORM_H
#include <opencv2/opencv.hpp>
#include <vector>

#include "common/common/utils.h"

class Transform {
 public:
  Transform();
  ~Transform();

  bool IsRotationMatrix(const cv::Mat &R);
  void EulerAnglesToRotationMatrix(const cv::Vec3i &theta, double R[3][3]);
  cv::Vec3f RotationMatrixToEulerAngels(const cv::Mat &R);
  void GaussProjInvCal(double X, double Y, double *longitude, double *latitude);
};

#endif  // TRANSFORM_H
