#include "camera_filters.hpp"

#include <string.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for getOptimalNewCameraMatrix
#include <opencv2/imgproc/imgproc.hpp> // for undistort

#include "misc/utils.h"

int initUndistortMap(optimal_camera_matrix *dst, std::string name, int width, int height) {
  std::string dir = getBinDir() + "../c/configs/" + name + "_camera_matrix_params.yaml";
  cv::FileStorage fs;
  cv::Mat M, D;

  fs.open(dir, CV_STORAGE_READ);
  if (!fs.isOpened()) {
    return 0;
  }

  fs["M"] >> M;
  fs["D"] >> D;

  //    Mat optimalNewMtx = getOptimalNewCameraMatrix(M,D,Size(width,height),-1);

  // the the optimalNewCameraMatrix as defined by opencv doesn't work well.
  // instead, hack together this undistortion map. This might not work for
  // future cameras, but it has worked for cameras on Gemini and Argo
  double scale = 1.7;
  cv::Mat P1 = scale * M;
  P1.at<double>(0,2) = P1.at<double>(0,2) / scale - 53; // X offset
  P1.at<double>(1,2) = P1.at<double>(1,2) / scale; // Y offset
  P1.at<double>(2,2) = 1.0;

  initUndistortRectifyMap(M, D, cv::Mat(), P1, cv::Size(width, height), CV_32FC1, dst->map1, dst->map2);

  return 1;
}
