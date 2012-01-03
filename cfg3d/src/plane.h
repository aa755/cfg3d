#include <ros/ros.h>
#include <pcl/point_types.h>
#include "pcl/ModelCoefficients.h"

 struct plane{
  std::vector<pcl::PointXYZRGB> pts;
  pcl::ModelCoefficients normal;
  int points;
};

struct orientation{
  float x;
  float y;
  float z;
};
