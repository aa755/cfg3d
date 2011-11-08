#ifndef PREGRASP_POINT_STRUCT
#define PREGRASP_POINT_STRUCT
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include "pcl/kdtree/tree_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/impl/tree_types.hpp"
#include "pcl/kdtree/impl/organized_data.hpp"
#include "pcl/features/normal_3d.h"
#include "pcl/features/impl/normal_3d.hpp"

//#include "includes/point_types.h"

namespace pcl{
#define  RGB_INDEX_CLUSTER_C_STRUCT union {                   \
  struct {                                \
    float rgb;                          \
    unsigned int index, cluster_id;                   \
  };                                      \
  float data_c[4];                        \
}

#define  RGB_INDEX_C_STRUCT union {                   \
  struct {                                \
    float rgb;                          \
    unsigned int index;                   \
  };                                      \
  float data_c[4];                        \
}

struct PointXYZRGBIndexCluster {
  PCL_ADD_POINT4D;
  /*union {
    struct {
    float rgb;
    unsigned int index;
    unsigned int cluster_id;
    };
    float data_c[4];
    };*/
  RGB_INDEX_CLUSTER_C_STRUCT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PointXYZRGBIndexCluster operator+(const PointXYZRGBIndexCluster &b) {
    PointXYZRGBIndexCluster c = *this;
    c.x += b.x;
    c.y += b.y;
    c.z += b.z;
    return c;
  }
  PointXYZRGBIndexCluster operator-(const PointXYZRGBIndexCluster &b) {
    PointXYZRGBIndexCluster c = *this;
    c.x -= b.x;
    c.y -= b.y;
    c.z -= b.z;
    return c;
  }
  PointXYZRGBIndexCluster operator/(double s) {
    PointXYZRGBIndexCluster c = *this;
    c.x /= s;
    c.y /= s;
    c.z /= s;
    return c;
  }
} EIGEN_ALIGN16;

struct PointXYZRGBIndex {
  PCL_ADD_POINT4D;
  /*union {
    struct {
    float rgb;
    unsigned int index;
    unsigned int cluster_id;
    };
    float data_c[4];
    };*/
  RGB_INDEX_C_STRUCT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PointXYZRGBIndex operator/ (double s)
  {
      PointXYZRGBIndex c = *this;
      c.x /= s;
      c.y /= s;
      c.z /= s;
      return c;
  }
  PointXYZRGBIndex operator+(const PointXYZRGBIndex &b)
  {
      PointXYZRGBIndex c = *this;
      c.x += b.x;
      c.y += b.y;
      c.z +=b.z;
      return c;
  }
  PointXYZRGBIndex operator-(const PointXYZRGBIndex &b)
  {
      PointXYZRGBIndex c = *this;
      c.x -= b.x;
      c.y -= b.y;
      c.z -=b.z;
      return c;
  }
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBIndexCluster,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (unsigned int, index, index)
    (unsigned int, cluster_id, cluster_id));

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBIndex,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (unsigned int, index, index));


#endif // PREGRASP_POINT_STRUCT
