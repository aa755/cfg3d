#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <string>
#include "pcl/filters/extract_indices.h"
#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "segment-graph.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>

using namespace std;



class ColorRGB
{
    float r;
    float g;
    float b;

public:
    ColorRGB(float rgb)
    {
        int rgbi=*reinterpret_cast<int*>(&rgb);
        parseColorRGB(rgbi);

    }

    ColorRGB(int rgbi)
    {
        parseColorRGB(rgbi);
    }

    void parseColorRGB(int rgbi)
    {
        int ri=(rgbi&(0xff0000))>>16;
        int gi=(rgbi&(0xff00))>>8;
        int bi=(rgbi&(0xff));
        r=ri/255.0;
        g=gi/255.0;
        b=bi/255.0;
    }

    float getFloatRep()
    {
        int color=(((int)(r*255))<<16)+(((int)(g*255))<<8)+(((int)(b*255)));
        return *reinterpret_cast<float*>(&color);
    }

    float squaredError(ColorRGB c)
    {
        float err= sqrG(r-c.r)+sqrG(g-c.g)+sqrG(b-c.b);
       // cout<<err<<endl;
        return err;
    }

    void print()
    {
        std::cerr<<r*255<<" "<<g*255<<" "<<b*255<<endl;
    }
};



/* ---[ */
int
  main (int argc, char** argv)
{
}
/* ]--- */
