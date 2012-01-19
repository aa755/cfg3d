/* 
 * File:   transferlabels.cpp
 * Author: aa755
 *
 * Created on January 18, 2012, 6:23 PM
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include "pcl/io/pcd_io.h"
#include "point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/intensity_spin.h"
#include "pcl/features/normal_3d.h"

#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl_ros/io/bag_io.h>
typedef pcl::PointXYZRGBCamSL PointT;


using namespace std;

pcl::PCDWriter writer;


/**
 * 
 */
int main(int argc, char** argv)
{

    if(argc!=3)
        cerr<<"usage:"<<argv[0]<<"sourcePCD targetPCD"<<endl;
    
    pcl::PointCloud<PointT> src;
    pcl::PointCloud<PointT> dst;
    pcl::io::loadPCDFile<PointT>(argv[1], src);
    pcl::io::loadPCDFile<PointT>(argv[2], dst);

    assert(src.size()==dst.size());
    for(int i=0;i<(int)src.size();i++)
    {
        dst.points[i].label=src.points[i].label;
    }
    
    writer.write<PointT>(argv[2],dst,true);
    return 0;
}

