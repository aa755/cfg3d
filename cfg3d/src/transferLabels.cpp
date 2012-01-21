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
 * barring nan's, points in source and destination PCD must be in same order
 */
int main(int argc, char** argv)
{

    if(argc!=3)
        cerr<<"usage:"<<argv[0]<<"sourcePCD targetPCD"<<endl;
    
    pcl::PointCloud<PointT> src;
    pcl::PointCloud<PointT> dst;
    pcl::io::loadPCDFile<PointT>(argv[1], src);
    pcl::io::loadPCDFile<PointT>(argv[2], dst);

    {//braces to restrict scope of iterators .. to prevent accidental use
        
         
        pcl::PointCloud<PointT>::iterator itSrc = src.begin();
        pcl::PointCloud<PointT>::iterator itDst = dst.begin();
        for (;(itSrc!=src.end()&&itDst!=dst.end()) ;)
        {
            if (isnan((*itSrc).x))
            {
                itSrc++;
                continue;
            }
            if (isnan((*itDst).x))
            {
                itDst++;
                continue;
            }
            
            (*itDst).label=(*itSrc).label;
            itSrc++;
            itDst++;
        }
        // the remaining part in this scope is only for safety check
            while (itSrc!=src.points.end() && isnan((*itSrc).x))
                itSrc++;
        
            while (itDst!=dst.points.end() && isnan((*itDst).x))
                itDst++;
        
        assert(itSrc==src.points.end()&&itDst==dst.points.end()); // both src and dst must have same number of nonNans
        
        
    }
    
    writer.write<PointT>(argv[2],dst,true);
    return 0;
}

