/* 
 * File:   OccupancyMapWrapper.h
 * Author: aa755
 *
 * Created on November 5, 2011, 8:35 PM
 */

#ifndef OCCUPANCYMAP_H
#define	OCCUPANCYMAP_H
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <iostream>
#include <fstream>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include "utils.h"
//#include "wallDistance.h"
#include "wallDistance.h"
using namespace std;

template <typename PointOutT>
class OccupancyMap
{
protected:
    float resolution;
    octomap::OcTreeROS tree;
    pcl::PointCloud<pcl::PointXYZ> xyzcloud;
    pcl::KdTreeFLANN<PointOutT> nnFinder;
    pcl::PointCloud<PointOutT> *cloudSeg;
    static const float nearThresh=0.07;
    static const float nearOccThresh=0.2;
    static const float step = 0.005;
    bool maxDistReady;
public:
    double maxDist[360];

    void setmaxDistReady()
    {
        maxDistReady=true;
    }
    
    bool isWithinBoundary(const pcl::PointXYZ pt)
    {
        assert(maxDistReady);
                 pair<int,double>angDist= get2DAngleDegreesAndDistance<pcl::PointXYZ>(pt,cloudSeg->sensor_origin_);
                int angle=angDist.first;
                double dist=maxDist[angle]-angDist.second;
                
//                if(dist<0)
//                    cerr<<"outside boundary"<<endl;
                
                return (dist>=0);
       
    }
    
    pcl::PointXYZ convertFromVector(Eigen::Vector4f p)
    {
        return pcl::PointXYZ(p(0), p(1), p(2));
    }

    static void convertToXYZ(const pcl::PointCloud<PointOutT> &cloud, pcl::PointCloud<pcl::PointXYZ> & cloudxyz)
    {
        cloudxyz.points.resize(cloud.size());
        cloudxyz.sensor_origin_=cloud.sensor_origin_;
        for (size_t i = 0; i < cloud.size(); i++)
        {
            cloudxyz.points[i].x = cloud.points[i].x;
            cloudxyz.points[i].y = cloud.points[i].y;
            cloudxyz.points[i].z = cloud.points[i].z;
        }
    }

    enum OccupancyState
    {
        OCCUPANCY_OCCUPIED = 1, OCCUPANCY_FREE = 2, OCCUPANCY_OUT_OF_RANGE = 3
    };

    OccupancyMap(pcl::PointCloud<PointOutT> & cloud, float resolution_ = 0.02) : tree(resolution_)
    {
        maxDistReady=false;
        cloudSeg=& cloud;
        resolution = resolution_;
#ifndef USING_SVM_FOR_LEARNING_CFG
        convertToXYZ(cloud, xyzcloud);
    //    cout<<"Computing Occupancy Map using origin as: "<<cloud.sensor_origin_<<endl;
        tree.insertScan(xyzcloud, convertFromVector(cloud.sensor_origin_), -1, true);
        //http://www.ros.org/doc/api/octomap_ros/html/classoctomap_1_1OctomapROS.html
        
        nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointOutT> >(&cloud));
#endif

    }

    OccupancyState getOccupancyState(Eigen::Vector3d ptv)
    {
        pcl::PointXYZ pt;
        for(int i=0;i<3;i++)
            pt.data[i]=ptv(i);
        pt.data[3]=1.0f;
        return getOccupancyState(pt);
    }
    
    OccupancyState getOccupancyState(const pcl::PointXYZ pt)
    {
        octomap::OcTreeROS::NodeType * treeNode;
        treeNode = tree.search(pt);

        if (treeNode == NULL)
        {
           // cout << "null returned" << endl;
            return OCCUPANCY_OUT_OF_RANGE;
        }
        double occupancy = treeNode->getOccupancy();
        cout << "getOcc:" << occupancy << endl;
        if (occupancy >= 0.5)
            return OCCUPANCY_OCCUPIED;
        else
            return OCCUPANCY_FREE;
    }
    
    bool isFree(const pcl::PointXYZ pt)
    {
        octomap::OcTreeROS::NodeType * treeNode;
        treeNode = tree.search(pt);

        if (treeNode == NULL)
            return false;
        double occupancy = treeNode->getOccupancy();
        if (occupancy >= 0.5)
            return false;
        else
            return true;
        
    }
    
/**
 * things outside boundary are not considered occluded.
 * because we assume, every object is fully inside the boundary
 * @param pt
 * @return 
 */
    bool isOccluded(const pcl::PointXYZ pt)
    {
        assert(false); // insert scan was disabled in cosntructor
        if(!isWithinBoundary(pt))
            return false;
        
        octomap::OcTreeROS::NodeType * treeNode;
        treeNode = tree.search(pt);

        if (treeNode == NULL)
            return true;
        else
            return false;
        
    }
    
    bool isOccluded(Eigen::Vector3d ptv)
    {
        pcl::PointXYZ pt;
        for(int i=0;i<3;i++)
            pt.data[i]=ptv(i);
        pt.data[3]=1.0f;
        
        return isOccluded(pt);        
    }
    
    OccupancyState getOccupancyState(size_t index)
    {
        return getOccupancyState(xyzcloud.points[index]);
    }

    void printPoint(PointOutT p,string msg="")
    {
        cout<<msg<<":("<<p.x<<","<<p.y<<","<<p.z<<","<<")"<<endl;
    }

};



#endif	/* OCCUPANCYMAP_H */

