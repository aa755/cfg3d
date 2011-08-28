#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
//#include "point_struct.h"
#include "point_types.h"
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include "utils.h"
#include <pcl/segmentation/extract_clusters.h>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/dynamic_bitset.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/algorithms/transform.hpp>

using namespace std;

typedef pcl::PointXYZRGBCamSL PointOutT;
typedef pcl::PointXYZRGB PointInT;

    void getComplementPointSet(vector<int>& memberIndices, vector<int>& complementIndices ,pcl::PointCloud<PointInT> & cloud_seg)
    {
        int it1=0;
        int numPoints=cloud_seg.size();
        complementIndices.clear();
        sort(memberIndices.begin(),memberIndices.end());
        vector<int>::iterator it2=memberIndices.begin();
        while ((it1 < numPoints ) && (it2 != memberIndices.end()))
        {
            if (it1 < *it2)
            {
                complementIndices.push_back(it1);
                it1++;
            }
            else if (it1==*it2)
            { 
                ++it1; //skip this point
                ++it2;
            }
            else
            {
                assert(1==2);
            }

        
        }
        
        while (it1 < numPoints )
        {
            complementIndices.push_back(it1);
            it1++;
        }
    }
    
    
    void getComplementSearchTree(pcl::KdTreeFLANN<PointInT> &nnFinder,vector<int>& memberIndices,pcl::PointCloud<PointInT> & cloud_seg)
    {
                
        vector<int> complementIndices;
        
        assert(cloud_seg.size()!=memberIndices.size()); // else it will crash in setInputCloud
        
        complementIndices.reserve(cloud_seg.size()-memberIndices.size()); // +1 is not required
                
        getComplementPointSet(memberIndices,complementIndices,cloud_seg);
        
        nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointInT> >(&cloud_seg),createStaticShared<vector<int> >(&complementIndices));
    }
    
    
class NeighborFinder
{
    boost::dynamic_bitset<> directions;
    int numAzimuthBins;
    int numElevationBins;
public:
    NeighborFinder(int numAzimuthBins_=8,int numElevationBins_=8)
    {
        numAzimuthBins=numAzimuthBins_;
        numElevationBins=numElevationBins_;
        directions.resize(numAzimuthBins*numElevationBins,false);
    }
    
    int getIndex(double azimuth, double elevation)
    {
        int azimuthBin=(int)round(azimuth);
        int elevationBin=(int)round(elevation);
        assert(elevationBin<=numElevationBins);
        assert(azimuthBin<numAzimuthBins);
      //  if(elevationBin==numElevationBins)
      //      elevationBin=0; //circularity
      //  if(azimuthBin<numAzimuthBins);
        
        return azimuthBin*numElevationBins+elevationBin;
    }
    
    void processNeighbor(PointOutT p)
    {
        //rev svn r59763
/* bg::model::point<long double, 2, bg::cs::spherical<bg::degree> > p1(15.0, 5.0);
    
    // Transform from degree to radian. Default strategy is automatically selected,
    // it will convert from degree to radian
    bg::model::point<long double, 2, bg::cs::spherical<bg::radian> > p2;
    bg::transform(p1, p2);
    
    // Transform from degree (lon-lat) to 3D (x,y,z). Default strategy is automatically selected, 
    // it will consider points on a unit sphere
    bg::model::point<long double, 3, bg::cs::cartesian> p3;
    bg::transform(p1, p3);
    
    std::cout 
        << "p1: " << bg::dsv(p1) << std::endl
        << "p2: " << bg::dsv(p2) << std::endl
        << "p3: " << bg::dsv(p3) << std::endl;

    return 0;        
  */      
    }
    
};
class OccupancyMap    
{
     float resolution;
     octomap::OcTreeROS tree;
     pcl::PointCloud<pcl::PointXYZ> xyzcloud;     
public:
    pcl::PointXYZ convertFromVector(Eigen::Vector4f p)
    {
        return pcl::PointXYZ(p(0),p(1),p(2));
    }
    
    static void convertToXYZ(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,pcl::PointCloud<pcl::PointXYZ> & cloudxyz)
    {
        cloudxyz.points.resize(cloud.size());
        for(size_t i=0;i<cloud.size();i++)
        {
            cloudxyz.points[i].x=cloud.points[i].x;
            cloudxyz.points[i].y=cloud.points[i].y;
            cloudxyz.points[i].z=cloud.points[i].z;
        }
    }
    
     enum OccupancyState {OCCUPANCY_OCCUPIED = 1, OCCUPANCY_FREE = 2, OCCUPANCY_OCCLUDED = 3,OCCUPANCY_OUT_OF_RANGE = 4};
    OccupancyMap(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, float resolution_=0.02): tree(resolution_)
    {
        resolution=resolution_;
        // convert to  pointXYZ format
        convertToXYZ(cloud,xyzcloud);
       // pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZ>(cloud,xyzcloud);        
        tree.insertScan(xyzcloud,convertFromVector(cloud.sensor_origin_),-1, true);
        octomap::point3d test;
        test(0)=-10;
        test(1)=-10;
        test(2)=-10;
        tree.octree.setBBXMin(test);
        test(0)=10;
        test(1)=10;
        test(2)=10;
        tree.octree.setBBXMax(test );
        
        //tree.octree.setBBXMax(octomap::point3d(10,10,10));
        //http://www.ros.org/doc/api/octomap_ros/html/classoctomap_1_1OctomapROS.html
    }
    
    
    
    OccupancyState getOccupancyState(const pcl::PointXYZ pt)
    {
            octomap::OcTreeROS::NodeType * treeNode;
//            pcl::PointXYZ ptemp=pt;
  //          ptemp.y+=0.1;
            treeNode = tree.search(pt);
    
            if(treeNode==NULL)
            {
                cout<<"null returned"<<endl;
                return OCCUPANCY_OUT_OF_RANGE;
            }
            double occupancy=treeNode->getOccupancy();            
            cout<<"getOcc:"<<occupancy<<endl;
            if(occupancy>=0.7)
                return OCCUPANCY_OCCUPIED;
            else if (occupancy>0.5)
                return OCCUPANCY_OCCLUDED;
            else 
                return OCCUPANCY_FREE;
    }
    
    OccupancyState getOccupancyState(size_t index)
    {
        return getOccupancyState(xyzcloud.points[index]);
    }
};

int main(int argc, char** argv)
{
    namespace bg = boost::geometry;
    typedef bg::point<double, 3, bg::cs::cartesian> cartesian;
    typedef bg::point<double, 2, bg::cs::spherical<bg::degree> > spherical;
//  ros::init(argc, argv,"segmenterAndGraphMaker");
  
cartesian p3(-1.0/sqrt(2),-1.0/sqrt(2),0);
spherical p1;
bg::transform<cartesian,spherical>(p3,p1);
cout<<bg::dsv(p1)<<endl;
exit(0);

pcl::PointCloud<PointOutT> cloud_seg;
  pcl::PointCloud<PointInT> cloud;
  pcl::PointCloud<PointInT>::Ptr cloud_ptr=createStaticShared<pcl::PointCloud<PointInT> >(&cloud);
  pcl::io::loadPCDFile<PointInT>(argv[1], cloud);
  
  

    int number_neighbours = 100;
    float radius = 0.05;// 0.025
    float angle = 0.32;
    pcl::KdTree<PointInT>::Ptr normals_tree_, clusters_tree_;
    pcl::NormalEstimation<PointInT,pcl::Normal> n3d_;
  std::vector<pcl::PointIndices> clusters;


    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointInT> > ();
    initTree (0, clusters_tree_);
    clusters_tree_->setInputCloud (cloud_ptr);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointInT> > ();
    n3d_.setKSearch (number_neighbours);
//    n3d_.setSearchMethod (normals_tree_);
    n3d_.setSearchMethod (clusters_tree_);

    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(cloud_normals);
    
    //pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = createStaticShared<const pcl::PointCloud<pcl::Normal> > (& cloud_normals);
    pcl::extractEuclideanClusters<PointInT,pcl::Normal> ( cloud, cloud_normals, radius, clusters_tree_, clusters, angle);
    std::cout<<clusters.size() << "clusters found in pcd of size "<<cloud.size()<<std::endl;

    cloud_seg.points.resize(cloud.size());
    
    for (size_t i = 0; i < cloud.size (); i++)
  {
       cloud_seg.points[i].clone(cloud.points[i]);
        
    }
    
    int total=0;
    
  for (size_t i = 0; i < clusters.size (); i++)
  {
    for (size_t j = 0; j < clusters[i].indices.size (); j++)
    {
       cloud_seg.points[clusters[i].indices[j]].segment=i+1;
    }
    std::cout<<"seg size "<<clusters[i].indices.size()<<std::endl;
    total+=clusters[i].indices.size();
  }
    std::cout<<total<<std::endl;

    OccupancyMap occupancy(cloud);
    for(size_t i=0;i<cloud.size();i++)
    {
        cout<<occupancy.getOccupancyState(i)<<endl;
    }
    
    pcl::PointXYZ t;
    t.x=-1;
    t.y=1.2;
    t.z=-0.5;
            
    cout<<"special test point in:"<<occupancy.getOccupancyState(t)<<endl;
    
    t.y=0.9;
    cout<<"special test point out:"<<occupancy.getOccupancyState(t)<<endl;
    
  //pcl::io::savePCDFile<PointOutT>("segmented_"+std::string(argv[1]), cloud_seg);
    
    }


