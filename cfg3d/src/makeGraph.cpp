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
    
int main(int argc, char** argv)
{
//  ros::init(argc, argv,"segmenterAndGraphMaker");
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

  pcl::io::savePCDFile<PointOutT>("segmented_"+std::string(argv[1]), cloud_seg);
    
    }


