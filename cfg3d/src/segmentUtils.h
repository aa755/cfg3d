/* 
 * File:   segmentUtils.h
 * Author: aa755
 *
 * Created on February 10, 2012, 7:12 PM
 */

#ifndef SEGMENTUTILS_H
#define	SEGMENTUTILS_H
#include "structures.cpp"
#include "OccupancyMap.h"
#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
//#include "point_struct.h"
#include "point_types.h"
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/dynamic_bitset.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include "extract_planes.cpp"
#include "utils.h"
//#define FILTER_SEGMENTS_SEGMENTATION
typedef pcl::PointXYZRGBCamSL PointOutT;
//typedef pcl::PointXYZRGBCamSL PointInT;
typedef pcl::PointXYZRGB PointInT;
template<typename PointET>
Eigen::Vector3f getPoint(PointET p)
{
    Eigen::Vector3f temp;
    for(int i=0;i<3;i++)
        temp(i)=p.data[i];
    
    return temp;
}

  template <typename PointT, typename Normal> 
  void extractEuclideanClustersM( const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals, \
      float tolerance, const boost::shared_ptr<pcl::KdTree<PointT> > &tree, \
      std::vector<pcl::PointIndices> &clusters, double eps_angle, \
      unsigned int min_pts_per_cluster = 1, \
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!", cloud.points.size (), normals.points.size ());
      return;
    }

    Eigen::Vector3f origin;
    for(int i=0;i<3;i++)
        origin(i)=cloud.sensor_origin_(i);
    
    cout<<"origin:"<<endl;
    cout<<origin<<endl;
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (processed[i])
        continue;

      std::vector<unsigned int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (i);

      processed[i] = true;

      while (sq_idx < (int)seed_queue.size ())
      {
          Eigen::Vector3f pc=getPoint(cloud.points.at(seed_queue[sq_idx]));
          Eigen::Vector3f dir=(pc-origin);
                  
          //cout<<"direction:"<<dir<<endl<<"dirnorm"<<dir.norm()<<endl;
          float rad=sqr(dir.norm())*0.0025+tolerance;
          //cout<<"rad:"<<rad<<endl;
        // Search for sq_idx
        if (!tree->radiusSearch (seed_queue[sq_idx],rad , nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                         // Has this point been processed before ?
            continue;


          
         // processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                         normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                         normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle /* TODO: check color*/)
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }

        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
      {
        pcl::PointIndices r;
        r.indices.resize (seed_queue.size ());
        for (size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);   // We could avoid a copy by working directly in the vector
      }
    }
  }
  
template <typename PointSegT>
  void segment( pcl::PointCloud<PointSegT> &cloud,std::vector<pcl::PointIndices> &clusters, float radius=0.04, double angle=0.52, unsigned int numNbrForNormal=50, unsigned int min_pts_per_cluster = 1)
{
    typename pcl::KdTree<PointSegT>::Ptr normals_tree_, clusters_tree_;
    pcl::NormalEstimation<PointSegT, pcl::Normal> n3d_;
        typename pcl::PointCloud<PointSegT>::Ptr cloud_ptr = createStaticShared<pcl::PointCloud<PointSegT> >(&cloud);


    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointSegT> > ();
    initTree(0, clusters_tree_);
    clusters_tree_->setInputCloud(cloud_ptr);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointSegT> > ();
    n3d_.setKSearch(numNbrForNormal);
    //    n3d_.setSearchMethod (normals_tree_);
    n3d_.setSearchMethod(clusters_tree_);

    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(cloud_normals);

    //pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = createStaticShared<const pcl::PointCloud<pcl::Normal> > (& cloud_normals);
 //   pcl::extractEuclideanClusters<PointInT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,300);
    
    extractEuclideanClustersM<PointSegT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,500);
    
}

bool compareSegsDecreasing(const pcl::PointIndices & seg1,const pcl::PointIndices & seg2) 
{
    return (seg1.indices.size()> seg2.indices.size());
}

#endif	/* SEGMENTUTILS_H */

