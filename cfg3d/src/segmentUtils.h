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
#include "segment-graph.h"
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

template <typename Point>
class SegmentPCD
{
public:
    virtual void segment(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointIndices> &clusters)=0;    
};

  template <typename PointT, typename Normal> 
  void extractEuclideanClustersM( const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals, \
      float tolerance, const boost::shared_ptr<pcl::KdTree<PointT> > &tree, \
      std::vector<pcl::PointIndices> &clusters,double eps_angle, \
      unsigned int min_pts_per_cluster = 1, \
       bool useColor=false, float colorT=0.3,unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
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
          bool colorAccept=true;
          if(useColor)
          {
                ColorRGB c1(cloud.points[i].rgb);
                ColorRGB c2(cloud.points[nn_indices[j]].rgb);
                float cdistance=ColorRGB::distance(c1,c2);
           //     cout<<"color dist"<<cdistance<<endl;
                if(cdistance>1.73*colorT)
                    colorAccept=false;
          }
          if ( fabs (acos (dot_p)) < eps_angle && colorAccept/* TODO: check color*/)
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
  void segment( pcl::PointCloud<PointSegT> &cloud,std::vector<pcl::PointIndices> &clusters, float radius=0.04, double angle=0.52, unsigned int numNbrForNormal=50, bool useColor=false, float colorT=0.3, unsigned int min_pts_per_cluster = 1)
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
    
    extractEuclideanClustersM<PointSegT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,100,useColor,colorT);
    cout<<"tolerance " << radius<<" angle "<<angle<<endl;
//    pcl::extractEuclideanClusters<PointSegT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,100);
    
}

bool compareSegsDecreasing(const pcl::PointIndices & seg1,const pcl::PointIndices & seg2) 
{
    return (seg1.indices.size()> seg2.indices.size());
}

template <typename Point>
class SegmentPCDGraph : public SegmentPCD<Point> {

    float sqrG(float y) {
        return y*y;
    }

    float maxG(float y1, float y2) {
        if (y1 > y2)
            return y1;
        else
            return y2;

    }

    float distanceG(pcl::PointXYZRGBNormal p1, pcl::PointXYZRGBNormal p2) {
        float ans = sqrG(p1.x - p2.x) + sqrG(p1.y - p2.y) + sqrG(p1.z - p2.z); //+sqrG(p1.normal_x-p2.normal_x);
        ans = sqrt(ans);
        return ans;

    }

    double diffclock(clock_t clock1, clock_t clock2) {
        double diffticks = clock1 - clock2;
        double diffms = (diffticks * 10) / CLOCKS_PER_SEC;
        return diffms;
    }

    float radius;
    int numNbrForNormal ;
    float colorWt ;
    int min_pts_per_cluster;
    float thresh;

    SegmentPCDGraph(float thresh = 0.1, float radius = 0.02, unsigned int numNbrForNormal = 50, bool useColor = false, float colorWt = 0.3, unsigned int min_pts_per_cluster = 1) {
        this->radius = radius;
        this->numNbrForNormal = numNbrForNormal;
        this->colorWt = colorWt;
        this->min_pts_per_cluster = min_pts_per_cluster;
        this->thresh = thresh;
    }

    float weightG(pcl::PointXYZRGBNormal p1, pcl::PointXYZRGBNormal p2) {
        ColorRGB c1(p1.rgb);
        ColorRGB c2(p2.rgb);
        //    float ans=c1.squaredError(c2)+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
        float ans = maxG(colorWt * c1.squaredError(c2), (1 - fabs(p1.normal_x * p2.normal_x + p1.normal_y * p2.normal_y + p1.normal_z * p2.normal_z)));
        //    float ans=(c1.squaredError(c2));//+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
        return ans;

    }
    //float radius=0.04, double angle=0.52, unsigned int numNbrForNormal=50, bool useColor=false, float colorT=0.3, unsigned int min_pts_per_cluster = 1;

    void concatenate(pcl::PointCloud<Point> &cloud, pcl::PointCloud<pcl::Normal> &normals, pcl::PointCloud<pcl::PointXYZRGBNormal> & cloundWnormal)
    {
        cloundWnormal.points.resize(cloud.size());
        for(int i=0;i<cloud.size();i++)
        {
            cloundWnormal.points[i].x=cloud.points[i].x;
            cloundWnormal.points[i].y=cloud.points[i].y;
            cloundWnormal.points[i].z=cloud.points[i].z;
            cloundWnormal.points[i].rgb=cloud.points[i].rgb;
            
            cloundWnormal.points[i].curvature=normals.points[i].curvature;            
            for(int j=0;j<4;j++)
                cloundWnormal.points[i].data_n[j]=normals.points[i].data_n[j];
        }
    }
    
    void segment(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointIndices> &clusters) {
        std::cerr << "using thresh of: " << thresh << endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB > ());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal > ());


        pcl::PassThrough<Point> pass_;
        
        pass_.setInputCloud(cloud);
        pass_.filter(*cloud_filtered);

        // Fill in the cloud data
        typename pcl::PointCloud<Point>::Ptr cloud_ptr = createStaticShared<pcl::PointCloud<PointT> >(&cloud);

        // Create the filtering object
        pcl::NormalEstimation<Point, pcl::Normal> n3d_;
       typename pcl::KdTree<Point>::Ptr normals_tree_, clusters_tree_;
        normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
        //  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
        pcl::PointCloud<pcl::Normal> cloud_normals;
        // Normal estimation parameters
        n3d_.setKSearch(numNbrForNormal);
        n3d_.setSearchMethod(normals_tree_);
        n3d_.setInputCloud(cloud_filtered);
        n3d_.compute(cloud_normals);
        
        concatenate(*cloud_filtered, cloud_normals, *final_cloud);


        size_t numPoints = final_cloud->size();
        std::cerr << "number of points : " << numPoints << std::endl;
        std::vector<int> k_indices;
        std::vector<float> k_distances;

        pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> nnFinder;
        nnFinder.setInputCloud(final_cloud);


        size_t numNeighbors;

        std::vector<edge> edges;
        size_t nbr;
        clock_t begin = clock();
        edge tedg; //=new edge;

        for (size_t i = 0; i < numPoints; i++) {
            numNeighbors = nnFinder.radiusSearch(i, radius, k_indices, k_distances, 20);
            for (size_t j = 0; j < numNeighbors; j++) {

                nbr = k_indices[j];
                if (nbr > i) {
                    tedg.a = i;
                    tedg.b = nbr;
                    tedg.w = weightG(final_cloud->points[i], final_cloud->points[nbr]);
                    edges.push_back(tedg);
                }
            }

        }
        // myfile.close();
        std::cerr << " num_edges: " << edges.size() << endl;
        clock_t end = clock();
        std::cerr << "Time elapsed: " << double(diffclock(end, begin)) << " ms" << endl;
        universe *u = segment_graph(numPoints, edges.size(), edges.data(), thresh);
        end = clock();
        std::cerr << "Time elapsed after segmentation: " << double(diffclock(end, begin)) << " ms" << endl;
        map<int,int> index2Count;
        std::cerr << "started loop: " << double(diffclock(end, begin)) << " ms" << endl;

        int comp;
        vector<int> segmentIndices;
        segmentIndices.resize(numPoints);
        for (size_t i = 0; i < numPoints; i++) {
            comp = u->find(i);
            index2Count[comp] = index2Count[comp] + 1;
            segmentIndices[i]=comp;
        }
        
        map<int, int> segRenaming;
        
        int goodClusterCount=0;
        for(map<int,int>::iterator it=index2Count.begin();it!=index2Count.end();it++)
        {
            if(it->second>=min_pts_per_cluster)
            {
                goodClusterCount++;
                segRenaming[it->first]=goodClusterCount; // 0=> not present
            }
        }
        
        clusters.resize(goodClusterCount);
        
        end = clock();
        std::cerr << "finished generating segmented pcd: " << double(diffclock(end, begin)) << " ms" << endl;
        for (size_t i = 0; i < numPoints; i++) {
            //            std::cerr<<i<<endl;
            comp = segmentIndices[i];
            int newIndex=segRenaming[comp];
            if (newIndex!=0)
                clusters.at(newIndex-1).indices.push_back(i);
        }



    }
};
#endif	/* SEGMENTUTILS_H */

