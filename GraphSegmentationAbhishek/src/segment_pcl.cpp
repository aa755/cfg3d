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
//#include "image.h"
//#include "misc.h"
//#include "filter.h>
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


float sqrG(float y)
{
    return y*y;
}

float maxG(float y1, float y2)
{
    if(y1>y2)
        return y1;
    else
        return y2;

}

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

float distanceG(pcl::PointXYZRGBNormal p1,pcl::PointXYZRGBNormal p2)
{
    float ans=sqrG(p1.x-p2.x)+sqrG(p1.y-p2.y)+sqrG(p1.z-p2.z);//+sqrG(p1.normal_x-p2.normal_x);
    ans=sqrt(ans);
    return ans;

}
float weightG(pcl::PointXYZRGBNormal p1,pcl::PointXYZRGBNormal p2)
{
    ColorRGB c1(p1.rgb);
    ColorRGB c2(p2.rgb);
//    float ans=c1.squaredError(c2)+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
    float ans=maxG(0.3*c1.squaredError(c2),(1-fabs(p1.normal_x*p2.normal_x+p1.normal_y*p2.normal_y+p1.normal_z*p2.normal_z)));
//    float ans=(c1.squaredError(c2));//+sqrG(p1.normal_x-p2.normal_x)+sqrG(p1.normal_y-p2.normal_y)+sqrG(p1.normal_z-p2.normal_z);
    return ans;

}

double diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*10)/CLOCKS_PER_SEC;
	return diffms;
}

/* ---[ */
int
  main (int argc, char** argv)
{
   float thresh;
    if(argc<3)
        thresh=1.5;
    else
    {
        thresh=atof(argv[2]);
    }
        std::cerr<<"using thresh of: "<<thresh<<endl;
    
    float radius=0.02;
  typedef pcl::PointXYZRGB    Point;
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ()), cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB> ()),cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

typedef  pcl::KdTree<Point> KdTree;
typedef  pcl::KdTree<Point>::Ptr KdTreePtr;
int stage=1;
  // Fill in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  if(stage<=1)
{
  reader.read<pcl::PointXYZRGB> (argv[1], *cloud);

    pcl::PassThrough<Point> pass_;
 pass_.setInputCloud (cloud);
  pass_.filter (*cloud_filtered);

  // Fill in the cloud data

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered1);

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  ror.setInputCloud (cloud_filtered1);
  std::cerr << "before radius : " << cloud_filtered1->size()<<std::endl;
  ror.setRadiusSearch(0.999*radius);
  ror.setMinNeighborsInRadius(2);
  ror.filter (*cloud_filtered2);
  std::cerr << "after radius : " <<cloud_filtered2->size()<<std::endl;

  writer.write<pcl::PointXYZRGB> ("inliers.pcd", *cloud_filtered2, false);
}
else if(stage==2)
{
  reader.read<pcl::PointXYZRGB> ("inliers.pcd", *cloud_filtered2);
}
  // release sor.
//  sor.setNegative (true);
//  sor.filter (*cloud_filtered);
//  writer.write<pcl::PointXYZRGB> ("outliers.pcd", *cloud_filtered, false);
if(stage<=2)
{
    *cloud_filtered2=*cloud_filtered;
   pcl::NormalEstimation<Point, pcl::Normal> n3d_;
     KdTreePtr normals_tree_, clusters_tree_;
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
//  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
pcl::PointCloud<pcl::Normal> cloud_normals;
  // Normal estimation parameters
  n3d_.setKSearch (50);
  n3d_.setSearchMethod (normals_tree_);
   n3d_.setInputCloud (cloud_filtered2);
  n3d_.compute (cloud_normals); 
  writer.write<pcl::Normal> ("normals.pcd", cloud_normals, false);
 pcl::concatenateFields (*cloud_filtered2, cloud_normals, *final_cloud);
  writer.write<pcl::PointXYZRGBNormal> ("xyzRGBnormals.pcd", *final_cloud, false);
}
else
  reader.read<pcl::PointXYZRGBNormal> ("xyzRGBnormals.pcd", *final_cloud);
  
  size_t numPoints=final_cloud->size();
  std::cerr << "number of points : " << numPoints<<std::endl;
  std::vector<int> k_indices;
  std::vector<float> k_distances;

  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> nnFinder;
  nnFinder.setInputCloud(final_cloud);


//  ofstream myfile;
//  myfile.open ("edges.txt");
  size_t numNeighbors;
//  delete cloud;
//  delete cloud_filtered;
//  delete cloud_filtered2;
//  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
   //pcl::ExtractIndices<pcl::PointXYZ> extract;
//   vector<int> isolated
  std::vector<edge> edges;
  size_t nbr;
  	clock_t begin=clock();
                    edge tedg;//=new edge;

  for(size_t i=0;i<numPoints;i++)
  {
    numNeighbors=nnFinder.radiusSearch(i,radius,k_indices,k_distances,20);
    for(size_t j=0;j<numNeighbors;j++)
    {

        nbr=k_indices[j];
        if(nbr>i)
        {
            tedg.a=i;
            tedg.b=nbr;
            tedg.w=weightG(final_cloud->points[i],final_cloud->points[nbr]);
 //           myfile<<tedg.a<<","<<tedg.b<<","<<tedg.w<<endl;
            edges.push_back(tedg);
        }
    }

  }
 // myfile.close();
        std::cerr<<" num_edges: "<<edges.size()<<endl;
	clock_t end=clock();
  std::cerr << "Time elapsed: " << double(diffclock(end,begin)) << " ms"<< endl;
    universe *u = segment_graph(numPoints, edges.size(), edges.data(), thresh);
	end=clock();
  std::cerr << "Time elapsed after segmentation: " << double(diffclock(end,begin)) << " ms"<< endl;
        vector<int> colors;
        vector<int> counts;
        vector<int> segmentIndices;
        colors.reserve(numPoints);
        counts.reserve(numPoints);
        segmentIndices.reserve(numPoints);
        std::cerr << "started loop: " << double(diffclock(end,begin)) << " ms"<< endl;
        for(size_t i=0;i<numPoints;i++)
        {
            //value=value+72857234;
            colors.push_back((int)random());
            counts.push_back(0);
        }
	end=clock();
        std::cerr << "finished generating random nos: " << double(diffclock(end,begin)) << " ms"<< endl;
        int comp;
        for(size_t i=0;i<numPoints;i++)
        {
//            std::cerr<<i<<endl;
            comp = u->find(i);
            counts[comp]=counts[comp]+1;
            ColorRGB tempc(colors[comp]);
            final_cloud->points[i].rgb=tempc.getFloatRep();
            segmentIndices.push_back(comp);
            
        }
        pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
        pcl::PointIndices::Ptr outliers (new pcl::PointIndices ());

	end=clock();
        std::cerr << "finished generating segmented pcd: " << double(diffclock(end,begin)) << " ms"<< endl;
        for(size_t i=0;i<numPoints;i++)
        {
//            std::cerr<<i<<endl;
            comp=segmentIndices[i];
            if(counts[comp]<50)
                outliers->indices.push_back(i);

        }
extract.setInputCloud (final_cloud);
    extract.setIndices (outliers);
    extract.setNegative (true);
    extract.filter (*segmented_cloud);

        
  writer.write<pcl::PointXYZRGBNormal> ("Segmentation.pcd", *segmented_cloud, false);


  return (0);
}
/* ]--- */
