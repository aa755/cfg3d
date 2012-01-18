/* 
 * File:   singleBag2PCD.cpp
 * Author: aa755
 *
 * Created on January 17, 2012, 8:18 PM
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
#include<boost/numeric/ublas/matrix.hpp>
#include<boost/numeric/ublas/io.hpp>
#include<boost/numeric/bindings/traits/ublas_matrix.hpp>
#include<boost/numeric/bindings/lapack/gels.hpp>
#include <boost/numeric/bindings/traits/ublas_vector2.hpp>


using namespace std;

pcl::PCDWriter writer;

using namespace std;

/** 
 * takes a bag file containing a single PCD and camera positions and writes a 
 * PCD file containing both
 */
int main(int argc, char** argv)
{
    if(argc<2 || argc >3)
    {
        cerr<<"usage:"<<argv[0]<<" bagFile [topic]"<<endl;
    }
  pcl_ros::BAGReader reader;
  const char * topic=string("/rgbdslam/my_clouds").data();
  
  if(argc==3)
      topic=argv[2];
  else
      cerr<<"using default topic "<<topic<<endl;
  
  if (!reader.open (argv[1], topic))
    {
      cerr << "given bag file either does not exist or does not contain a message of the given topic" << (topic);
      exit(-1);
    }
  
      rosbag::Bag bag;
      bag.open(argv[1], rosbag::bagmode::Read);
       sensor_msgs::PointCloud2ConstPtr cloud_blob;
       
      cloud_blob = reader.getNextCloud ();
      
      pcl::PointCloud<PointT> cloud;
      pcl::fromROSMsg (*cloud_blob, cloud);


        rosbag::View view_tf(bag, rosbag::TopicQuery("/tf"));//, ptime - ros::Duration(0, 1), ptime + ros::Duration(0, 100000000));
        int tf_count = 0;

        tf::Transform final_tft;

        BOOST_FOREACH(rosbag::MessageInstance const mtf, view_tf)
        {
            tf::tfMessageConstPtr tf_ptr = mtf.instantiate<tf::tfMessage > ();
            assert(tf_ptr != NULL);
            std::vector<geometry_msgs::TransformStamped> bt;
            tf_ptr->get_transforms_vec(bt);
            
            cloud.sensor_orientation_.x()=bt[0].transform.rotation.x;
            cloud.sensor_orientation_.y()=bt[0].transform.rotation.y;
            cloud.sensor_orientation_.z()=bt[0].transform.rotation.z;
            cloud.sensor_orientation_.w()=bt[0].transform.rotation.w;
            
           cloud.sensor_origin_(0)=bt[0].transform.translation.x;
           cloud.sensor_origin_(1)=bt[0].transform.translation.y;
           cloud.sensor_origin_(2)=bt[0].transform.translation.z;
            //tf::Transform tft(getQuaternion(bt[0].transform.rotation), getVector3(bt[0].transform.translation));

                tf_count++;
              //  final_tft = tft;
        }

        assert(tf_count == 1);

        writer.write(std::string(argv[1])+".pcd",cloud,true);

    return 0;
}

