#include "ros/ros.h"
#include <rosbag/bag.h>
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
<<<<<<< HEAD

using namespace std;

void processPointCloud(/*const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,*/
        const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

    ROS_INFO("Received frame from kinect");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        
                pcl::fromROSMsg(*point_cloud, *cloud);
                
=======
#include "04_seg_sample_1.cpp"
#include <pcl/io/pcd_io.h>

using namespace std;

void computeAvgDist(const PointCloud<PointXYZRGB>::Ptr & cloud)
{
>>>>>>> 67d30c41e474063aa37e4ba321ac75e6bbce60be
        float sumx=0;
        int count=0;
        for(int i=0;i<(int)cloud->size();i++)
        {
            if(!isnan(cloud->points[i].z))
		{
            	sumx+=cloud->points[i].z;
		count++;
		}
        }
        cout<<sumx/count<<","<<count<<endl;
    
}
void processPointCloud(/*const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,*/
        const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

    static int counter=0;
    if(counter%20==0)
    ROS_INFO("Received frame from kinect");
    else
    ROS_INFO("Rejected frame from kinect");
        

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        
                pcl::fromROSMsg(*point_cloud, *cloud);
                computeAvgDist(cloud);
                
              //  sample1(cloud);
        
       // pcl::fromROSMsg(*point_cloud, cloud);
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_reader_tut");
    ros::NodeHandle n;
    
    ros::Subscriber cloud_sub_ = n.subscribe(argv[1], 1, processPointCloud);

    ros::spin();
  

}
