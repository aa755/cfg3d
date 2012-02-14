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

void processPointCloud(/*const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::CameraInfoConstPtr& cam_info,*/
        const sensor_msgs::PointCloud2ConstPtr& point_cloud) {

    static int callback_counter_ = 0;
    callback_counter_++;
    ROS_INFO("Received frame from kinect");
        ROS_INFO("accepted it");

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*point_cloud, cloud);
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_reader_tut");
    ros::NodeHandle n;
    
    ros::Subscriber cloud_sub_ = n.subscribe(argv[1], 1, processPointCloud);

    ros::spin();
  

}
