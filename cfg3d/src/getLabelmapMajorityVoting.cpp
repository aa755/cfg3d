/* 
 * File:   getLabelmapMajorityVoting.cpp
 * Author: aa755
 *
 * Created on January 18, 2012, 6:44 PM
 */

#include <cstdlib>
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

/*
 * 
 */

const int AMBIGOUS_LABEL=125; 
int getMajorityLabel(const pcl::PointCloud<PointT> &cloud){
    map<int,int> label_count_map;
    int max_count=0;
    int max_label=0;
    int second_max_label=0;
    int second_max_count =0;

    for (size_t i = 0; i < cloud.points.size(); ++i) {
        label_count_map[cloud.points[i].label] ++;
    }
    multimap<int,int> count_label_map;
    for (map<int, int>::iterator it= label_count_map.begin(); it != label_count_map.end(); it++)
    {
        if((*it).first!=0&&(*it).first!=8 && (*it).first!=19&& (*it).first!=17)
        count_label_map.insert(pair<int,int>((*it).second,(*it).first));
    }
    multimap<int, int>::reverse_iterator it = count_label_map.rbegin();
    if (it != count_label_map.rend()) {
        max_count = (*it).first;
        max_label = (*it).second;
        it++;

        if (it != count_label_map.rend()) {
            second_max_count = (*it).first;
            second_max_label = (*it).second;
        }
    }
    //cout << "max_count:" << max_count << " second_max_count:" << second_max_count << " segment_size:" << cloud.points.size() << endl;
    if (max_count > 20 )
  //  if (max_count > cloud.points.size()/10 )
    {
        if( (max_count - second_max_count)> max_count/2 )
             return max_label;
        else
        {
            cerr<<max_label<<":"<<max_count<<","<<second_max_label<<":"<<second_max_count<<",";
            return AMBIGOUS_LABEL;//ambigous label
        }
    }
    return 0;
}

void apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    //ROS_INFO("applying filter");

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
//    outcloud.points = incloud.points;
    outcloud.points.resize ( incloud.points.size() );

    int j = -1;
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].segment == segment) {
          j++;
          outcloud.points[j].x = incloud.points[i].x;
          outcloud.points[j].y = incloud.points[i].y;
          outcloud.points[j].z = incloud.points[i].z;
          outcloud.points[j].rgb = incloud.points[i].rgb;
          outcloud.points[j].segment = incloud.points[i].segment;
          outcloud.points[j].label = incloud.points[i].label;
          outcloud.points[j].cameraIndex = incloud.points[i].cameraIndex;
          outcloud.points[j].distance = incloud.points[i].distance;

            //     std::cerr<<segment_cloud.points[j].label<<",";
        }
    }
    
    if(j>=0)
        outcloud.points.resize ( j+1 );
    else
       outcloud.points.clear ();
}

void readLabelsFile(const char * filename, vector<string> & labels)
{
    std::ifstream labelFile;
    string line;
    labelFile.open(filename);
        if (labelFile.is_open()) {
        int count = 1;
        while (labelFile.good()) {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0)
                break;
       //     cout << "adding label " << line << " with value:" << count << endl;
            count++;
            labels.push_back(line);
        }
    } else {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }
    
}

int main(int argc, char** argv)
{
    if(argc!=3)
        cerr<<"usage:"<<argv[0]<<"PCDFile labelsFile"<<endl;
    
    pcl::PointCloud<PointT> src;
    pcl::PointCloud<PointT> segment;
    pcl::io::loadPCDFile<PointT>(argv[1], src);

    vector<string> labels;
    
    readLabelsFile(argv[2],labels);

    map<int,set<int> > label2Segs;

    int segNum=1;
   // for(int i=0;i<src.size();i++)
     //   cout<<src.points[i].segment<<endl;
    
    while(true)
    {
        apply_segment_filter(src,segment,segNum);
        
        if(segment.size()==0)
            break;
        cout<<segNum<<","<<segment.size()<<endl;
        int label=getMajorityLabel(segment);
        if(label!= 0 && label!=AMBIGOUS_LABEL)
                label2Segs[label].insert(segNum);
                
        segNum++;
    }
    
    ofstream labelMap;
    string labelMapFileName=string(argv[1])+"_labelmap_gt.txt";
    labelMap.open(labelMapFileName.data(),ios::out);
    for(map<int,set<int> >::iterator it=label2Segs.begin();it!=label2Segs.end();it++)
    {
        labelMap<<labels.at(it->first-1);
        for(set<int>::iterator its=it->second.begin();its!=it->second.end();its++)
        {
            labelMap<<","<<*its;
        }
        labelMap<<endl;
    }

    return 0;
}

