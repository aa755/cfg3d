/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: segmentation_plane.cpp 35524 2011-01-26 08:20:18Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b segmentation_plane exemplifies how to run a Sample Consensus segmentation for planar models.

 **/
#include <iostream>
#include <boost/thread.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <stdint.h>
//#include <pcl_visualization/cloud_viewer.h>
#include "pcl_visualization/pcl_visualizer.h"
//#include "../../combine_clouds/src/CombineUtils.h"
#include "structures.cpp"

#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>
#include "utils.h"
#include "pcl/io/pcd_io.h"
#include "point_types.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic_reconfigure/server.h>
#include <cfg3d/labelerConfig.h>
#include "color.cpp"
#define NUM_CLASSES_TO_SHOW 10

//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef pcl::PointXYZRGBCamSL PointT;
using namespace boost;
dynamic_reconfigure::Server < cfg3d::labelerConfig > *srv;
cfg3d::labelerConfig conf;

boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportOrig = 0;
int viewportPred = 0;
std::vector<std::string> labels;

map<int, set<int> > neighbors;

sensor_msgs::PointCloud2 cloud_blob_orig;
sensor_msgs::PointCloud2 cloud_blob_filtered_orig;


pcl::PointCloud<PointT> cloud_orig;
pcl::PCDWriter writer;
ColorHandlerPtr color_handler_orig;
std::map<int,int > label_mapping_orig;

bool doUpdate = false;

float
sqrG(float y) {
    return y*y;
}
//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;

void get_sorted_indices(pcl::PointCloud<PointT> &incloud, std::vector<int> &segmentindices) {
    set<int> segs;
    for(unsigned int i=0;i<incloud.size();i++)
    {
        segs.insert(incloud.points.at(i).segment);
    }
    segs.erase(0);
    segmentindices.insert(segmentindices.begin(),segs.begin(),segs.end());
}


bool apply_label_filter(pcl::PointCloud<PointT> &incloud, int label, float color) {
    ROS_INFO("applying filter");
    bool changed = false;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].label == label) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            incloud.points[i].rgb = color;
            changed = true;
        }
    }
    return changed;
}
void writeNbrMap(string filename)
{
    ofstream ofile;
    ofile.open(filename.data(),ios::out);
        map<int, set<int> >::iterator it;

        for ( it=neighbors.begin() ; it != neighbors.end(); it++ )
        {
                ofile<<it->first;
            set<int> & segNbrs =it->second;
            set<int>::iterator sit;
            for(sit=segNbrs.begin();sit!=segNbrs.end();sit++)
                ofile<<","<<*sit;
            ofile<<endl;
        }

}

bool apply_label_filter(pcl::PointCloud<PointT> &incloud, vector<int> labels, float color) {
    ROS_INFO("applying filter");
    bool changed = false;

    for (size_t l = 0; l < labels.size(); l++) {
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].label == labels[l]) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            incloud.points[i].rgb = color;
            changed = true;
        }
    }
    }
    return changed;
}

bool apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    bool changed = false;

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points = incloud.points;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].segment == segment) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            outcloud.points[i].rgb = 0.00001;
            changed = true;
        }
    }
    return changed;
}

bool color_segment(pcl::PointCloud<PointT> &incloud, int segment, float color) 
{
    ROS_INFO("applying filter %d",segment);
    bool changed = false;


    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if ((int)incloud.points[i].segment == segment) {
            incloud.points[i].rgb = color;
            changed = true;
        }
    }
    return changed;
}


void spinThread() {
    std::cerr << "thread started";
    while (true)
        viewer.spinOnce(1000, true);
}

vector <string> getTokens(std::string str)
{
        char_separator<char> sep(",");
        tokenizer<char_separator<char> > tokens(str, sep);
       vector<string> out; 
        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(t);
        }
	return out;
}

        ColorRGB *labelColors[NUM_CLASSES_TO_SHOW];
    std::vector<int> segmentIndices;
    vector<int> colorToSeg;
    string *selLabels[NUM_CLASSES_TO_SHOW];

    void randomizeColors(vector<int> & segmentIndices, vector<int> & color2Seg )
    {
        color2Seg.resize(NUM_CLASSES_TO_SHOW);
        vector<int> temp= segmentIndices; // clone it for modification
        int randIndex;
        for(int i=0;i<NUM_CLASSES_TO_SHOW;i++)
        {
            if(temp.empty())
                break;
            
            randIndex=rand() % temp.size();
            color2Seg[i]=temp.at(randIndex);
            temp.erase(temp.begin()+randIndex);
        }
        
    }
    
    void randomizeColors(vector<int> & segmentIndices, vector<int> & color2Seg , int label)
    {
        color2Seg.resize(NUM_CLASSES_TO_SHOW);
        vector<int> temp;

           map<int,int>::iterator it;
           for(it=label_mapping_orig.begin();it!=label_mapping_orig.end();it++)
           {
               if(it->second==label)
                   temp.push_back(it->first);
               
           }
        
        int randIndex;
        for(int i=0;i<NUM_CLASSES_TO_SHOW;i++)
        {
            if(temp.empty())
            {
                color2Seg[i]=-1; // no segment
                continue;
            }
            randIndex=rand() % temp.size();
            color2Seg[i]=temp.at(randIndex);
            temp.erase(temp.begin()+randIndex);
        }
        
    }
    
    void sizeSortColors(vector<int> & segmentIndices, vector<int> & color2Seg )
    {
        color2Seg.resize(NUM_CLASSES_TO_SHOW);
        for(unsigned int i=0;i<segmentIndices.size();i++)
        {
            if(i==NUM_CLASSES_TO_SHOW)
                break;
            color2Seg.at(i)=segmentIndices[i];
        }
    }

void reconfig(cfg3d::labelerConfig & config, uint32_t level)
{
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
    pcl::PointCloud<PointT> cloud_colored_orig;
    selLabels[0] = &conf.red_label;
    selLabels[1] = &conf.green_label;
    selLabels[2] = &conf.blue_label;
    selLabels[3] = &conf.yellow_label;
    selLabels[4] = &conf.cyan_label;
    selLabels[5] = &conf.magenta_label;
    selLabels[6] = &conf.dark_red_label;
    selLabels[7] = &conf.dark_green_label;
    selLabels[8] = &conf.dark_blue_label;
    selLabels[9] = &conf.dark_yellow_label;

    for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++)
    {
        viewer.removeShape(*selLabels[color]);
    }


    viewer.setBackgroundColor(1.0, 1.0, 1.0);

    cloud_colored_orig = cloud_orig;
    viewer.removePointCloud("orig");

    if (conf.show_labels)
    {
        conf.show_labels = false;
        conf.show_segments = false;
        doUpdate = true;

        for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++)
        {

            *selLabels[color] = labels.at(label_mapping_orig[colorToSeg.at(color)]);

        }

    }

    if (conf.show_segments)
    {
        conf.show_labels = false;
        conf.show_segments = false;
        doUpdate = true;

        for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++)
        {
            *selLabels[color] = boost::lexical_cast<std::string > (colorToSeg.at(color));
        }
    }

    if (conf.accept_labels)
    {
        conf.accept_labels=false;
        doUpdate=true;
        string message="notFound:";
        for(int i=0;i<NUM_CLASSES_TO_SHOW;i++)
        {
            if(colorToSeg[i]==-1)
                continue;
            
            bool found=false;
            for(unsigned int j=0;j<labels.size();j++)
            {
                if(selLabels[i]->compare(labels.at(j))==0)
                {
                    found=true;
                    label_mapping_orig[colorToSeg[i]]=j;
                }
            }
            
            if(!found)
            {
                message.append(",");
                message.append(*selLabels[i]);
            }
        }
        conf.message=message;
    }

    if (conf.merge_preview)
    {
        doUpdate=true;
            *selLabels[0] = conf.merge1;
            *selLabels[1] = conf.merge2;
            colorToSeg[0] = boost::lexical_cast<int>(conf.merge1);
            colorToSeg[1] = boost::lexical_cast<int>(conf.merge2);
        //it is set false later
    }

    if(conf.merge)
    {
        conf.merge=false;
        doUpdate=true;
            int seg1 = boost::lexical_cast<int>(conf.merge1);
            int seg2 = boost::lexical_cast<int>(conf.merge2);
            int segSmall,segBig;
            segSmall=min(seg1,seg2);
            segBig=max(seg1,seg2);
            if(segSmall==segBig)
                
            {
                conf.message="both segments are same";                
                return;
            }
                
            
            //erase the bigger index
            for(vector<int>::iterator it=segmentIndices.begin();it!=segmentIndices.end();it++)
            {
                if(*it==segBig)
                {
                    segmentIndices.erase(it);
                    break;
                }
            }
            // remap segments of bigger index to smaller index
            
            for(vector<PointT, Eigen::aligned_allocator<PointT> >::iterator it=cloud_orig.points.begin() ; it!=cloud_orig.points.end(); it++)
            {
                if((int)(*it).segment==segBig)
                    (*it).segment=segSmall;
            }
            
            //fix map
            
            neighbors.erase(segBig);
            
        map<int, set<int> >::iterator it;

        for ( it=neighbors.begin() ; it != neighbors.end(); it++ )
        {
            set<int> & nbrs =it->second;
            if(nbrs.find(segBig)!=nbrs.end())
                nbrs.insert(segSmall);
            
            nbrs.erase(segBig);
        }
            
                randomizeColors(segmentIndices, colorToSeg);        
        }
    
    if (conf.randomize)
    {
        randomizeColors(segmentIndices, colorToSeg);
        conf.randomize = false;
        doUpdate=true;

        for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++)
        {
            *selLabels[color] = boost::lexical_cast<std::string > (colorToSeg.at(color));
        }
    }
    
    if (conf.showOnlyLabel)
    {
        conf.showOnlyLabel = false;
        int index=-1;
        for(unsigned int j=0;j<labels.size();j++)
            {
                if(conf.label.compare(labels.at(j))==0)
                {
                    index=j;
                }
            }
        
        if(index!=-1)
            randomizeColors(segmentIndices, colorToSeg,index);
        else
            conf.message="no segment has that label";
        
        doUpdate=true;
    }

    if (conf.add_new_label)
    {
        randomizeColors(segmentIndices, colorToSeg);
        conf.add_new_label = false;
        doUpdate=true;

        for(unsigned int j=0;j<labels.size();j++)
            {
                if(conf.label.compare(labels.at(j))==0)
                {
                    conf.message="new label already exists!";
                    doUpdate=true;
                    break;
                }
            }
        labels.push_back(conf.label);
    }

    float charCount = 0.4;
    for (size_t color = 0; color < NUM_CLASSES_TO_SHOW; color++)
    {
        //cout<<labelColors[color]->r*255.0<<labelColors[color]->g*255.0<<labelColors[color]->b*255.0<<endl;
        if(conf.merge_preview&&color==2)
        {
            conf.merge_preview=false;
            break;
        }
        viewer.addText(*selLabels[color], charCount * 11, 50, labelColors[color]->r, labelColors[color]->g, labelColors[color]->b);
        charCount = charCount + selLabels[color]->size() + 0.9;
        color_segment(cloud_colored_orig, colorToSeg.at(color), labelColors[color]->getFloatRep());
    }

    pcl::toROSMsg(cloud_colored_orig, cloud_blob_filtered_orig);
    color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered_orig));
    viewer.addPointCloud(cloud_colored_orig, color_handler_orig, "orig");

}

int
main(int argc, char** argv) {

    ros::init(argc, argv, "labeler");
    if(argc!=4 && argc!=5)
    {
        cerr<<"usage:"<<argv[0]<<" <segmented_PCD> <neighborMap> <labelsFile> [segLabelMap]"<<endl;
        exit(-1);
    }
        labelColors[0]= new ColorRGB(1,0,0);
        labelColors[1]= new ColorRGB(0,1,0);
        labelColors[2]= new ColorRGB(0,0,1);
        labelColors[3]= new ColorRGB(1,1,0);
        labelColors[4]= new ColorRGB(0,1,1);
        labelColors[5]= new ColorRGB(1,0,1);
        labelColors[6]= new ColorRGB(0.5,0,0);
        labelColors[7]= new ColorRGB(0,0.5,0);
        labelColors[8]= new ColorRGB(0,0,0.5);
        labelColors[9]= new ColorRGB(0.5,0,0.5);


    std::ifstream fileI;
    std::string line;
    char *labelFileC=argv[3];
    fileI.open(labelFileC);

            labels.clear();
            labels.push_back("n");
    if (fileI.is_open()) {
        int count = 1;
        while (fileI.good()) {
            getline(fileI, line); //each line is a label
            if (line.size() == 0)
                break;
            cout << "adding label " << line << " with value:" << count << endl;
            count++;
            labels.push_back(line);
        }
    } else {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }

    fileI.close();

    if (argc == 5)
    {
        fileI.open(argv[4]);
        if (fileI.is_open())
        {
            while (fileI.good())
            {
                getline(fileI, line); //each line is a label
                if (line.size() == 0)
                    break;

                vector<int> nums;
                getTokens(line, nums);
                label_mapping_orig[nums.at(0)] = nums.at(1);

            }
        }
        else
        {
            cout << "could not open the segLabelMap file you specified .it is optional..exiting\n";
            exit(-1);
        }


    }


    // read from file
    if ( pcl::io::loadPCDFile<PointT > (argv[1], cloud_orig) == -1) {
        ROS_ERROR("Couldn't read file ");
        return (-1);
    }
    


    // Convert to the templated message type
  //  pcl::fromROSMsg(cloud_blob_orig, cloud_orig);
   // pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));



    get_sorted_indices(cloud_orig, segmentIndices);
    sizeSortColors(segmentIndices, colorToSeg);

    


 //   color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_orig));

  //  viewer.addPointCloud(*orig_cloud_ptr, color_handler_orig, "orig");


    
    srv = new dynamic_reconfigure::Server < cfg3d::labelerConfig > (global_mutex);
    dynamic_reconfigure::Server < cfg3d::labelerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


    srv->setCallback(f);
    conf.message="all is well";
    conf.accept_labels=false;
    conf.add_new_label=false;
    conf.exit=false;
    conf.merge=false;
    conf.merge_preview=false;
    conf.randomize=false;
    conf.show_labels=false;
    int maxSegIndex= parseNbrMap(argv[2],neighbors);
    
    bool isDone = false;
    //ROS_INFO ("Press q to quit.");
    while (!isDone) {
        viewer.spinOnce();
        ros::spinOnce();
        if (conf.exit) {
            conf.exit = false;
            srv->updateConfig(conf);
            
            //savePCDAndLabels ();
            ofstream fileO;
            fileO.open(labelFileC);
            for(int i=1;i<(int)labels.size();i++) // ignore the 1st one "n"
            {
                fileO<<labels.at(i)<<endl;
            }
            fileO.close();
            
            string pcdFile(argv[1]);
            string pcdFileName=pcdFile.substr(0,pcdFile.length()-4);
            cout<<pcdFileName<<endl;
            cout<<pcdFile<<endl;
            
           pcl::io::savePCDFile<PointT>(pcdFileName+"_merged.pcd" , cloud_orig,true);
           
            fileO.open((pcdFileName+"_labelmap.txt").data());
           map<int,int>::iterator it;
           for(it=label_mapping_orig.begin();it!=label_mapping_orig.end();it++)
           {
               if(it->second==0)
                   continue;
               
               fileO<<it->first<<","<<labels.at(it->second)<<endl;
           }
           fileO.close();
           cout<<"done saving"<<endl;
           string nbrmap(argv[2]);
           writeNbrMap(nbrmap+"_merged.txt");           
           
            break;
        }
        if (doUpdate) {
            doUpdate = false;
            srv->updateConfig(conf);
        }
    }

    cout << "normal kill";
    return (0);


}
