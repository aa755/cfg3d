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
#include <QtGui/QApplication>
#include "ParseTreeLablerForm.h"
 #include <QStandardItemModel>
 #include <QItemSelectionModel>
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
#include <qt4/QtGui/qtreeview.h>
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
std::map<int,string> label_mapping_orig;

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
    
    void randomizeColors(vector<int> & segmentIndices, vector<int> & color2Seg , string label)
    {
        color2Seg.resize(NUM_CLASSES_TO_SHOW);
        vector<int> temp;

           map<int,string>::iterator it;
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


int
main(int argc, char** argv) {

    ros::init(argc, argv, "labeler");
    if(argc!=5 && argc!=6)
    {
        cerr<<"usage:"<<argv[0]<<" <segmented_PCD> <neighborMap> <labelmap> <typenames> [parseTree]"<<endl;
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
    char *labelFileC=argv[4];
    fileI.open(labelFileC);

            labels.clear();
            labels.push_back("n");
    if (fileI.is_open()) {
        int count = 1;
        while (fileI.good()) {
            getline(fileI, line); //each line is a label
            if (line.size() == 0)
                break;
            cout << "adding typename " << line  << endl;
            count++;
            labels.push_back(line);
        }
    } else {
        cout << "could not open typenames file...exiting\n";
        exit(-1);
    }

    fileI.close();

    QStandardItemModel * standardModel = new QStandardItemModel ;
    QStandardItem *rootNode = standardModel->invisibleRootItem();

    set<int> segsWithLabel;
    fileI.open(argv[3]);
    if (fileI.is_open())
    {
        while (fileI.good())
        {
            getline(fileI, line); 
            if (line.size() == 0)
                break;

            vector<string> toks;
            getTokens(line, toks);
            int segNum=lexical_cast<int>(toks.at(1));
            label_mapping_orig[segNum] = toks.at(0);
            segsWithLabel.insert(segNum);
            QStandardItem *item = new QStandardItem(QString("seg_")+QString((toks.at(0)+"_").data())+QString("%1").arg(segNum));        
            rootNode->appendRow(item);
        }
    }
    else
    {
        cout << "could not open the gt LabelMap file you specified ..exiting\n";
        exit(-1);
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


    
    QApplication app(argc, argv);

    ParseTreeLablerForm PTlabler;

    PTlabler.widget.treeView->setModel(standardModel);
    PTlabler.show();


    
    bool isDone = false;
    
    //ROS_INFO ("Press q to quit.");
    while (!isDone) {
        viewer.spinOnce();
        QCoreApplication::sendPostedEvents();
        QCoreApplication::processEvents();
    }

    cout << "normal kill";
    return (0);


}
