#include <iostream>
#include <boost/thread.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <stdint.h>
//#include <pcl_visualization/cloud_viewer.h>
#include "pcl_visualization/pcl_visualizer.h"
//#include "../../combine_clouds/src/CombineUtils.h"


#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/tree_types.h"
#include <pcl/features/normal_3d.h>

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
#include <cfg3d/labelviewerConfig.h>
#include "color.h"

//typedef pcl::PointXYZRGB PointT;
//std::string initLabels[]={"wall","floor","table","shelf","chair","cpu","monitor","clutter"};

typedef pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef pcl::PointXYZRGBCamSL PointT;
using namespace boost;
dynamic_reconfigure::Server < cfg3d::labelviewerConfig > *srv;
cfg3d::labelviewerConfig conf;
boost::recursive_mutex global_mutex;
pcl_visualization::PCLVisualizer viewer("3D Viewer");
int viewportOrig = 0;
int viewportPred = 0;
std::vector<std::string> labels;

sensor_msgs::PointCloud2 cloud_blob_orig;
sensor_msgs::PointCloud2 cloud_blob_pred;
sensor_msgs::PointCloud2 cloud_blob_filtered_orig;
sensor_msgs::PointCloud2 cloud_blob_filtered_pred;


pcl::PointCloud<PointT> cloud_orig;
pcl::PointCloud<PointT> cloud_pred;
pcl::PCDWriter writer;
ColorHandlerPtr color_handler_orig;
ColorHandlerPtr color_handler_pred;

pcl::PointCloud<PointT>::Ptr cloud_colored_orig(new pcl::PointCloud<PointT > ());
map<int, set<int> > neighbors;

bool doUpdate = false;

float
sqrG(float y) {
    return y*y;
}
//typedef my_ns::MyPoint PointT;
using namespace pcl_visualization;

void get_sorted_indices(pcl::PointCloud<PointT> &incloud, std::vector<int> &segmentindices, int size) {
    std::map<int, int> countmap;
    for (int i = 1; i <= size; i++) {
        countmap[i] = 0;
    }
    for (size_t i = 0; i < incloud.points.size(); ++i) {
        countmap[incloud.points[i].segment ] = countmap[incloud.points[i].segment ] + 1;
    }
    std::multimap<int, int> inverted_countmap;
    for (std::map<int, int>::iterator it = countmap.begin(); it != countmap.end(); it++)
        inverted_countmap.insert(std::pair<int, int>(it->second, it->first));
    for (std::multimap<int, int>::reverse_iterator rit = inverted_countmap.rbegin(); rit != inverted_countmap.rend(); rit++)
        segmentindices.push_back(rit->second);

}


bool apply_segment_filter(pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointT> &outcloud, int segment) {
    ROS_INFO("applying filter");
    bool changed = false;

    outcloud.points.erase(outcloud.points.begin(), outcloud.points.end());

    outcloud.header.frame_id = incloud.header.frame_id;
    outcloud.points = incloud.points;


    ColorRGB green(0,1,0);
    ColorRGB red(1,0,0);
        set<int> & nbrs=neighbors[segment];
        cout<<"segment:"<<segment<<", neighbors:";
        set<int>::iterator it;
        for(it=nbrs.begin();it!=nbrs.end();it++)
            cout<<*it<<",";
        cout<<endl;
    
    for (size_t i = 0; i < incloud.points.size(); ++i) {

        if (incloud.points[i].segment == (uint32_t)segment) {

            //     std::cerr<<segment_cloud.points[j].label<<",";
            outcloud.points[i].rgb = green.getFloatRep();
            changed = true;
        }
        else if(nbrs.find(incloud.points[i].segment)!=nbrs.end())
        {
            outcloud.points[i].rgb = red.getFloatRep();
            changed = true;
            
        }
    }
    return changed;
}


void getTokens(std::string str,vector <string> & out)
{
        char_separator<char> sep(",");
        tokenizer<char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(t);
        }
        
}

void getTokens(std::string str,vector<int> & out)
{
        char_separator<char> sep(",");
        tokenizer<char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(boost::lexical_cast<int>(t));
        }
        
}

void reconfig(cfg3d::labelviewerConfig & config, uint32_t level) {
    conf = config;
    boost::recursive_mutex::scoped_lock lock(global_mutex);
    pcl::PointCloud<PointT>::Ptr pred_cloud_ptr(new pcl::PointCloud<PointT > (cloud_pred));
    pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));
    bool c = false;

//    int NUM_CLASSES_TO_SHOW=10;
//    int labelNum = 0;

    if (conf.showSegment) {
        conf.showSegment = false;
        doUpdate = true;

            viewer.removePointCloud("orig");

            c = apply_segment_filter(*orig_cloud_ptr, *cloud_colored_orig, conf.segmentNum);


            pcl::toROSMsg(*cloud_colored_orig, cloud_blob_filtered_orig);
            color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_filtered_orig));
            viewer.addPointCloud(*cloud_colored_orig, color_handler_orig, "orig", viewportOrig);


    }

}

/* ---[ */
int
main(int argc, char** argv) {

    ros::init(argc, argv, "labelviewer");
//    bool groundSelected = false;
//    bool editLabel = false;
//    int targetLabel;

    boost::numeric::ublas::matrix<double> outMat(4, 4);

    std::ifstream labelFile;
    std::string line;
    labelFile.open(argv[2]);

    std::cerr << "you can only quit by pressing 9 when the prompt mentions... quitting in other ways will discard any newly added labels\n";
    vector<int> nbrs;
    
    
    if (labelFile.is_open()) {
//        int count = 1;
        while (labelFile.good()) {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0)
                break;
            
            getTokens(line, nbrs);
            int segIndex=nbrs.at(0);
            set<int> temp;
            neighbors[segIndex]=temp;
            
            for(size_t i=1;i<nbrs.size();i++)
            {
                neighbors[segIndex].insert(nbrs.at(i));
                cout<<"adding "<<nbrs.at(i)<<" as a neighbos of "<<segIndex<<endl;
            }
        }
    } else {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }



    // read from file
    if (pcl::io::loadPCDFile(argv[1], cloud_blob_orig) == -1) {
        ROS_ERROR("Couldn't read file");
        return (-1);
    }
    ROS_INFO("Loaded %d data points from %s with the following fields: %s", (int) (cloud_blob_orig.width * cloud_blob_orig.height), argv[1], pcl::getFieldsList(cloud_blob_orig).c_str());



    // Convert to the templated message type
    pcl::fromROSMsg(cloud_blob_orig, cloud_orig);
    pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));




    std::vector<int> segmentIndices;
    // get_sorted_indices(*cloud_ptr, segmentIndices, max_segment_num);

    


    viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewportOrig);
    color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_orig));

    viewer.addPointCloud(*orig_cloud_ptr, color_handler_orig, "orig", viewportOrig);


    srv = new dynamic_reconfigure::Server < cfg3d::labelviewerConfig > (global_mutex);
    dynamic_reconfigure::Server < cfg3d::labelviewerConfig >::CallbackType f = boost::bind(&reconfig, _1, _2);


    srv->setCallback(f);
    conf.done = false;

    bool isDone = false;
    //ROS_INFO ("Press q to quit.");
    while (!isDone) {
        viewer.spinOnce();
        ros::spinOnce();
        if (conf.done) {
            conf.done = false;
            srv->updateConfig(conf);
            //savePCDAndLabels ();
            break;
        }
        if (doUpdate) {
            doUpdate = false;
            srv->updateConfig(conf);
        }
    }

    string filename=string(argv[1]).append(".labelColored.pcd");
    //writer.write<PointT > (filename, *cloud_colored_orig, true);
    cout << "normal kill";





    
    viewer.removePointCloud("orig");


    return (0);
}
/* ]--- */
