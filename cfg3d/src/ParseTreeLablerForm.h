/* 
 * File:   ParseTreeLablerForm.h
 * Author: aa755
 *
 * Created on February 5, 2012, 9:20 PM
 */

#ifndef _PARSETREELABLERFORM_H
#define	_PARSETREELABLERFORM_H

#include "ui_ParseTreeLablerForm.h"
 #include <QStandardItemModel>
 #include <QItemSelectionModel>
#include "PTNodeTableModel.h"
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
#include <cfg3d/labelerConfig.h>
#include "color.cpp"
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random/mersenne_twister.hpp>

typedef pcl::PointXYZRGBCamSL PointT;
using namespace boost;

class ParseTreeLablerForm : public QDialog {
    Q_OBJECT
    
public slots:
    void selectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/);
    void addNodeButtonClicked();
    void deleteNodeButtonClicked();
    void combineButtonClicked();
    void clearButtonClicked();
    void windowClosing();
    
public:
    const static int NUM_CLASSES_TO_SHOW=10;
    ParseTreeLablerForm();
    virtual ~ParseTreeLablerForm();
    Ui::ParseTreeLablerForm widget;
    QStandardItemModel * standardModel ;
    QStandardItem *rootNode ;
    std::map<int,string> label_mapping_orig;
    ColorRGB *labelColors[NUM_CLASSES_TO_SHOW];
    std::vector<int> segmentIndices;
pcl::PointCloud<PointT> cloud_orig;
pcl::PointCloud<PointT> cloud_colored;
pcl::PCDWriter writer;

pcl_visualization::PCLVisualizer viewer;
pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2>::Ptr color_handler;
sensor_msgs::PointCloud2 colored_cloud_blob;
PTNodeTableModel * nodeTableModel;
map<string,QStandardItem *> nameToTreeNode;
map<string,int> typeMaxId;
char *parseTreeFileName;
char* typeListFile;
 boost::uniform_int<> randSix;
     map<int,float> segNumToColor;
    boost::mt19937 rng;
    map<int,set<int> > nbrMap;
    
    
    void updateTypeCounts(string fullname);


    void setUpTree(char * labelMapFile);
    void readTree(char * treeFile);
    void writeTree(char * treeFile);
    
    void init(int argc, char** argv);
    
    void updatePCDVis();
    void colorSegs(map<int,float> & seg2color, bool fresh=false);
    float randColor();
    

};

#endif	/* _PARSETREELABLERFORM_H */
