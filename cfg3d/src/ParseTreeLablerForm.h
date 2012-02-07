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
typedef pcl::PointXYZRGBCamSL PointT;
using namespace boost;

class ParseTreeLablerForm : public QDialog {
    Q_OBJECT
    
public slots:
    void selectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/);
    void addNodeButtonClicked();
    void combineButtonClicked();
    void clearButtonClicked();
    
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
    std::vector<std::string> labels;
pcl::PointCloud<PointT> cloud_orig;
pcl::PointCloud<PointT> cloud_colored;
pcl::PCDWriter writer;

pcl_visualization::PCLVisualizer viewer;
pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2>::Ptr color_handler;
sensor_msgs::PointCloud2 colored_cloud_blob;
PTNodeTableModel * nodeTableModel;
map<string,QStandardItem *> nameToTreeNode;
map<string,int> typeCounts;
    
    void setUpTree(char * labelMapFile);
    
    void init(int argc, char** argv);
    
    void updatePCDVis();
    void colorSegs(const set<int>& segs, bool fresh=false);
    
    

};

#endif	/* _PARSETREELABLERFORM_H */
