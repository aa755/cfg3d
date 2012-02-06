/*
 * File:   ParseTreeLablerForm.cpp
 * Author: aa755
 *
 * Created on February 5, 2012, 9:20 PM
 */

#include "ParseTreeLablerForm.h"
#include "utils.h"

//#include "structures.cpp"
using namespace boost;

ParseTreeLablerForm::ParseTreeLablerForm()
{
    widget.setupUi(this);
}

ParseTreeLablerForm::~ParseTreeLablerForm()
{
}

void ParseTreeLablerForm::setUpTree(char * labelMapFile)
{
    std::ifstream fileI;
    std::string line;
    standardModel = new QStandardItemModel ;
    rootNode = standardModel->invisibleRootItem();

    set<int> segsWithLabel;
    fileI.open(labelMapFile);
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
            QStandardItem *item = new QStandardItem(QString("seg_")+QString("%1").arg(segNum)+QString(("_"+toks.at(0)).data())); 
            cout<<item->text().toUtf8().constData()<<endl;
            rootNode->appendRow(item);
        }
            widget.treeView->setModel(standardModel);
    }
    else
    {
        cout << "could not open the gt LabelMap file you specified ..exiting\n";
        exit(-1);
    }
    
}

pcl_visualization::PCLVisualizer viewer("3D Viewer");


void ParseTreeLablerForm::init(int argc, char** argv)
{
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


    setUpTree(argv[3]);


    // read from file
    if ( pcl::io::loadPCDFile<PointT > (argv[1], cloud_orig) == -1) {
        ROS_ERROR("Couldn't read file ");
        exit (-1);
    }
    
     QItemSelectionModel *selectionModel= widget.treeView->selectionModel();
//     connect(selectionModel, SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
//             this, SLOT(selectionChangedSlot(const QItemSelection &, const QItemSelection &)));

    // Convert to the templated message type
  //  pcl::fromROSMsg(cloud_blob_orig, cloud_orig);
   // pcl::PointCloud<PointT>::Ptr orig_cloud_ptr(new pcl::PointCloud<PointT > (cloud_orig));



//    get_sorted_indices(cloud_orig, segmentIndices);
//    sizeSortColors(segmentIndices, colorToSeg);

    


 //   color_handler_orig.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (cloud_blob_orig));

  //  viewer.addPointCloud(*orig_cloud_ptr, color_handler_orig, "orig");

    
}
int
main(int argc, char** argv) {

    ros::init(argc, argv, "labeler");

    
    QApplication app(argc, argv);

    ParseTreeLablerForm PTlabler;

    PTlabler.init(argc,argv);
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

