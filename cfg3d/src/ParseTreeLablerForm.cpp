/*
 * File:   ParseTreeLablerForm.cpp
 * Author: aa755
 *
 * Created on February 5, 2012, 9:20 PM
 */

#include "ParseTreeLablerForm.h"
#include "utils.h"
#include "qt4/QtCore/qstring.h"
#include "PTNodeTableModel.h"
#include <queue>
//#include "structures.cpp"

ParseTreeLablerForm::ParseTreeLablerForm() : viewer("3DViewer"), randSix(0,5)
{
    widget.setupUi(this);
}

ParseTreeLablerForm::~ParseTreeLablerForm()
{
}
string getCppString(QString str)
{
    return string(str.toUtf8().constData());
}
class Node
{
    string type;
    int id;
    string memo;
public:
    Node(string fullname)
    {
        int typeEnd=fullname.find("__");
        type=fullname.substr(0,typeEnd);
        int idEnd=fullname.find("__",typeEnd+2);
        
        if(idEnd==(int)string::npos)
            idEnd=fullname.size();
        cout<<"++"<<fullname.substr(typeEnd+2,idEnd)<<"++"<<endl;
        id=lexical_cast<int>(fullname.substr(typeEnd+2,idEnd));
        memo=fullname.substr(idEnd);
        cout<<type<<"-"<<id<<"-"<<memo<<endl;
    }
};

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
            string name="seg_"+lexical_cast<string>(segNum)+"_"+toks.at(0);
            QStandardItem *item = new QStandardItem(name.data()); 
            nameToTreeNode[name]=item;
            
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

float ParseTreeLablerForm::randColor()
{
    return labelColors[randSix(rng)]->getFloatRep();
}

void ParseTreeLablerForm::readTree(char * treeFile)
{
    std::ifstream fileI;
    std::string line;
    standardModel = new QStandardItemModel ;
    rootNode = standardModel->invisibleRootItem();

    set<int> segsWithLabel;
    fileI.open(treeFile);
    getline(fileI, line);
    assert(line.substr(0,7)=="digraph");
    
    if (fileI.is_open())
    {
        while (fileI.good())
        {
            getline(fileI, line); 
            if (line.at(0) == '}')
                break;

            
            vector<string> toks;
            getTokens(line, toks,"\t ->;");
//            for(vector<string>::iterator it=toks.begin();it!=toks.end();it++)
//            {
//                if((*it).size()!=0)
//                        cout<<*it<<",,";
//            }
//            cout<<endl;
            if (toks.size() == 1)
            {
                //orphan
                string name = toks.at(0);
                Node nd(name);
                QStandardItem *item = new QStandardItem(name.data());
                nameToTreeNode[name] = item;

                rootNode->appendRow(item);

            }
            else if (toks.size() == 2)
            {
                // with a parent
                string name = toks.at(1);
                Node nd(name);
                QStandardItem *item = new QStandardItem(name.data());
                nameToTreeNode[name] = item;
                QStandardItem *parent = nameToTreeNode[toks.at(0)];
                assert(parent != NULL);
                parent->appendRow(item);

            }
            else
            {
                assert(false);
            }
        }
            widget.treeView->setModel(standardModel);
        
    }
    else
    {
        cout << "could not open the gt LabelMap file you specified ..exiting\n";
        exit(-1);
    }
}

void ParseTreeLablerForm::writeTree(char * treeFile)
{
    cerr<<"writing the parse tree"<<endl;
    rootNode = standardModel->invisibleRootItem();
    ofstream ofile;
    ofile.open(treeFile,ios::out);
    ofile<<"digraph g{\n";
    queue<QStandardItem *> bfsQueue;
    bfsQueue.push(rootNode);
    while(!bfsQueue.empty())
    {
        QStandardItem *curNode=bfsQueue.front();
        int numChildren=curNode->rowCount();
        string parName="";
        if(curNode!=rootNode)
        {
            parName=getCppString(curNode->text())+" -> ";
        }
        cout<<parName<<endl;
        for(int i=0;i<numChildren;i++)
        {
            QStandardItem *child=curNode->child(i,0);
            ofile<<parName<<getCppString(child->text())<<" ;"<<endl;
          //  cout<<getCppString(child->text())<<endl;
            bfsQueue.push(child);
        }
        bfsQueue.pop();
    }
    ofile<<"}";
    ofile.close();
    
}

void ParseTreeLablerForm::updatePCDVis()
{
    viewer.removePointCloud("colored");    
    
    pcl::toROSMsg(cloud_colored, colored_cloud_blob);

    color_handler.reset(new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2 > (colored_cloud_blob));

    viewer.addPointCloud(cloud_colored, color_handler, "colored");    
}

void ParseTreeLablerForm::colorSegs(map<int,float> & seg2color, bool fresh) {
    ROS_INFO("color segs");
    if(fresh)
    {
        cloud_colored=cloud_orig;
    }



    ColorRGB green(0,1,0);
//    ColorRGB red(1,0,0);
//        for(set<int>::iterator it=segs.begin();it!=segs.end();it++)
//            cout<<*it<<",";
//        cout<<endl;
    
    for (size_t i = 0; i < cloud_colored.points.size(); i++) {
        PointT & pt= cloud_colored.points[i];
        if(seg2color.find(pt.segment)!=seg2color.end())
        {
            pt.rgb = seg2color[pt.segment];
        }
    }
}

void ParseTreeLablerForm::selectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/)
{
    //get the text of the selected item
    const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
    QString selectedText = index.data(Qt::DisplayRole).toString();
    string name = getCppString(selectedText);
    QStandardItem *parNode = nameToTreeNode[name];
    int numChildren = parNode->rowCount();
        segNumToColor.clear();
            if (name.substr(0, 10) == "Terminal__")
            {
                int end = name.find("_", 10);
                cout << "selseg:" << name.substr(10, end - 10) << endl;
                int segment = lexical_cast<int>(name.substr(10, end - 10));

                segNumToColor[segment] = randColor();
            }

    for (int i = 0; i < numChildren; i++)
    {
        QStandardItem *child = parNode->child(i, 0);


        queue<QStandardItem *> bfsQueue;
        float color = randColor();
        bfsQueue.push(child);
        //boost::random::uniform_int_distribution<> randSix(0,5);
        // distribution that maps to 1..6
        // see random number distributions
        while (!bfsQueue.empty())
        {
            QStandardItem *curNode = bfsQueue.front();
            bfsQueue.pop();
            int numChildren = curNode->rowCount();
            string parName = getCppString(curNode->text());
            cout << "selseg:" << parName.substr(0, 10) << endl;
            cout << "selseg:" << "Terminal__" << endl;

            if (parName.substr(0, 10) == "Terminal__")
            {
                int end = parName.find("_", 10);
                cout << "selseg:" << parName.substr(10, end - 10) << endl;
                int segment = lexical_cast<int>(parName.substr(10, end - 10));

                segNumToColor[segment] = color;
                continue;
            }
            cout << parName << endl;

            for (int i = 0; i < numChildren; i++)
            {
                QStandardItem *child = curNode->child(i, 0);
                cout << "child:" << getCppString(child->text()) << endl;
                bfsQueue.push(child);
            }

        }
    }
    colorSegs(segNumToColor, true);
    updatePCDVis();
    
     //find out the hierarchy level of the selected item
//     if(name.substr(0,4)=="seg_")
//     {
//         int end=name.find("_",4);
//         cout<<"selseg:"<<name.substr(4,end-3)<<endl;
//         int segment=lexical_cast<int>(name.substr(4,end-4));
//         cout<<"selseg:"<<segment<<endl;
//         set<int> segList;
//         segList.insert(segment);
//         colorSegs(segList,true);
//         updatePCDVis();
//     }
     
 }

void ParseTreeLablerForm::addNodeButtonClicked()
{
    cout<<"button handler called"<<endl;
     const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
     QString selectedText = index.data(Qt::DisplayRole).toString();
     //find out the hierarchy level of the selected item
     string name=getCppString(selectedText);
     
     nodeTableModel->addItem(name);
}

void ParseTreeLablerForm::combineButtonClicked()
{
    nodeTableModel->combineAll(this);
}

void ParseTreeLablerForm::clearButtonClicked()
{
    nodeTableModel->clearAll();
    cerr<<"clear button called"<<endl;
}

void ParseTreeLablerForm::windowClosing()
{
    writeTree(parseTreeFileName);   
}

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
            widget.comboBox->addItem(QString(line.data()));
            count++;
            labels.push_back(line);
        }
    } else {
        cout << "could not open typenames file...exiting\n";
        exit(-1);
    }

    fileI.close();


    parseTreeFileName=argv[3];
  //  setUpTree(argv[3]);
    readTree(argv[3]);
//    exit(-1);

    nodeTableModel=new PTNodeTableModel(0);
    widget.tableView->setModel(nodeTableModel);

    // read from file
    if ( pcl::io::loadPCDFile<PointT > (argv[1], cloud_orig) == -1) {
        ROS_ERROR("Couldn't read file ");
        exit (-1);
    }
    
    //viewer.createViewPort(0.0, 0.0, 1.0, 1.0);
    cloud_colored=cloud_orig;
    updatePCDVis();
    
     QItemSelectionModel *selectionModel= widget.treeView->selectionModel();
     connect(selectionModel, SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
             this, SLOT(selectionChangedSlot(const QItemSelection &, const QItemSelection &)));

     connect(widget.addButton, SIGNAL(clicked ()),
             this, SLOT(addNodeButtonClicked()));
     connect(widget.combineButton, SIGNAL(clicked ()),
             this, SLOT(combineButtonClicked()));
     connect(widget.clearButton, SIGNAL(clicked ()),
             this, SLOT(clearButtonClicked()));
     connect(this, SIGNAL(finished (int)),
             this, SLOT(windowClosing()));
     
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
        PTlabler.viewer.spinOnce();
        QCoreApplication::sendPostedEvents();
        QCoreApplication::processEvents();
    }

    cout << "normal kill";
    return (0);


}

