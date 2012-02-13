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
#include"segmentUtils.h"
#include <algorithm>
//#include "structures.cpp"

ParseTreeLablerForm::ParseTreeLablerForm() : viewer("3DViewer"), randSix(0,5)
{
    widget.setupUi(this);
    undoAvl=false;
    pcd_modified=false;
}

ParseTreeLablerForm::~ParseTreeLablerForm()
{
}

string getCppString(QString str)
{
    return string(str.toUtf8().constData());
}

template <typename T>
double getValueOfType(QString str)
{
    return lexical_cast<T>(getCppString(str));
}
class Node
{
public:
    string type;
    int id;
    string memo;
    
    Node(string fullname)
    {
        int typeEnd=fullname.find("__");
        type=fullname.substr(0,typeEnd);
        int idEnd=fullname.find("__",typeEnd+1);
        
        if(idEnd==(int)string::npos)
            idEnd=fullname.size();
        string ids=fullname.substr(typeEnd+2,idEnd-typeEnd-2);
        //cout<<"++"<<ids<<"++"<<endl;
        id=lexical_cast<int>(ids);
        if(idEnd==(int)fullname.size())
        {
            memo="";
        }
        else
        {
          memo=fullname.substr(idEnd+2);
        }
        //cout<<type<<"-"<<id<<"-"<<memo<<endl;
    }
    
    bool updateTypeCounts(map<string,int>& typeMaxId)
    {
        if(typeMaxId.find(type)==typeMaxId.end())
        {
            typeMaxId[type]=id;
            return true; // new type
        }
        else if(typeMaxId[type]<id)
            typeMaxId[type]=id;
        
        return false;
        
    }
};

void ParseTreeLablerForm::updateTypeCounts(string fullname)
{
                Node nd(fullname);
                if(nd.updateTypeCounts(typeMaxId))
                {
                        widget.comboBox->addItem(QString(nd.type.data()));
                        cerr<<"WARN:new types added from parsetree"<<endl;
                }
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
                updateTypeCounts(name);
                QStandardItem *item = new QStandardItem(name.data());
                nameToTreeNode[name] = item;

                rootNode->appendRow(item);

            }
            else if (toks.size() == 2)
            {
                // with a parent
                string name = toks.at(1);
                updateTypeCounts(name);
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
    
    viewer.addCoordinateSystem();
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

void ParseTreeLablerForm::splitButtonClicked()
{
    //get the text of the selected item
    const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
    QString selectedText = index.data(Qt::DisplayRole).toString();
    string name = getCppString(selectedText);
    std::vector<pcl::PointIndices> clusters;
    Node nd(name);
    cout<<"in split seg"<<endl;
    if (nd.type == "Terminal")
    {
        double radT=getValueOfType<double>(widget.radEdit->text());
        double angleT=getValueOfType<double>(widget.angleEdit->text());
        double colorT=getValueOfType<double>(widget.colorT->text());
        int numNbr=getValueOfType<int>(widget.nbrEdit->text());
        int segId = nd.id;
        pcl::PointCloud<PointT> segCloud;
        segCloud.header = cloud_orig.header;
        vector<int> originalIndices;
        undoAvl = true;
        cloud_undo=cloud_orig;
        for (int i = 0; i < (int)cloud_orig.points.size(); i++)
        {
            if ((int)cloud_orig.points[i].segment == segId)
            {
                segCloud.points.push_back(cloud_orig.points[i]);
                cloud_orig.points[i].segment=0;
                originalIndices.push_back(i);
            }
        }
        cout<<"oversegmenting this seg whihch had "<<segCloud.size()<<"with params"<<radT<<","<<angleT<<","<<numNbr<<endl;
    //    segment(segCloud, clusters,radT,angleT,numNbr,true,colorT);
        SegmentPCDGraph<PointT> segmenter(angleT,radT,numNbr,colorT,100);
        segmenter.segment(segCloud,clusters);
        sort(clusters.begin(), clusters.end(), compareSegsDecreasing);

        
        // the old segment now contains the largest segment after splitting
        segNumToColor.clear();
        segNumToColor[segId]=randColor();
        cout<<clusters.size()<<" oversegments found"<<endl;
        for (size_t i = 0; i < clusters.size(); i++)
        {
            cout<<"cluster "<<i<<"has "<<clusters[i].indices.size()<<" points\n";
            int newSegId;
            if(i==0)
            {
                newSegId=segId;
            }
            else
            {
                newSegId=(++typeMaxId["Terminal"]);
            }
            segNumToColor[newSegId]=randColor();
            for (size_t j = 0; j < clusters[i].indices.size(); j++)
            {
                cloud_orig.points[originalIndices.at(clusters[i].indices[j])].segment = newSegId;
            }
            if(i==0)
                continue;
            
            string name="Terminal__"+lexical_cast<string>(newSegId)+"__split"+lexical_cast<string>(segId);
                QStandardItem *item = new QStandardItem(name.data());
                nameToTreeNode[name] = item;
                undoNames.push_back(name);
                rootNode->appendRow(item);
            
        }
    colorSegs(segNumToColor, true);
    updatePCDVis();

              QMessageBox::warning(this, tr("Parse Tree Labaler"),
                                tr("IMPORTANT: undo operation should not be done after merging any newly created segments"
                                   "It will cause inconsistency"),
                                QMessageBox::Ok,QMessageBox::Ok);            

    }
}

void ParseTreeLablerForm::undoSplitButtonClicked()
{
    if(!undoAvl)
        return;
    undoAvl=false;
    
    cloud_orig=cloud_undo;
    for(vector<string>::iterator it=undoNames.begin();it!=undoNames.end();it++)
    {
        rootNode->removeRow(nameToTreeNode[*it]->row());
        nameToTreeNode.erase(*it);
    }
    undoNames.clear();
}

void ParseTreeLablerForm::addNodeFromTree(const QModelIndex & index)
{
         QString selectedText = index.data(Qt::DisplayRole).toString();
     //find out the hierarchy level of the selected item
     string name=getCppString(selectedText);
    Node nd(name);
    cout<<"memo: "<<nd.memo<<endl;
    if (nd.memo.size() != 0 && nd.type == "Terminal")
    {
        for (map<string, int>::iterator it = typeMaxId.begin(); it != typeMaxId.end(); it++)
        {
            if(it->first.find(nd.memo)!=string::npos)
            {
                widget.comboBox->setEditText(QString(it->first.data()));
            }
            
        }
    }
     
     nodeTableModel->addItem(name);

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
        Node nd(name);
            if (nd.type == "Terminal")
            {
                segNumToColor[nd.id] = randColor();
            }
        colorMapTableModel->clearAll();
    for (int i = 0; i < numChildren; i++)
    {
        QStandardItem *child = parNode->child(i, 0);
                        


        queue<QStandardItem *> bfsQueue;
        float color = randColor();
                        ColorRGB colorRGB(color);
                        colorMapTableModel->addItem(getCppString(child->text()),QColor(colorRGB.r*255,colorRGB.g*255,colorRGB.b*255));
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
            //cout << "selseg:" << parName.substr(0, 10) << endl;
            //cout << "selseg:" << "Terminal__" << endl;
            
            Node ndp(parName);

            if (ndp.type == "Terminal")
            {

                segNumToColor[ndp.id] = color;
                continue;
            }
            
            //cout << parName << endl;

            for (int i = 0; i < numChildren; i++)
            {
                QStandardItem *child = curNode->child(i, 0);
              //  cout << "child:" << getCppString(child->text()) << endl;
                bfsQueue.push(child);
            }

        }
    }
    colorSegs(segNumToColor, true);
    updatePCDVis();
    
     
 }

void ParseTreeLablerForm::showNbrButtonClicked()
{
    //get the text of the selected item
    const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
    QString selectedText = index.data(Qt::DisplayRole).toString();
    string name = getCppString(selectedText);
        segNumToColor.clear();
        Node nd(name);
        colorMapTableModel->clearAll();
            if (nd.type == "Terminal")
            {
                set<int> & nbrs =nbrMap[nd.id];
                for(set<int>::iterator it= nbrs.begin(); it!=nbrs.end(); it++)
                {
                    float colorS=randColor();
                        segNumToColor[*it] =colorS;
                        
                        ColorRGB colorRGB(colorS);
                        colorMapTableModel->addItem(string("Terminal__")+lexical_cast<string>(*it),QColor(colorRGB.r*255,colorRGB.g*255,colorRGB.b*255));
                }
                colorSegs(segNumToColor,true);
                updatePCDVis();
                
            }
            else
            {
              QMessageBox::warning(this, tr("Parse Tree Labaler"),
                                tr("Neighbors can be shown only for a Terminal"
                                   "Please select a Terminal in the tree explorer"),
                                QMessageBox::Ok,QMessageBox::Ok);            
              return;
                
            }

 }

void ParseTreeLablerForm::addNodeButtonClicked()
{
     const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
     addNodeFromTree(index);
}

void ParseTreeLablerForm::deleteNodeButtonClicked()
{
    cout<<"delete button handler called"<<endl;
     const QModelIndex index = widget.treeView->selectionModel()->currentIndex();
     QString selectedText = index.data(Qt::DisplayRole).toString();
     //find out the hierarchy level of the selected item
     string name=getCppString(selectedText);
     
     QStandardItem * node=nameToTreeNode[name];
     
     while(node->rowCount()>0)
     {
         QStandardItem * child=node->child(0,0);
         node->takeRow(0);
         rootNode->appendRow(child);
     }
     QStandardItem * parent=node->parent();
     if(parent==0)
         parent=rootNode;
//     int row=node->row();
     parent->removeRow(node->row());
     nameToTreeNode.erase(name);
     
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
    
    ofstream ofile;
    ofile.open(typeListFile,ios::out);
    for(map<string,int>::iterator it=typeMaxId.begin();it!=typeMaxId.end();it++)
    {
        ofile<<it->first<<endl;
    }
    ofile.close();
    pcl::io::savePCDFile(pcdFileName,cloud_orig,true);
    exit(0);
}

void ParseTreeLablerForm::init(int argc, char** argv)
{
    if(argc!=5 && argc!=6)
    {
        cerr<<"usage:"<<argv[0]<<" <segmented_PCD> <neighborMap> <inputTree> <typenames>"<<endl;
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
    typeListFile=argv[4];
    fileI.open(typeListFile);

    if (fileI.is_open()) {
        int count = 1;
        while (fileI.good()) {
            getline(fileI, line); //each line is a label
            if (line.size() == 0)
                break;
            cout << "adding typename " << line  << endl;
            widget.comboBox->addItem(QString(line.data()));
            count++;
            typeMaxId[line]=0;
        }
    } else {
        cout << "could not open typenames file...exiting\n";
        exit(-1);
    }
                        
    fileI.close();


    parseNbrMap(argv[2],nbrMap,INT_MAX);
    parseTreeFileName=argv[3];
  //  setUpTree(argv[3]);
    readTree(argv[3]);
//    exit(-1);

    nodeTableModel=new PTNodeTableModel(0);
    colorMapTableModel=new ColorMapTableModel(0);
    widget.colorTable->setModel(colorMapTableModel);
    widget.tableView->setModel(nodeTableModel);
    // read from file
    pcdFileName=argv[1];
    if ( pcl::io::loadPCDFile<PointT > (pcdFileName, cloud_orig) == -1) {
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
     connect(widget.deleteButton, SIGNAL(clicked ()),
             this, SLOT(deleteNodeButtonClicked()));
     connect(widget.showNbrButton, SIGNAL(clicked ()),
             this, SLOT(showNbrButtonClicked()));
     connect(widget.splitButton, SIGNAL(clicked ()),
             this, SLOT(splitButtonClicked()));
     connect(widget.undoSplitButton, SIGNAL(clicked ()),
             this, SLOT(undoSplitButtonClicked()));
     connect(widget.treeView, SIGNAL(doubleClicked(const QModelIndex &)),
             this, SLOT(addNodeFromTree(const QModelIndex &)));
     
     widget.comboBox->setEditable(true);
                    widget.comboBox->setDuplicatesEnabled(false);
 
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

