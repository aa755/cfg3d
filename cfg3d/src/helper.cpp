#include "structures.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;

map<string, Terminal*> labelToTerminals;
map<string, Plane*> labelToPlanes;
map<int, string> segNumToLabel;

int getMaxSegNumber(pcl::PointCloud<PointT> scene) {
    int maxSegNum = 0;
    int currentSegNum;
    for(unsigned int i = 0; i < scene.size(); i++) {
        currentSegNum = scene.points[i].segment;
        if (currentSegNum > maxSegNum) {
            maxSegNum = currentSegNum;
        }
    }
    return maxSegNum;
}

/**
 * Initialize Terminals without any points.
 * @param maxSegNum
 */
void initializeTerminals(int & maxSegNum) {
    Terminal* temp;
    for (int i = 1; i <= maxSegNum; i++) {
        
        if(segNumToLabel.find(i-1)==segNumToLabel.end())
        {
            cerr<<"WARN: some segments ignored coz only contiguous labeled segments are allowed\n";
            maxSegNum=i-1;
            break;
        }
        
        temp = new Terminal(i - 1); // index is segment Number -1 
        cout<<"segNumToLabel.size() = "<<segNumToLabel.size()<<endl;
        string terminalLabel = segNumToLabel.at(i - 1);
        temp->setLabel(terminalLabel);
        labelToTerminals[terminalLabel] = temp;
    }
}

/**
 * Initialize labelToPlanes.
 * @param maxSegNum
 */
void initializePlanes(int maxSegNum) {
    for (int i = 1; i <= maxSegNum; i++) {
        string label = segNumToLabel.at(i-1);
        Terminal* terminal = labelToTerminals.at(label);
        terminal->computeFeatures();
        terminal->declareOptimal();

        cout<<"cost terminal1"<<terminal->getCost()<<endl;
        Plane* pl = new Plane();
        pl->addChild(terminal);
        cout<<"cost terminal2"<<terminal->getCost()<<endl;
        pl->computeFeatures();
        cout<<"cost terminal3"<<terminal->getCost()<<endl;
        pl->setAbsoluteCost(0);
        pl->declareOptimal();
//        plane->returnPlaneParams();
      
        cout<<"Label: "<<label<<endl;
        labelToPlanes[label] = pl;
    }
}

vector<string> addToSegNumToLabelFromString(string line) {
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    vector<string> segmentLabelPair;
    cout<<"segmentLabelPair: "<<endl;
    BOOST_FOREACH(string t, tokens)
    {
//        cout<<t<<",";
        segmentLabelPair.push_back(t);
    }
    cout<<endl;
    return segmentLabelPair;
}

//run fridge.pcd_segmented.pcd fridge.pcd_segmented.pcd_nbr.txt
void initializeSegNumToLabel(char* segNumToLabelFileName) {
    ifstream segNumToLabelStream;
    segNumToLabelStream.open(segNumToLabelFileName);

    string line;
    if (segNumToLabelStream.is_open())
    {
        while (segNumToLabelStream.good()) {
//            cout<<"testhere"<<endl;
            getline(segNumToLabelStream, line);
            if (line.size() == 0) 
            {
                break;
            }
            vector<string> segmentLabelPair = addToSegNumToLabelFromString(line);
            string segNumString = segmentLabelPair.at(0);
            segNumToLabel[atoi(segNumString.c_str()) - 1] = segmentLabelPair.at(1);
//            string segNumString = segmentLabelPair.at(1);
//            segNumToLabel[atoi(segNumString.c_str())] = segmentLabelPair.at(0);
            // If file is given in reverse order of label->segmentNumber
        }
    }
}

void initialize(pcl::PointCloud<PointT> scene, char* segNumToLabelFile) {
    
    // SegNumToLabel initialization.
    initializeSegNumToLabel(segNumToLabelFile);
        
    // Terminal initialization.
    int maxSegNum = getMaxSegNumber(scene);
    initializeTerminals(maxSegNum);
    set<int> segNums;
    for(unsigned int i = 0; i < scene.size(); i++)
    {
        int currentSegNum = scene.points[i].segment-1;

        
        if(currentSegNum >= 0 && currentSegNum < maxSegNum)
        {
            segNums.insert(currentSegNum);
            string label = segNumToLabel[currentSegNum];
            cout<<endl;
            Terminal* terminalToAddTo = labelToTerminals[label];
//            cout<<labelToTerminals.size()<<endl;
//            cout<<"with label: "<<label<<endl;
            assert(terminalToAddTo!=NULL);
            terminalToAddTo->addPointIndex(i);            
        }
    
    }
    // Plane initialization.
    initializePlanes(maxSegNum);
}

Plane* getPlane(string label) {
    return labelToPlanes.at(label);
}