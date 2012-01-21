#ifndef CFG3D_HELPER_CPP
#define	CFG3D_HELPER_CPP


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
 * Initialize global "terminals" vector without any points.
 * @param maxSegNum
 */
vector<Terminal *> terminals;
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
        string terminalLabel = segNumToLabel.at(i - 1);
        temp->setLabel(terminalLabel);
        labelToTerminals[terminalLabel] = temp;
        terminals.push_back(temp);
    }
    
    Terminal::totalNumTerminals=maxSegNum;
    assert((int)terminals.size()==Terminal::totalNumTerminals);
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
        terminal->setNeighbors(maxSegNum);
        terminal->declareOptimal();

        Plane* pl = new Plane();
        pl->addChild(terminal);
        pl->computeSpannedTerminals();
        pl->computeFeatures();
        pl->computePlaneParamsAndEigens();
        pl->setAbsoluteCost(0);
        pl->declareOptimal();
      
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
        segmentLabelPair.push_back(t);
    }
    cout<<endl;
    return segmentLabelPair;
}

/**
 * Initializes the mapping, segNumToLabel, between the segment number and the label we assign to it in learning.
 * @param segNumToLabelFileName
 */
void initializeSegNumToLabel(char* segNumToLabelFileName) {
    ifstream segNumToLabelStream;
    segNumToLabelStream.open(segNumToLabelFileName);

    string line;
    if (segNumToLabelStream.is_open())
    {
        while (segNumToLabelStream.good()) {
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
            assert(terminalToAddTo!=NULL);
            
            // Adding points indices to the empty Terminals.
            terminalToAddTo->addPointIndex(i);            
        }
    
    }
    segMinDistances.setZero(terminals.size(),terminals.size());
    
    for(unsigned int i1=0;i1<terminals.size();i1++)
    {
            for(unsigned int i2=i1+1;i2<terminals.size();i2++)
            {
                float minDistance=getSmallestDistance(scene, terminals.at(i1)->getPointIndicesBoostPtr(), terminals.at(i2)->getPointIndicesBoostPtr());
                segMinDistances(i1,i2)=minDistance;
                segMinDistances(i2,i1)=minDistance;
            }
    }
    
    // Plane initialization.
    initializePlanes(maxSegNum);
}

Plane* getPlane(string label) {
    return labelToPlanes.at(label);
}

vector<string> splitLine(string line) {
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    vector<string> tokenList;
    BOOST_FOREACH(string t, tokens)
    {   
        tokenList.push_back(t);
    }
    return tokenList;
}

#endif //CFG3D_HELPER_CPP