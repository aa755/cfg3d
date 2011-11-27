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
void initializeTerminals(int maxSegNum) {
    Terminal* temp;
    for (int i = 1; i <= maxSegNum; i++) {
        temp = new Terminal(i - 1); // index is segment Number -1 
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

        Plane* plane = new Plane();
        plane->addChild(terminal);
        plane->addPointIndices(terminal->getPointIndices());
        plane->computeFeatures();
        plane->computePlaneParams();
        labelToPlanes[label] = plane;
    }
}

vector<string> addToSegNumToLabelFromString(string line) {
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    vector<string> segmentLabelPair;
    BOOST_FOREACH(string t, tokens)
    {
        segmentLabelPair.push_back(t);
    }
    return segmentLabelPair;
}


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
            segNumToLabel[atoi(segNumString.c_str())] = segmentLabelPair.at(1);
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
    for(unsigned int i = 0; i < scene.size(); i++)
    {
        int currentSegNum = scene.points[i].segment;
        string label = segNumToLabel[currentSegNum];
        if(currentSegNum > 0 && currentSegNum <= maxSegNum)
        {
            Terminal* terminalToAddTo = labelToTerminals[label];
            terminalToAddTo->addPointIndex(i);            
        }
    
    }
    // Plane initialization.
    initializePlanes(maxSegNum);
}

Plane* getPlane(string label) {
    return labelToPlanes.at(label);
}