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
    cerr<<maxSegNum<<endl;
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
//            cerr<<"adding entry:"<<segmentLabelPair.at(0)<<segmentLabelPair.at(1)<<endl;
//            string segNumString = segmentLabelPair.at(1);
//            segNumToLabel[atoi(segNumString.c_str())] = segmentLabelPair.at(0);
            // If file is given in reverse order of label->segmentNumber
        }
    }
}

map<int,Terminal*> numToTerminal;
Terminal * getTerminalSafe(int segmentNum)
{
    Terminal * ret=numToTerminal[segmentNum];
    assert(ret!=NULL);
    return ret;
}
vector<Terminal*> dummy;
void initialize(pcl::PointCloud<PointT> & scene, pcl::PointCloud<PointT> & originalScene) {
    pcl::PointCloud<PointT>::Ptr originalScenePtr=createStaticShared<pcl::PointCloud<PointT> >(&originalScene);
    generatePTIndexMapping(scene,originalScene);
    
    hogpcd.init(originalScenePtr);

    overallMinZ=infinity();
    Terminal::totalNumTerminals=0;
    string command="rospack find cfg3d";
    rulePath=exec(command.data());
    rulePath=rulePath.substr(0,rulePath.length()-1)+"/rules";   
    for(unsigned int i = 0; i < scene.size(); i++)
    {
        int currentSegNum = scene.points[i].segment;

        if(currentSegNum > 0)
        {
            if(Terminal::totalNumTerminals<currentSegNum)
                Terminal::totalNumTerminals=currentSegNum;
            
            if(overallMinZ>scene.points[i].z)
                overallMinZ=scene.points[i].z;
            
            Terminal* terminalToAddTo = numToTerminal[currentSegNum];
            if(terminalToAddTo==NULL)
            {
                terminalToAddTo=new Terminal(currentSegNum-1);
                assert(terminalToAddTo!=NULL);
                numToTerminal[currentSegNum]=terminalToAddTo;
            }
            terminalToAddTo->addPointIndex(i);            
        }
    
    }
    
    dummy.clear();
    for(map<int,Terminal*>::iterator it=numToTerminal.begin();it!=numToTerminal.end();it++)
    {
        Terminal *terminal=it->second;
        dummy.push_back(terminal);
        terminal->computeFeatures();
        terminal->setNeighbors(Terminal::totalNumTerminals);
        terminal->declareOptimal();         
    }
    
    double maxDist[360];
    getSegmentDistanceToBoundaryOptimized(scene,dummy,maxDist);
            
    segMinDistances.setZero(Terminal::totalNumTerminals,Terminal::totalNumTerminals);
    
    for(int i1=1;i1<=Terminal::totalNumTerminals;i1++)
    {
            for(int i2=i1+1;i2<=Terminal::totalNumTerminals;i2++)
            {
                Terminal * t1=numToTerminal[i1];
                Terminal * t2=numToTerminal[i2];
                if(t1!=NULL && t2!=NULL)
                {
                        float minDistance=getSmallestDistance(scene, t1->getPointIndicesBoostPtr(), t2->getPointIndicesBoostPtr());
                        segMinDistances(i1-1,i2-1)=minDistance;
                        segMinDistances(i2-1,i1-1)=minDistance;
                }
                else
                {
                        segMinDistances(i1-1,i2-1)=numeric_limits<double>::quiet_NaN( );
                        segMinDistances(i2-1,i1-1)=numeric_limits<double>::quiet_NaN( );
                }
            }
    }
  
    // Plane initialization.
//    initializePlanes(maxSegNum);
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

//class Floor : public NonTerminal{};
//class Wall : public NonTerminal{};
//class door : public NonTerminal{};
//class tableTop : public NonTerminal{};

vector<string> tempTypeStrs;
#endif //CFG3D_HELPER_CPP
