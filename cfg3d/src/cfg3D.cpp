/*
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include "structures.cpp"
#include "wallDistance.h"
#include "CPU_generatedDataStructures.cpp"
#include "Monitor_generatedDataStructures.cpp"
#include "Printer_generatedDataStructures.cpp"
#include "Rules_Floor.h"
#include "Rules_Wall.h"
//#define FILTER_LABELS
#define GREEDY_OBJECTS
// Manual rules that we need.
class RPlaneSeg : public Rule {
public:

    Plane * applyRule(Symbol * extractedSym)
    {
        Plane * LHS = new Plane();
        LHS->addChild(extractedSym);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        return LHS;
        
    }
    
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) != typeid (Terminal))
            return;
        
        addToPqueueIfNotDuplicate(applyRule(extractedSym),pqueue); //TODO: duplicate check is not required
    }
};

// Manual rules that we need.
class RPlane_PlaneSeg : public Rule {
public:
    
    int get_Nof_RHS_symbols() {
        return 2;
    }

    void get_typenames(vector<string> & names) {
        names.push_back(typeid (Plane).name());
        names.push_back(typeid (Terminal).name());
    }
    
    NonTerminal* applyRule(Plane * RHS_plane, Terminal *RHS_seg, vector<Terminal*> & terminals) {
        if (!RHS_plane->isAllCloseEnough(RHS_seg)) {
            return NULL;
        }
        
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        //TODO : FIRST COMPUTE PLANE PARAMS FROM JUST THIS AND THEN 
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        return LHS;
    }

    NonTerminal* applyRuleLearning(Plane * RHS_plane, Terminal *RHS_seg) {
        
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndEigens(); // also computes feats
        LHS->declareOptimal();
        
        return LHS;
    }
    
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        set<int>::iterator it;
        //all terminals have cost=0, all NT's have cost>0,
        //so when a terminal is extracted, no non-terminal(plane)
        //has been extracted yet
        //so, if sym is of type Terminal, it cannot be combined with a plane
        if (typeid (*extractedSym) == typeid (Plane))
        {
            extractedSym->resetNeighborIterator();
            int index;
            while (extractedSym->nextNeighborIndex(index))
            {
                Plane * plane=dynamic_cast<Plane*> (extractedSym);
                NonTerminal *newNT=applyRule(plane,terminals[index],terminals);
                addToPqueueIfNotDuplicate(newNT,pqueue);
            }
        }
    }
};
    vector<VisualObject*> identifiedScenes;
    
class RGreedyScene : public Rule {
public:
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        VisualObject *dummyTypeCheck=dynamic_cast<VisualObject*>(extractedSym);
        if (dummyTypeCheck!=NULL) // if min is of type Scene(Goal)
        {
           // cout << "An object!!" << endl;
            if (dummyTypeCheck->doesNotOverlapWithScenes( identifiedScenes)) 
            {
                identifiedScenes.push_back(dummyTypeCheck);
            }
        }
    }
};

class RVisualObjects : public Rule {
public:
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (extractedSym->isOfSubClass<VisualObject>()) // if min is of type Scene(Goal)
        {
            VisualObjects * LHS= new VisualObjects();
                LHS->addChild(extractedSym);
                LHS->computeSpannedTerminals();
                LHS->setAdditionalCost(0);
                addToPqueueIfNotDuplicate(LHS,pqueue);
                
            
        }
    }
};

class RScene : public Rule {
public:
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (extractedSym->isOfSubClass<VisualObjects>()) // if min is of type Scene(Goal)
        {
            Scene * LHS= new Scene();
                LHS->addChild(extractedSym);
                LHS->computeSpannedTerminals();
                assert(extractedSym->getNumPoints()!=0);
                LHS->setAdditionalCost(1000*(NUMTerminalsToBeParsed-extractedSym->getNumTerminals()));
                addToPqueueIfNotDuplicate(LHS,pqueue);
                
            
        }
    }
};

template<>
bool DoubleRule<VisualObjects,VisualObjects,VisualObject>::isLearned()
{
    return false;
}

template<>
VisualObjects * DoubleRule<VisualObjects,VisualObjects,VisualObject>::applyRuleInference(VisualObjects * RHS1, VisualObject * RHS2, vector<Terminal*> & terminals)
{
            VisualObjects* LHS= new VisualObjects();
                LHS->addChild(RHS1);
                LHS->addChild(RHS2);
                LHS->computeSpannedTerminals();
                LHS->setAdditionalCost(0);
                //LHS->setAdditionalCost(NUMPointsToBeParsed-RHS1->getNumPoints()-RHS2->getNumPoints());
                return LHS;
}

void appendRuleInstancesForPrimitives(vector<RulePtr> & rules) {
    
    // planes
    rules.push_back(RulePtr(new RPlaneSeg()));
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));
#ifdef GREEDY_OBJECTS
    rules.push_back(RulePtr(new RGreedyScene()));
#else
    rules.push_back(RulePtr(new RVisualObjects()));
    rules.push_back(RulePtr(new DoubleRule<VisualObjects, VisualObjects, VisualObject>()));
    rules.push_back(RulePtr(new RScene()));
    
#endif
}

void outputOnBothStreams(string str)
{
    cout<<str<<endl;
    cerr<<str<<endl;
}

void runParse(map<int, set<int> > & neighbors, int maxSegIndex, char * labelMapFile=NULL) {
    vector<RulePtr> rules;
    appendRuleInstancesForPrimitives(rules);
    CPUAppendLearningRules(rules);
//    MonitorAppendLearningRules(rules);
//    rules.push_back(RulePtr(new SingleRule<Wall, Plane>()));
//    rules.push_back(RulePtr(new SingleRule<Floor, Plane>()));
//    printerAppendLearningRules(rules);

    //    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());

    SymbolPriorityQueue pq(maxSegIndex);

    vector<Terminal *> terminals;
    NUMTerminalsToBeParsed=0;
#ifdef FILTER_LABELS
    assert(labelMapFile!=NULL);
    LabelSelector labSel;
    labSel.addAcceptedLabel(1);//Wall
    labSel.addAcceptedLabel(2);//Floor
    labSel.addAcceptedLabel(7);//Monitor
    labSel.addAcceptedLabel(6);//CPUFront
    labSel.addAcceptedLabel(36);//CPUSide
    labSel.addAcceptedLabel(34);//Monitor
    labSel.addAcceptedLabel(22);//printerFront
    labSel.addAcceptedLabel(112);//printerSide
    labSel.addAcceptedLabel(117);//printerTop
    map<int,int> labelmap;
    readLabelMap(labelMapFile,labelmap);
#endif
    Terminal * temp;
    for (int i = 1; i <= maxSegIndex; i++) {
        temp = new Terminal(i-1); // index is segment Number -1 
        temp->setNeighbors( neighbors[i],maxSegIndex);
        terminals.push_back(temp);
        
        
        {
#ifdef FILTER_LABELS        
        if(labSel.acceptLabel(labelmap[i])) // get labelmap fro gt files
#endif
                {
                pq.pushTerminal(temp);
                NUMTerminalsToBeParsed++;
                }
        }
        
        
    }

    NUMPointsToBeParsed=0;
    for(unsigned int i=0;i<scene.size();i++)
    {
        if(rand()%10 != 1)
            continue;
        int segIndex=scene.points[i].segment;
        if(segIndex>0 && segIndex<=maxSegIndex)
        {
            terminals.at(segIndex-1)->addPointIndex(i);
            
            
            {
#ifdef FILTER_LABELS        
        if(labSel.acceptLabel(segIndex)) // get labelmap fro gt files
#endif
              NUMPointsToBeParsed++;
            }
            
            
        }
    }
    
    for(unsigned int i=0;i<terminals.size();i++)
    {
      //  terminals.at(i)->computeMinDistanceBwNbrTerminals(terminals)
        terminals.at(i)->computeFeatures();
    }

    getSegmentDistanceToBoundaryOptimized(scene,terminals,occlusionChecker->maxDist);
    
    occlusionChecker->setmaxDistReady();
    
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
    
    cout<<"minDistances computed\n"<<endl;
    cerr<<"minDistances computed\n"<<endl;

    for(unsigned int i=0;i<terminals.size();i++)
    {
       // cout<<"s "<<i<<terminals[i]->getPointIndices().size()<<endl;
        assert(terminals[i]->getPointIndices().size()>0); 
        // if this happens, delete this NT. NEED TO CHANGE SIZE OF NEIGHBOR VECTOR
    }
    
    Terminal::totalNumTerminals = terminals.size();

    Symbol *min;
    long count = 0;
    long rulecount = 0;
    bool alreadyExtracted=false;
    Scene::COST_THERSHOLD=infinity();
    
#ifdef GREEDY_OBJECTS
    Scene::COST_THERSHOLD=700;
#endif
    
    
    while (true) {
        min = pq.pop(alreadyExtracted);

        if (min == NULL || min->getCost() > Scene::COST_THERSHOLD)
        {
            //outputOnBothStreams("parsing failed. goal is not derivable from the given rules ... fix the rules or PQ insertion threshold ... or rules' thershold\n");

            if (identifiedScenes.size() == 0)
            {
                outputOnBothStreams("Parsing failed. Goal is not derivable from the given rules. Fix the neighbor map.\n");
            }
            else
            {
                Scene::printAllScenes(identifiedScenes);
            }
            exit(-1);
        }
        
        if(alreadyExtracted)
        {
            delete min;
            cout << "Dup." << endl;
            // since there are no parent links yet(not yet declared optimal),
            // and it was not a child of anyone (not yet combined)
            // and it was repoved from PQ
            // and it was not in NTSetsExtracted,
            // deleting does not cause dangling pointers
            continue;
        }

        cout << "\n\n\nIteration: " << count++ << " Cost: " << min->getCost() <<" Type: "<<min->getName()<< endl;

        Scene *dummyTypeCheck=dynamic_cast<Scene*>(min);
        if (dummyTypeCheck!=NULL) // if min is of type Scene(Goal)
        {
            cout << "Goal reached!!" << endl;
            cerr << "Goal reached!! with cost:"<<min->getCost()<< endl;
            min->printData();
            return;
        }
        
        if (typeid (*min) == typeid (Terminal) || !alreadyExtracted) {
            min->declareOptimal();
            min->printData();
 //           cout<<"mz"<<min->getMaxZ()<<endl;
            
            for (size_t i = 0; i < rules.size(); i++) {
                rules.at(i)->combineAndPush(min, pq, terminals,rulecount++); // combine with the eligible NT's to form new NTs and add them to the priority queue
                //an eligible NT should not span any terminal already in min
                //an eligible NT should contain atleast 1 terminal in combneCandidates
            }

        }
        //pq.pop();

    }
}

void subsample(pcl::PointCloud<PointT> & inp, pcl::PointCloud<PointT> & out) {
    out.points.clear();
    out.header = inp.header;
    for (size_t i = 0; i < inp.size(); i++) {
        if (rand() % 5 == 1) {
            out.points.push_back(inp.points[i]);
        }
    }
}

void convertToXY(const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<pcl::PointXY> & cloudxy)
{
    cloudxy.points.resize(cloud.size());
    cloudxy.sensor_origin_=cloud.sensor_origin_;
    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloudxy.points[i].x = cloud.points[i].x;
        cloudxy.points[i].y = cloud.points[i].y;
    }
}

int main(int argc, char** argv) {
    assert(isinf(infinity()));
    
    if(argc!=3&&argc!=4)
    {
        cerr<<"Usage: "<<argv[0]<<" <pcdFile> <nbrMapFile> [gtLabelfile]"<<endl;
    }
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
    fileName = string(argv[1]);
    fileName = fileName.substr(0, fileName.length()-4);

    occlusionChecker = new OccupancyMap<PointT>(scene);
    //convertToXY(scene,scene2D);
  //  scene2DPtr=createStaticShared<pcl::PointCloud<pcl::PointXY> >(&scene2D);
    map<int, set<int> > neighbors;
    int maxSegIndex= parseNbrMap(argv[2],neighbors,MAX_SEG_INDEX);
    cout<<"Scene has "<<scene.size()<<" points."<<endl;
    
    char *gtLableFileName=NULL;
    if(argc==4)
        gtLableFileName=argv[3];
    
    string command="rospack find cfg3d";
    rulePath=exec(command.data());
    rulePath=rulePath.substr(0,rulePath.length()-1)+"/rules";
    runParse(neighbors, maxSegIndex,gtLableFileName);

    return 0;
    
}
