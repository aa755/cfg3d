/*
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include "structures.h"
#include "wallDistance.h"
//#include "CPU_generatedDataStructures.cpp"
#include "generatedDataStructures.cpp"
//#include "Monitor_generatedDataStructures.cpp"
//#include "Printer_generatedDataStructures.cpp"
//#include "Rules_Floor.h"
//#include "Rules_Wall.h"
//#define FILTER_LABELS
// Manual rules that we need.
    vector<VisualObject::SPtr> identifiedScenes;
    
class RGreedyScene : public Rule {
public:
    void combineAndPush(Symbol::SPtr extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal_SPtr> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        VisualObject::SPtrdummyTypeCheck=boost::dynamic_pointer_cast<VisualObject::SPtr>(extractedSym);
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
    void combineAndPush(Symbol::SPtr extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal_SPtr> & terminals /* = 0 */, long iterationNo /* = 0 */)
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

Scene *bestSceneSoFar=NULL;
template<typename SceneType>
class RScene : public Rule {
public:
    void combineAndPush(Symbol::SPtr extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal_SPtr> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (extractedSym->isOfSubClass<SceneType>()) // if min is of type Scene(Goal)
        {
            Scene * LHS= new Scene();
                LHS->addChild(extractedSym);
                LHS->computeSpannedTerminals();
                assert(extractedSym->getNumPoints()!=0);
                int numTerminalsNotExplained=extractedSym->thisScene->getNumTerminalsToBeParsed()-extractedSym->getNumTerminals();
                //LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained + extractedSym->getNumObjectsSpanned()*Params::objectCost);
                LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained);
//                LHS->setAdditionalCost(0.5*(NUMPointsToBeParsed-extractedSym->getNumPoints()));
            if (addToPqueueIfNotDuplicate(LHS, pqueue))
            {
                if (bestSceneSoFar == NULL || bestSceneSoFar->getCost() > LHS->getCost())
                {
                    bestSceneSoFar = LHS;
                    cerr<<"bsc:"<<LHS->getCost()<<endl;
                }
            }
            
        }
    }
};

template<>
bool DoubleRule<VisualObjects,VisualObjects,VisualObject>::isLearned()
{
    return false;
}

//    
//template<typename SupportType, typename RHS_Type2 > 
//typename boost::disable_if<boost::is_base_of<Floor, SupportType>, bool>::type
//DoubleRuleComplex<SupportType,RHS_Type2> :: canBaseBeHallucinated()
//{
//    return false;
//}

template<>
VisualObjects * DoubleRule<VisualObjects,VisualObjects,VisualObject>::applyRuleInference(VisualObjects * RHS1, VisualObject::SPtr RHS2)
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
//    rules.push_back(RulePtr(new RVisualObjects()));
//    rules.push_back(RulePtr(new DoubleRule<VisualObjects, VisualObjects, VisualObject>()));
//    rules.push_back(RulePtr(new RScene()));
    
    appendLearningRules(rules);
    rules.push_back(RulePtr(new RScene<SupportComplex<Floor> >()));
    
#endif
}

void outputOnBothStreams(string str)
{
    cout<<str<<endl;
    cerr<<str<<endl;
}

void runParse(vector<Terminal_SPtr> & terminals) {
    vector<RulePtr> rules;
    appendRuleInstancesForPrimitives(rules);

    SymbolPriorityQueue pq(terminals.size());

    for(unsigned int i=0;i<terminals.size();i++)
    {
         pq.pushTerminal(terminals.at(i));
    }



    Symbol::SPtr min;
    long count = 0;
    long rulecount = 0;
    bool alreadyExtracted=false;
    Scene::COST_THERSHOLD=infinity();
    
#ifdef GREEDY_OBJECTS
    Scene::COST_THERSHOLD=700;
#endif
    TicToc timer;
    cout<<"statring timer"<<endl;
    
    while (true) {
        min = pq.pop(alreadyExtracted);

//        if (min == NULL || min->getCost() > Scene::COST_THERSHOLD )
        if (min == NULL )
        {
            //outputOnBothStreams("parsing failed. goal is not derivable from the given rules ... fix the rules or PQ insertion threshold ... or rules' thershold\n");

            if(bestSceneSoFar!=NULL)
            {
                cerr<<"parsing completed ... PQ empty"<<endl;
                bestSceneSoFar->printData();
            }
//            exit(0);
            
//            if (identifiedScenes.size() == 0)
//            {
//                outputOnBothStreams("Parsing failed. Goal is not derivable from the given rules. Fix the neighbor map.\n");
//            }
//            else
//            {
//                Scene::printAllScenes(identifiedScenes);
//            }
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

        Scene *dummyTypeCheck=boost::dynamic_pointer_cast<Scene*>(min);
        
        if (dummyTypeCheck!=NULL) // if min is of type Scene(Goal)
        {
            cout << "Goal reached!! in time "<< timer.toc() << endl;
            cerr << "Goal reached!! with cost:"<<min->getCost()<< endl;
            min->printData();
            return;
        }
        
        if(timer.toc()>Params::timeLimit)
        {
            cout << "TimeOut!!" << endl;
            
            cerr << "TimeOut!!"<<endl;
            if(bestSceneSoFar!=NULL)
            {
                bestSceneSoFar->printData();
            }
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
//    assert(isinf(infinity()));
    vector<Terminal_SPtr>  terminals;
    SceneInfo::SPtr scn=initParsing(argc,argv);
    runParse(scn->terminals);

    return 0;
    
}
