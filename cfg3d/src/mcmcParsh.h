/* 
 * File:   mcmcParsh.h
 * Author: aa755
 *
 * Created on May 7, 2012, 2:48 PM
 */

#ifndef MCMCPARSH_H
#define	MCMCPARSH_H


#include "structures.h"
#include "wallDistance.h"
//#include "CPU_generatedDataStructures.cpp"
#include "generatedDataStructures.cpp"

using namespace std;


/* parsing using MCMC
 * 
 */

class Forest;

/**
 * this class is designed to support multiple fast queries for rules
 */

class RulesDB
{
    vector<RulePtr>  rules;
    vector<RulePtr>  planarPrimitiveRules;
    map<set<string>, vector<RulePtr> > childTypeToRule; // same LHS can have multiple RHS; Wall, Floor -> Plane
    vector<RulePtr> emptyRules;
    int totalNumParams;
public:
    typedef  boost::shared_ptr<RulesDB> SPtr;

    void prepareRulesVector()
    {
        rules.push_back(RulePtr(new RPlaneSeg()));
        rules.push_back(RulePtr(new RPlane_PlaneSeg()));
        appendLearningRules(rules); 
                
    }
    
    RulesDB()
    {
        prepareRulesVector();
        
        for(vector<RulePtr>::iterator it=rules.begin();it!=rules.end();it++)
        {
            childTypeToRule[(*it)->getChildrenTypesAsSet()].push_back(*it);
            if((*it)->makesPlanarPrimitive())
            {
                cerr<<"ppr:"<<typeid(*(*it)).name()<<endl;
                planarPrimitiveRules.push_back(*it);
            }
        }
        totalNumParams=0;
#ifdef USING_SVM_FOR_LEARNING_CFG
        for(vector<RulePtr>::iterator it=rules.begin();it!=rules.end();it++)
        {
            (*it)->setStartIndex(totalNumParams);
            totalNumParams+=(*it)->getNumParams();
        }
        
#endif        
        cerr<<"rules map has size: "<<childTypeToRule.size()<<","<<totalNumParams<<endl;
    }
    
    void readModel(double * w_svm)
    {
        for(vector<RulePtr>::iterator it=rules.begin();it!=rules.end();it++)
        {
            (*it)->readModel(w_svm);
        }        
    }
    
    vector<RulePtr> & lookupRule(set<string> & childTypes)
    {
        map<set<string>, vector<RulePtr> >::iterator it= childTypeToRule.find(childTypes);
        if(it==childTypeToRule.end())
        {
            return emptyRules;
        }
        else
        {
            return it->second;
        }
    }
    
    template<typename T>
    boost::shared_ptr<T> lookupRuleOfSameType(T & rul)
    {
        for(vector<RulePtr>::iterator it=rules.begin();it!=rules.end();it++)
        {
            if(typeid(*(*it))==typeid(rul))
                return boost::dynamic_pointer_cast<T>(*it);
        }
        
        return createStaticShared<T>(&rul);
    }
    
    const vector<RulePtr> & lookupSingleRule(Symbol::Ptr child)
    {
        set<string> childTypeSet;
        childTypeSet.insert(typeid(*child).name());
        return lookupRule(childTypeSet);
    }
    
    const vector<RulePtr> & lookupDoubleRule(Symbol::Ptr child1, Symbol::Ptr child2 )
    {
        set<string> childTypeSet;
        childTypeSet.insert(typeid(*child1).name());
        childTypeSet.insert(typeid(*child2).name());
        if(childTypeSet.size()==1)//both same
            return emptyRules;
        return lookupRule(childTypeSet);
    }
    
    const vector<RulePtr> & getRulesMakingPlanarPrimitives()
    {
        return planarPrimitiveRules;
    }

    int getTotalNumParams() const {
        return totalNumParams;
    }
};

class SVM_CFG_Y
{
protected:
    vector<Symbol::Ptr> trees;
    VectorXd psi;
    bool featsReadFromFileNoTrees;
    LABELMAP_TYPE labelMap;
public:
    typedef  boost::shared_ptr<SVM_CFG_Y> SPtr;
    typedef  boost::shared_ptr<const SVM_CFG_Y> CSPtr;
    /**
     * 
     * @param segmentNum 1 based segment index
     * @return "" if not found.
     */
    
    static string lookupLabel(const LABELMAP_TYPE &labelMap,int segmentNum)
    {
        LABELMAP_CITER res=labelMap.find(segmentNum);
        if(res==labelMap.end())
        {
            return "";
        }
        else
        {
            return res->second;
        }
    }
    
    string lookupLabel(int segmentNum)
    {
        return lookupLabel(labelMap, segmentNum);
    }
    
    double evalLossDeltaOnAdd(const LABELMAP_TYPE & add) const
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(add,it->first);
            if(label==it->second)
                loss-=2;
        }
        
        return loss;
    }
    
    double evalLossDeltaOnDel(const LABELMAP_TYPE & del) const 
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(del,it->first);
            if(label==it->second)
                loss+=2;
        }
        
        return loss;
    }
        
    double evalLoss(const LABELMAP_TYPE & olabelMap) const
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(olabelMap,it->first);
          /*  if(label=="") // commented to make deltaLoss independent of current state
                loss+=1;
            else */if(label!=it->second)
                loss+=2;
                
        }
        
        return loss;
    }
    
    double getFeat(int i)
    {
        return psi(i);
    }
    
    int getSizePsi()
    {
        return psi.cols();
    }
    SVM_CFG_Y(vector<Symbol::Ptr> &trees)
    {
        init(trees);
    }
    
    int countNumNZ()
    {
        int sizeP=psi.cols();
        assert(sizeP>1);
        int count=0; 
        for(int i=0;i<sizeP;i++)
        {
            if(psi(i)!=0)
                count++;
        }
        return count;
    }
    
    void init(vector<Symbol::Ptr> &trees)
    {
        this->trees=trees;
        featsReadFromFileNoTrees=false;                
    }
    
    SVM_CFG_Y(string file)
    {
        init(file);
    }
    SVM_CFG_Y(){}
    
    void init(string file)
    {

        vector<string> lines;
        getLines(file.data(),lines);

        int numFeats=lines.size();
        psi.setZero(numFeats); // if not present, use line counter
        
        for(int count=0;count<numFeats;count++)
        {
             psi(count)=boost::lexical_cast<double>(lines.at(count));
        }
        featsReadFromFileNoTrees=true;
    }
#ifdef USING_SVM_FOR_LEARNING_CFG
    
    virtual void computePsi()
    {
        SceneInfo::SPtr sceneInfo=trees.at(0)->thisScene;
        psi.setZero(sceneInfo->psiSize);
        for (vector<Symbol::SPtr>::iterator it = trees.begin(); it != trees.end(); it++) 
        {
            (*it)->addYourPsiVectorTo(psi);
        }
    }
        
#endif

    VectorXd getPsi() const {
        return psi;
    }
    
#ifdef USING_SVM_FOR_LEARNING_CFG        
    void printPsi()
    {
        ofstream file;
        SceneInfo::SPtr sceneInfo=trees.at(0)->thisScene;
        string yfile=sceneInfo->fileName+".ypred";
       file.open(yfile.data(), ios::out);
       for(int i=0;i<sceneInfo->psiSize;i++)
       {
           file<<psi(i)<<endl;
       }
       file.close();
    }
#endif    
    
    
};

class Move
{
private:
    double costDelta;
    double transProb;
    LABELMAP_TYPE addMap;
    LABELMAP_TYPE delMap;
protected:
    bool transProbSet;
    bool applied;
    
    /**
     * pnew/pold=exp(-cost_new)/exp(-cost_old)=exp(cost_old-cost_new)=cost(-costDelta)
     */
    void setTransProbFromDelta()
    {
        transProb=exp(-costDelta/10.0);
        transProbSet=true;
    }
    
public:
    void applylabelMapDelta( LABELMAP_TYPE & lab)
    {
        appendLabelmap(addMap,lab);
        subtractLabelmap(delMap,lab);
    }
    
    typedef SupportComplex<Floor> SCENE_TYPE;
    void resetCostDelta()
    {
        costDelta=0;
    }
    
    void adjustCostDeltaForNodeAddition(Symbol::Ptr newNode, Forest & cfor);
    void adjustCostDeltaForNodeRemoval(Symbol::Ptr remNode, Forest & cfor);
    virtual void adjustCostDeltaForNodeAdditionExtra(Symbol::Ptr newNode, Forest & cfor);
    virtual void adjustCostDeltaForNodeRemovalExtra(Symbol::Ptr remNode, Forest & cfor);
    
    virtual string toString()=0;
    double getCostDelta()
    {
        return costDelta;
    }
    
    Move()
    {
        transProb=false;
        costDelta=0;
        applied=false;
    }
    
    virtual bool moveCreationSucceded()
    {
        return true;
    }
    
typedef  boost::shared_ptr<Move> SPtr;

    virtual void applyMove(Forest & cfor)=0;
    /**
     * 
     * @return Q(f',f) where f' is the forest which will be generated when
     * applyMove is called on f
     */
    virtual double getTransitionProbUnnormalized()
    {
        assert(transProbSet);
        return transProb;
    }
    
    virtual bool isInvalidatedOnDeletion(int index)=0;
    virtual bool handleMove(int oldIndex, int newIndex, Forest * cfor /* = NULL */)=0;
    virtual ~Move() {}
};


class Forest
{
    vector<Symbol::Ptr> trees;
    vector<Move::SPtr> moves;
    double curNegLogProb;
    RulesDB::SPtr rulesDB;
    double bestCostSoFar;
    SceneInfo::SPtr sceneInfo;
    LABELMAP_TYPE labelmap;
    SVM_CFG_Y::SPtr gtSVMY;
    
public:
    const static double ADDITIONAL_COMPONENT_PENALTY=150;
    const static double ADDITIONAL_PLANE_TERMINAL_PENALTY=-70;
    const static int NUM_MCMC_ITERATIONS=10000000;
    const static int NON_FLOORCOMPLEX_PENALTY=0;
    const static int timeLimit=500;

    bool lossAugmented()
    {
        return (gtSVMY!=NULL);
    }
    
    SVM_CFG_Y::CSPtr getGTSVMY()
    {
        return gtSVMY;
    }
    void init (SceneInfo::SPtr sceneInfo, RulesDB::SPtr rulesDB)
    {
        this->rulesDB=rulesDB;
        curNegLogProb = 0;
        this->sceneInfo=sceneInfo;
        for (unsigned int i = 0; i <sceneInfo->terminals.size(); i++)
        {
            sceneInfo->terminals.at(i)->declareOptimal(false);
            addTree(sceneInfo->terminals.at(i));
        }

    }
    
    Forest(SceneInfo::SPtr sceneInfo, RulesDB::SPtr rulesDB)
    {
        init(sceneInfo,rulesDB);
    }
    
    /**
     * use this constructor for loss augmented inference
     * @param sceneInfo
     * @param rulesDB
     * @param gtY
     */
    Forest(SceneInfo::SPtr sceneInfo, RulesDB::SPtr rulesDB,SVM_CFG_Y::SPtr gtY)
    {
        init(sceneInfo,rulesDB);
        gtSVMY=gtY;
    }
    
    
    SVM_CFG_Y::SPtr getParsingResult()
    {
        return SVM_CFG_Y::SPtr(new SVM_CFG_Y(trees));
    }
    
    /**
     * 
     * @param index
     * @return : deleted moves
     */
    vector<Move::SPtr> updateMovesOnDeletion(int index)
    {
        vector<Move::SPtr> delMoves;
        int count = 0;
        int size = moves.size();
        while (count < size)
        {
            Move::SPtr mptr = moves.at(count);
            bool ret = mptr->isInvalidatedOnDeletion(index);
            if (ret)
            {
                delMoves.push_back(moves.at(count));
                fast_erase(moves, count);
                size--;
                assert(size == (int)moves.size());
            }
            else
                count++; // the last node moved at count ... needs to be fixed
            
        }
        
        return delMoves;
    }
    
    void print()
    {
        Scene::printAllScenes(trees,sceneInfo);
    }
    
    void deleteTree(int index)
    {
        
//        Symbol::Ptr oldTree=trees.at(index);
//        curNegLogProb-=(oldTree->getCost()+ADDITIONAL_COMPONENT_PENALTY);
        
        vector<Move::SPtr> delMoves=updateMovesOnDeletion(index);

        int oldSize=trees.size();
        int oldIndex=trees.size()-1; // the former last index ... the node there was moved to index
        Symbol::Ptr movedTree=trees.at(oldIndex);
        fast_erase(trees,index);
        assert((int)trees.size()==oldSize-1);
        
        if(oldIndex!=index) // if the last element was deleted, then no moving
                assert(trees.at(index)=movedTree);
        
        {
            vector<Move::SPtr>::iterator it;
            
            for(it=moves.begin();it!=moves.end();it++)
            {
                (*it)->handleMove(oldIndex, index, this);
            }
            
            // some of these deleted moves might be active .. this function 
            // might have been called from one of those deleted moves
            // eg. see mergeMove::applyMove
            // so those also need to be updated
            // those Moves would also carefully recheck validity of indices after
            // this call
            for(it=delMoves.begin();it!=delMoves.end();it++)
            {
                (*it)->handleMove(oldIndex, index, this);
            }
        }
    }
    
    /**
     * create the new moves involving @param tree which was added at @param index
     */
    void addNewMoves(Symbol::Ptr tree, int index);
    
    void addTree(Symbol::Ptr tree)
    {
        assert(tree!=NULL);
        trees.push_back(tree);
        cerr<<"adTr:"<<tree->getName()<<endl;
        addNewMoves(tree,trees.size()-1);
    }
    
    /**
     * 
     * @param index
     * @param tree : tree not \in forest
     */
    void replaceTree(int index,Symbol::Ptr tree)
    {
        updateMovesOnDeletion(index);
        trees.at(index)=tree;
        addNewMoves(tree,index);
        
    }
    
    
    
    Symbol::Ptr getTree(int index)
    {
        return trees.at(index);
    }
    
    /**
     * get a random float b/w 0 and range
     * @param range
     * @return 
     */
    float getRandFloat(float range)
    {
            return ((float)range*(float)rand())/(float)RAND_MAX;        
    }
    
    /* 0 to range -1
     */
    int getRandInt(int range)
    {
            int ret= (int)(getRandFloat(1.0)*(range-1));
            assert(ret < range);
        //    cerr<<"rand:"<<ret<<endl;
            return ret;
    }
    
//    int sampleNextMove()
//    {
//        double sum=0;
//        //double partialSums[moves.size()];
//        //std::array<double,moves.size()> partialSums;
//        vector<double> partialSums;
//        partialSums.resize(moves.size());
//        cerr<<moves.size()<<":";
//        for(int i=0; i< (int)moves.size(); i++ )
//        {
//            double moveProb=moves.at(i)->getTransitionProbUnnormalized(curNegLogProb);
//            sum+=moveProb;
//            partialSums[i]=sum;
//            cerr<<moves.at(i)->getCostDelta() <<",";
//        }
//        cerr<<endl;
//        
//        cerr<<"sum:"<<sum<<endl;
//        
//        int count=0;
//        while(true)
//        {
//            float r = getRand(sum);
//            vector<double>::iterator upb;
//            upb=upper_bound(partialSums.begin(),partialSums.end(),r);
//            assert(upb!=partialSums.end() || r==sum);
//            int selectedMove=(int)(upb-partialSums.begin());
//            count++;
//            Move::SPtr selMove=moves.at(selectedMove);
//            double ratio=1.0/(selMove->getTransitionProbUnnormalized(curNegLogProb));
//            if(ratio>1 || getRand(1.0)<ratio)
//            {
//                cerr<<"#trials= "<<count<<endl;
//                return selectedMove;
//            }
//        }
//    }
    
//    int sampleNextMoveUniform()
//    {
//        
//        int count=0;
//        while(true)
//        {
//            int selectedMove = getRandInt(moves.size());
//            count++;
//            Move::SPtr selMove=moves.at(selectedMove);
//            double q_old_to_new=1.0/moves.size();
//            cerr<<"old #moves:"<<moves.size()<<endl;
//            Forest newF=*this;
////            selMove->applyMove(newF);
//            cerr<<"selMove:"<< selMove->toString() <<endl;
//            newF.moves.at(selectedMove)->applyMove(newF);
//            cerr<<"new #moves:"<<newF.moves.size()<<endl;
//            
//            double q_new_to_old=1.0/newF.moves.size();
//            
//            double factor1=exp(curNegLogProb-newF.curNegLogProb); // newProb/oldProb ; p(x')/p(x)
//            cerr<<"factor1:"<<factor1<<endl;
//            
//            double factor2=q_new_to_old/q_old_to_new;
//            
//            double ratio=factor1*factor2;
//            cerr<<"ratio:"<<ratio<<endl;
//            
//            if(ratio>1 || getRandFloat(1.0)<=ratio)
//            {
//                cerr<<"#trials= "<<count<<endl;
//                return selectedMove;
//            }
//        }
//    }
    
    int sampleNextMoveUniformApprox()
    {
        
        int count=0;
        while(true)
        {
            int selectedMove = getRandInt(moves.size());
            count++;
            Move::SPtr selMove=moves.at(selectedMove);
            
            double factor1=selMove->getTransitionProbUnnormalized(); // newProb/oldProb ; p(x')/p(x)
//            cerr<<"factor1:"<<factor1<<endl;
            
            double factor2=1.0; // approximation 
            // can write a dry run version of applyMove to estimate new # moves
            
            double ratio=factor1*factor2;
            
            if(ratio>1.0 || getRandFloat(1.0)<ratio)
            {
            
                cerr<<"#tri= "<<count<<endl;
                cerr<<"sMv:"<<selMove->toString()<<",rat:"<<ratio<<",#n:"<<trees.size()<<endl;
                return selectedMove;
            }
//            else
//                cerr<<"rMv:"<<selMove->toString()<<endl;
                
        }
    }

    void runMCMC()
    {
        bestCostSoFar=infinity();
        TicToc timer;
        int iter=0;
        while(timer.toc()<timeLimit)
        {
            iter++;
            int nm=sampleNextMoveUniformApprox();
            Move::SPtr selMove=moves.at(nm);
            selMove->applyMove(*this);
            curNegLogProb+=(selMove->getCostDelta());
            
            if(lossAugmented())
                selMove->applylabelMapDelta(labelmap);
            
            if(bestCostSoFar>curNegLogProb)
            {
                bestCostSoFar=curNegLogProb;
              //  fileName=origFileName+"__"+boost::lexical_cast<string>(bestCostSoFar);
              //  print();
            }
        }
    }
    
    
};

void Move::adjustCostDeltaForNodeAddition(Symbol::Ptr newNode, Forest & cfor)
{
    costDelta += (newNode->getCost() + Forest::ADDITIONAL_COMPONENT_PENALTY);

    if (!(newNode->isOfSubClass<Terminal > () || newNode->isOfSubClass<Plane> ()))
        costDelta += Forest::ADDITIONAL_PLANE_TERMINAL_PENALTY;

    if (!(newNode->isOfSubClass<SCENE_TYPE > () ))
        costDelta += Forest::NON_FLOORCOMPLEX_PENALTY;
    
    if(cfor.lossAugmented())
            adjustCostDeltaForNodeAdditionExtra(newNode,cfor);
 
}

void Move::adjustCostDeltaForNodeRemoval(Symbol::Ptr remNode, Forest & cfor)
{
    costDelta -= (remNode->getCost() + Forest::ADDITIONAL_COMPONENT_PENALTY);

    if (!(remNode->isOfSubClass<Terminal > () || remNode->isOfSubClass<Plane> ()))
        costDelta -= Forest::ADDITIONAL_PLANE_TERMINAL_PENALTY;

    if (!(remNode->isOfSubClass<SCENE_TYPE > () ))
        costDelta -= Forest::NON_FLOORCOMPLEX_PENALTY;
    
    if(cfor.lossAugmented())
            adjustCostDeltaForNodeRemovalExtra(remNode,cfor);
}

    void Move::adjustCostDeltaForNodeAdditionExtra(Symbol::Ptr newNode, Forest & cfor)
    {
#ifdef USING_SVM_FOR_LEARNING_CFG        
            newNode->addYourLabelmapTo(addMap);
            costDelta+=cfor.getGTSVMY()->evalLossDeltaOnAdd(newNode->getLabelMap());
           
#endif
    }
    
    
    void Move::adjustCostDeltaForNodeRemovalExtra(Symbol::Ptr remNode, Forest & cfor)
    {        
#ifdef USING_SVM_FOR_LEARNING_CFG        
            remNode->addYourLabelmapTo(delMap);
            costDelta+=cfor.getGTSVMY()->evalLossDeltaOnDel(remNode->getLabelMap());
#endif
    }

/**
 * store nodes(along with indices) also for typechecks
 */
class MergeMove: public Move
{
    int mergeIndex1,mergeIndex2;
    Symbol::Ptr mergeNode1,mergeNode2; // store nodes also for additional safety check
    RulePtr mergeRule;
    NonTerminal_SPtr mergeResult;
    
public:
typedef  boost::shared_ptr<MergeMove> SPtr;

    virtual bool moveCreationSucceded()
    {
        return (mergeResult!=NULL);
    }
    
    vector<Symbol::Ptr> marshalParams(RulePtr mergeRule)
    {
        vector<Symbol::Ptr> nodes;
        if(mergeRule->getChildrenTypes().at(0)==string(typeid(*mergeNode1).name()))
        {
              nodes.push_back(mergeNode1);
              nodes.push_back(mergeNode2);
        }
        else
        {            
            assert(mergeRule->getChildrenTypes().at(0)==string(typeid(*mergeNode2).name()));
              nodes.push_back(mergeNode2);
              nodes.push_back(mergeNode1);            
        }
        return nodes;
    }
    
    virtual ~MergeMove()
    {
    }
    
    MergeMove(Forest & cfor, int mergeIndex1, int mergeIndex2, RulePtr mergeRule)
    {
        this->mergeIndex1=mergeIndex1;
        this->mergeIndex2=mergeIndex2;
        this->mergeRule=mergeRule;
        assert(mergeRule->getChildrenTypes().size()==2);
        
        assert(mergeIndex1!=mergeIndex2);
        
        mergeNode1=cfor.getTree(mergeIndex1);
        mergeNode2=cfor.getTree(mergeIndex2);
        
        assert(cfor.getTree(mergeIndex1)==mergeNode1);
        assert(cfor.getTree(mergeIndex2)==mergeNode2);
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams(mergeRule));
        
        
        if(mergeResult==NULL)
        {
            cerr<<"mf:"<<mergeNode1->getName()+","+mergeNode2->getName()<<":"<<typeid(*mergeRule).name()<<endl;
            return;
        }

        mergeResult->declareOptimal(false);
        
        //costDelta=mergeResult->getCost()-mergeNode1->getCost()-mergeNode2->getCost()-Forest::ADDITIONAL_COMPONENT_PENALTY;
        adjustCostDeltaForNodeAddition(mergeResult,cfor);
        adjustCostDeltaForNodeRemoval(mergeNode1,cfor);
        adjustCostDeltaForNodeRemoval(mergeNode2,cfor);
        setTransProbFromDelta();
        
    }
    
    virtual string toString()
    {
        return mergeNode1->getName()+","+mergeNode2->getName()+"->"+mergeResult->getName();
    }
    
    virtual void applyMove(Forest & cfor)
    {
        applied=true;
        // safety checks ... in the face of index updates
        assert(cfor.getTree(mergeIndex1)==mergeNode1);
        assert(cfor.getTree(mergeIndex2)==mergeNode2);
        
        cerr<<"mer-move rule "<<typeid(*mergeRule).name()<<endl;
        
        cfor.deleteTree(mergeIndex1);// max to reduce #index updates in moves
        
        assert(cfor.getTree(mergeIndex2)==mergeNode2);
        
        cfor.replaceTree(mergeIndex2,mergeResult); // isSpanDisjo
    }
    
    
    virtual bool isInvalidatedOnDeletion(int index)
    {
        if(index==mergeIndex1 || index==mergeIndex2)
            return true;
        
     
        return false;
    }
    
    virtual bool handleMove(int oldIndex, int newIndex , Forest * cfor /* = NULL */)
    {
        bool changed=false;
        
        
        if(mergeIndex1 == oldIndex)
        {
            mergeIndex1 = newIndex;
            changed=true;
        }
        
        if(mergeIndex2 == oldIndex)
        {
            mergeIndex2 = newIndex;
            changed=true;
        }
        
//        assert(cfor->getTree(mergeIndex1)==mergeNode1);
//        assert(cfor->getTree(mergeIndex2)==mergeNode2);
            
     
        return changed;
    }
};

/**
 * store node(along with index) also for typechecks
 */
class SingleRuleMove: public Move
{
protected:
    int mergeIndex;
    Symbol::Ptr mergeNode; // store nodes also for additional safety check
    RulePtr mergeRule;
    Symbol::Ptr mergeResult;
    
public:
typedef  boost::shared_ptr<MergeMove> SPtr;
    
    virtual string toString()
    {
        return mergeNode->getName()+"->"+mergeResult->getName();
    }
    
    vector<Symbol::Ptr> marshalParams()
    {
        vector<Symbol::Ptr> nodes;
        nodes.push_back(mergeNode);
        return nodes;
    }
    
    virtual ~SingleRuleMove() 
    {
    }
    
    SingleRuleMove(Forest & cfor, int mergeIndex, RulePtr mergeRule)
    {
        this->mergeIndex=mergeIndex;
        this->mergeRule=mergeRule;
        
        mergeNode=cfor.getTree(mergeIndex);
        
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
      
//        assert(mergeResult!=NULL);
        if(mergeResult==NULL)
            return;
          mergeResult->declareOptimal(false);
        
        adjustCostDeltaForNodeAddition(mergeResult,cfor);
        adjustCostDeltaForNodeRemoval(mergeNode,cfor);
        setTransProbFromDelta();
        
    }
    SingleRuleMove()
    {
        
    }
    
    virtual bool moveCreationSucceded()
    {
        return (mergeResult!=NULL);
    }
    
    
    virtual void applyMove(Forest & cfor)
    {
        
        applied=true;
        assert(cfor.getTree(mergeIndex)==mergeNode);
        cerr<<"sr"<<mergeNode->getName()<<"->"<<mergeResult->getName()<<endl;
        
        cfor.replaceTree(mergeIndex,mergeResult);
        
        singleMoveAdditionalWork();
        
    }
    
    virtual void singleMoveAdditionalWork()
    {
        
    }
    
    virtual bool isInvalidatedOnDeletion(int index)
    {
        if(index==mergeIndex)
            return true;
     
        return false;
    }
    
    virtual bool handleMove(int oldIndex, int newIndex , Forest * cfor /* = NULL */)
    {
        bool changed=false;
        if(mergeIndex == oldIndex)
        {
            mergeIndex = newIndex;
            changed=true;
        }
                    
     
        return changed;
    }
};

class MutatePlanarPrimitiveMove : public SingleRuleMove
{
protected:
    Plane::SPtr  rhs;
public:
    vector<Symbol::Ptr> marshalParams()
    {
        vector<Symbol::Ptr> nodes;
        nodes.push_back(rhs);
        return nodes;
    }
    
    MutatePlanarPrimitiveMove(Forest & cfor, int mergeIndex, RulePtr mergeRule)
    {
        this->mergeIndex=mergeIndex;
        this->mergeRule=mergeRule;
        
        mergeNode=cfor.getTree(mergeIndex);

        NonTerminal_SPtr nt = boost::dynamic_pointer_cast<NonTerminal>(mergeNode); 
        assert(nt);
        assert(nt->children.size()==1);
        rhs=boost::dynamic_pointer_cast<Plane>(nt->getChild(0));
        assert(rhs!=NULL);
        
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
      
//        assert(mergeResult!=NULL);
        if(mergeResult==NULL)
            return;
          mergeResult->declareOptimal(false);
        
          assert(typeid(*mergeResult)!=typeid(*mergeNode)); // this is useless
        adjustCostDeltaForNodeAddition(mergeResult,cfor);
        adjustCostDeltaForNodeRemoval(mergeNode,cfor);
        setTransProbFromDelta();        
    }
    
    virtual string toString()
    {
        return string("mut:")+mergeNode->getName()+"->"+mergeResult->getName();
    }
    
    virtual void singleMoveAdditionalWork()
    {
    }
    
};
class SplitMove: public Move
{
    int delIndex;
    Symbol::Ptr delNode;
    string desc;
public:
    
    SplitMove(Forest & cfor,int delIndex)
    {
        this->delIndex=delIndex;
        delNode=cfor.getTree(delIndex);
        desc="del:"+delNode->getName();
        NonTerminal_SPtr nt=boost::dynamic_pointer_cast<NonTerminal>(delNode);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        adjustCostDeltaForNodeRemoval(delNode,cfor);
        
        {
            vector<Symbol::SPtr>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                adjustCostDeltaForNodeAddition(*it,cfor);
            }
        }
        
        setTransProbFromDelta();
        
    }
    
    virtual string toString()
    {
        return desc;
    }
    
    virtual void applyMove(Forest & cfor)
    {
        applied=true;
        assert(cfor.getTree(delIndex)==delNode);
        cfor.deleteTree(delIndex);
        NonTerminal_SPtr nt=boost::dynamic_pointer_cast<NonTerminal>(delNode);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        
        {
            vector<Symbol::SPtr>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                cfor.addTree(*it);
            }
        }
        
    }
    

    virtual bool isInvalidatedOnDeletion(int index)
    {
        if(index== delIndex)
            return true;
     
        return false;
    }
    
    virtual bool handleMove(int oldIndex, int newIndex /* = 0 */, Forest * cfor /* = NULL */)
    {
        bool changed=false;
        if(delIndex == oldIndex)
        {
            delIndex = newIndex;
            changed=true;
        }
        
     
        return changed;
    }
};

void Forest::addNewMoves(Symbol::Ptr tree, int index)
{

    //single rule

    
    const vector<RulePtr> & rules = rulesDB->lookupSingleRule(tree);
    for(int i=0;i<(int)rules.size();i++)
    {
        RulePtr rule=rules.at(i);
        Move::SPtr newMerge=(Move::SPtr(new SingleRuleMove(*this, index, rule)));
                    if(newMerge->moveCreationSucceded())
                        moves.push_back(newMerge);
    }


    // try to apply double rules
    for (vector<Symbol::Ptr>::iterator it = trees.begin(); it != trees.end(); it++)
    {
        if (tree != (*it))
        {
            //cerr<<"db:"<<tree->getName()<<":"<<(*it)->getName()<<endl;
            assert(tree->isSpanExclusive((*it)));
            if (!(tree->isSpanExclusive((*it)->getNeigborTerminalBitset())))
            {
                
                const vector<RulePtr> & rules = rulesDB->lookupDoubleRule(tree, *it);
                for(int i=0;i<(int)rules.size();i++)
                {
                    RulePtr rul=rules.at(i);
                    Move::SPtr newMerge=Move::SPtr(new MergeMove(*this, index, it - trees.begin(), rul));
                    if(newMerge->moveCreationSucceded())
                        moves.push_back(newMerge);
                }
            }
        }
    }

    // add the delete move for this new node
    if(tree->isOfSubClass<NonTerminal>())
        moves.push_back(Move::SPtr(new SplitMove(*this, index)));

    // mutation
    if (tree->isOfSubClass<PlanarPrimitive > ())
    {
        const vector<RulePtr> & rules = rulesDB->getRulesMakingPlanarPrimitives();
        for (int i = 0; i < (int) rules.size(); i++)
        {
            RulePtr rule = rules.at(i);
            if(rule->getLHSType()!=string(typeid(*tree).name()))
            {
                Move::SPtr newMerge = (Move::SPtr(new MutatePlanarPrimitiveMove(*this, index, rule)));
                if (newMerge->moveCreationSucceded())
                    moves.push_back(newMerge);
            }
        }

    }
    


}

#endif	/* MCMCPARSH_H */

