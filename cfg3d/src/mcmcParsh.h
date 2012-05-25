/* 
 * File:   mcmcParsh.h
 * Author: aa755
 *
 * Created on May 7, 2012, 2:48 PM
 */

#ifndef MCMCPARSH_H
#define	MCMCPARSH_H


#define GREEDY_SVM_TRAINING_PREDICTION
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


class SVM_CFG_Y :  public boost::enable_shared_from_this<SVM_CFG_Y> 
{
protected:
    vector<Symbol::Ptr> trees;
    VectorXd psi;
    bool featsReadFromFileNoTrees;
    LABELMAP_TYPE labelMap;
    const static double LOSS_PER_NODE=0.1;
    map<boost::dynamic_bitset<>, string >  span2NT;
public:
    typedef  boost::shared_ptr<SVM_CFG_Y> SPtr;
    typedef  boost::shared_ptr<const SVM_CFG_Y> CSPtr;
    /**
     * 
     * @param segmentNum 1 based segment index
     * @return "" if not found.
     */
    int getSize()
    {
        return labelMap.size();
    }
    
    double getEmptyTreeLoss()
    {
        return getSize()*LOSS_PER_NODE;
    }
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
    
    void printLabelMap()
    {
        printLabelmap(labelMap,cerr);
    }
    
    double evalLossDeltaOnAdd(const LABELMAP_TYPE & add) const
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(add,it->first);
            if(label==it->second)
            {
  //              cerr<<"lr:"<<it->second<<endl;
                loss-=LOSS_PER_NODE;
            }
        }
   //     cout<<"rl\n";
        return loss;
    }
    
    double evalLossDeltaOnDel(const LABELMAP_TYPE & del) const 
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(del,it->first);
            if(label==it->second)
                loss+=LOSS_PER_NODE;
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
                loss+=LOSS_PER_NODE;
                
        }
        
        return loss;
    }
    
    double evalLoss(SVM_CFG_Y::CSPtr other) const
    {
        double loss=0;
        for(LABELMAP_CITER it=labelMap.begin();it!=labelMap.end();it++)
        {
            string label=lookupLabel(other->labelMap,it->first);
          /*  if(label=="") // commented to make deltaLoss independent of current state
                loss+=1;
            else */if(label!=it->second)
                loss+=LOSS_PER_NODE;
                
        }
        cerr<<"mcmcmloss:"<<loss<<endl;
        return loss;
    }
    
    double computeScore(VectorXd & wSvm, SVM_CFG_Y::CSPtr gt) const
    {
        return wSvm.dot(psi) + gt->evalLoss(shared_from_this());
    }
    
    double getFeat(int i)
    {
        return psi(i);
    }
    
    int getSizePsi()
    {
        return psi.rows();
    }
    
    SVM_CFG_Y(vector<Symbol::Ptr> &trees)
    {
        init(trees);
        
    }
    
    int countNumNZ()
    {
        int sizeP=psi.rows();
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
#ifdef USING_SVM_FOR_LEARNING_CFG
        for(vector<Symbol::Ptr>::iterator it =trees.begin();it!=trees.end();it++)
        {
            (*it)->addYourLabelmapTo(labelMap);
        }
        computePsi();
#endif
        featsReadFromFileNoTrees=false;                
    }
    
    void printPsi(string label="")
    {
        cerr<<"psi-"<<label<<endl;
        for(int i=0;i<psi.rows();i++)
        {
            cerr<<psi(i)<<",";
        }
        cerr<<endl;
    }
    
    SVM_CFG_Y(string file)
    {
        init(file);
    }
    SVM_CFG_Y(){}
    
    void init(string file)
    {

        string base=file.substr(0,file.length()-4);
        vector<string> lines;
        getLines((base+".ypred").data(),lines);

        int numFeats=lines.size();
        psi.setZero(numFeats); 
        
        for(int count=0;count<numFeats;count++)
        {
             psi(count)=boost::lexical_cast<double>(lines.at(count));
        }
        featsReadFromFileNoTrees=true;
        lines.clear();
        getLines((file+"_gt_tree.dot.labelmap").data(),lines);
        assert(lines.size()>0);
        for(vector<string>::iterator it=lines.begin();it!=lines.end();it++)
        {
            vector<string> toks; 
            getTokens(*it,toks);
            assert(toks.size()==2);
            labelMap[boost::lexical_cast<int>(toks.at(1))]=toks.at(0);
        }

        
    }
#ifdef USING_SVM_FOR_LEARNING_CFG
    
    virtual void computePsi()
    {
        SceneInfo::SPtr sceneInfo=trees.at(0)->thisScene;
        psi.setZero(sceneInfo->getPsiSize());
        for (vector<Symbol::SPtr>::iterator it = trees.begin(); it != trees.end(); it++) 
        {
            (*it)->addYourPsiVectorTo(psi);
        }
    }
        
#endif

    const VectorXd & getPsi() const {
        return psi;
    }
    
#ifdef USING_SVM_FOR_LEARNING_CFG        
    void printPsi()
    {
        ofstream file;
        SceneInfo::SPtr sceneInfo=trees.at(0)->thisScene;
        string yfile=sceneInfo->fileName+".ypred";
       file.open(yfile.data(), ios::out);
       for(int i=0;i<sceneInfo->getPsiSize();i++)
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
#ifdef USING_SVM_FOR_LEARNING_CFG
        transProb=exp(costDelta/10.0);
#else
        transProb=exp(-costDelta/10.0);
        
#endif
        transProbSet=true;
    }
    
public:
    void applylabelMapDelta( LABELMAP_TYPE & lab)
    {
//        cerr<<"---delta: nodes for deletion:-------\n";
//        printLabelmap(delMap,cerr);
        subtractLabelmap(delMap,lab);
        
//        cerr<<"---nodes for addition---\n";
//        printLabelmap(addMap,cerr);
        appendLabelmap(addMap,lab);
//        cerr<<"----------------------\n";
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
        TicToc timer;
        int timeLimit;
    
/**
     * inefficient, but needs less code-change . Alternatively, can make moves a sorted (multi)set
     */
     int findBestMove() 
    {

        double bestScore=-infinity();
        int bestMoveIndex=-1;
        for (int i=0 ; i < (int)moves.size() ; i++) 
        {
            double score=moves.at(i)->getCostDelta();
            if(bestScore<score)
            {
                bestScore=score;
                bestMoveIndex=i;
            }
        }
     //   assert(bestMoveIndex!=-1);
        return bestMoveIndex;
    }
    
     void searchStarting()
     {
         timer.tic();
        timeLimit=timeLimitInit;
        
#ifdef USING_SVM_FOR_LEARNING_CFG
        timeLimit+=(timeLimitIncrement*rulesDB->getCountIter());
#endif
        
     }
     
#ifdef GREEDY_SVM_TRAINING_PREDICTION
     bool shouldSearchEnd()
     {
         assert(trees.size()>=1);
         return (trees.size()==1 || timer.toc()>timeLimit);
         
     }     
#else
     bool shouldSearchEnd()
     {
         return (timer.toc()>timeLimit);
     }
#endif     
public:
    void validate()
    {
       // print();
        double score=0;
        for (vector<Symbol::Ptr>::iterator it = trees.begin(); it != trees.end(); it++)
        {
            score+=(*it)->getCost();
        }
        
//        assert(score==0);
        if(lossAugmented())
        {
            score+=gtSVMY->evalLoss(labelmap);
        }
        assert(floatEqual(score,curNegLogProb));
    }
#ifdef USING_SVM_FOR_LEARNING_CFG
    const static double ADDITIONAL_COMPONENT_PENALTY=0;
    const static double ADDITIONAL_PLANE_TERMINAL_PENALTY=0;
#else
    const static double ADDITIONAL_COMPONENT_PENALTY=150;
    const static double ADDITIONAL_PLANE_TERMINAL_PENALTY=-70;
#endif
    
    const static int NUM_MCMC_ITERATIONS=10000000;
    const static int NON_FLOORCOMPLEX_PENALTY=0;
    const static int timeLimitInit=50;
    const static int timeLimitIncrement=5;

    bool lossAugmented()
    {
//        assert (gtSVMY!=NULL);
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
        this->sceneInfo->setRulesDB(rulesDB);
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
   //     cout<<"created a l-ag forest"<<endl;
        gtSVMY=gtY;
        init(sceneInfo,rulesDB);
        curNegLogProb=gtY->getEmptyTreeLoss();
    }
    
    
    SVM_CFG_Y::SPtr getParsingResult()
    {
        SVM_CFG_Y::SPtr ret= SVM_CFG_Y::SPtr(new SVM_CFG_Y(trees));
        double scoreEstimate=ret->computeScore(rulesDB->getWSVM(), getGTSVMY());
        //print();
//        cerr<<"----estimated labelmap-----\n";
//        printLabelmap(labelmap,cerr);
//        cerr<<"-----actual labelmap------\n";
//        ret->printLabelMap();
//        cerr<<"-----------\n";
        cerr<<"scores"<<curNegLogProb<<","<<scoreEstimate<<endl;
        assert(floatEqual(curNegLogProb,scoreEstimate));
        return ret;
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
        tree->validateCost();
        assert(tree!=NULL);
        trees.push_back(tree);
       // cerr<<"adTr:"<<tree->getName()<<endl;
        addNewMoves(tree,trees.size()-1);
    }
    
    /**
     * 
     * @param index
     * @param tree : tree not \in forest
     */
    void replaceTree(int index,Symbol::Ptr tree)
    {
        tree->validateCost();
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
    
    int sampleNextMove()
    {
        double sum=0;
        //double partialSums[moves.size()];
        //std::array<double,moves.size()> partialSums;
        vector<double> partialSums;
        partialSums.resize(moves.size());
    //    cerr<<moves.size()<<":";
        for(int i=0; i< (int)moves.size(); i++ )
        {
            double moveProb=moves.at(i)->getTransitionProbUnnormalized();
            sum+=moveProb;
            partialSums[i]=sum;
  //          cerr<<moves.at(i)->getCostDelta() <<",";
        }
        cerr<<endl;
        
//        cerr<<"sum:"<<sum<<endl;
        
            float r = getRandFloat(sum);
            vector<double>::iterator upb;
            upb=upper_bound(partialSums.begin(),partialSums.end(),r);
            //assert(upb!=partialSums.end() || r==sum);
            int selectedMove=(int)(upb-partialSums.begin());
            if(selectedMove==(int)moves.size())
                selectedMove=moves.size()-1;
            
                return selectedMove;
    }
    int sampleNextMoveRarelyDelete();
    
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
            
   //             cerr<<"#tri= "<<count<<endl;
     //           cerr<<"sMv:"<<selMove->toString()<<",rat:"<<ratio<<",#n:"<<trees.size()<<endl;
                return selectedMove;
            }
//            else
//                cerr<<"rMv:"<<selMove->toString()<<endl;
                
        }
    }

    void runMCMC()
    {
        srand(time(NULL));
        bestCostSoFar=infinity();
        int iter=0;
        searchStarting();
        cerr<<"tl:"<<timeLimit<<endl;
        while(!shouldSearchEnd())
        {
            iter++;
#ifdef GREEDY_SVM_TRAINING_PREDICTION
            if(moves.size()==0)
                break;
            int nm=sampleNextMoveRarelyDelete(); 
#else
            int nm=sampleNextMoveUniformApprox();
#endif
            Move::SPtr selMove=moves.at(nm);
            selMove->applyMove(*this);
            curNegLogProb+=(selMove->getCostDelta());
            
            if(lossAugmented())
                selMove->applylabelMapDelta(labelmap);
            
            validate();
            if(bestCostSoFar>curNegLogProb)
            {
                bestCostSoFar=curNegLogProb;
                //fileName=origFileName+"__"+boost::lexical_cast<string>(bestCostSoFar);
            }
        }
                        print();

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
 //           cerr<<"mf:"<<mergeNode1->getName()+","+mergeNode2->getName()<<":"<<typeid(*mergeRule).name()<<endl;
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
        
  //      cerr<<"mer-move rule "<<typeid(*mergeRule).name()<<endl;
        
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
//        cerr<<"sr"<<mergeNode->getName()<<"->"<<mergeResult->getName()<<endl;
        
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
//        assert(rules.at(i)->getChildrenTypes().size()==1);
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

//#ifndef GREEDY_SVM_TRAINING_PREDICTION

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
    
//#endif

}
    int Forest::sampleNextMoveRarelyDelete()
    {
        while(true)
        {
                int n=sampleNextMove();
                if(typeid(*(moves.at(n)))!=typeid(SplitMove))
                    return n;
                else if(getRandInt(30)<2)
                    return n;
        }
            
    }

#endif	/* MCMCPARSH_H */

