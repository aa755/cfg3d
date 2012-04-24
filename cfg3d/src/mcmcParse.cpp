/* 
 * File:   mcmcParse.cpp
 * Author: aa755
 *
 * Created on April 12, 2012, 8:43 PM
 */

#include <cstdlib>
#include "structures.cpp"
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
    map<set<string>, vector<RulePtr> > childTypeToRule; // same LHS can have multiple RHS; Wall, Floor -> Plane
    vector<RulePtr> emptyRules;
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
        }
        cerr<<"rules map has size: "<<childTypeToRule.size()<<endl;
        
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
    
};

class Move
{
private:
    double costDelta;
protected:
    double transProb;
    bool transProbSet;
    bool applied;
    
    /**
     * pnew/pold=exp(-cost_new)/exp(-cost_old)=exp(cost_old-cost_new)=cost(-costDelta)
     */
    void setTransProbFromDelta()
    {
        transProb=exp(-costDelta);
        transProbSet=true;
    }
    
public:
    void resetCostDelta()
    {
        costDelta=0;
    }
    
    void adjustCostDeltaForNodeAddition(Symbol::Ptr newNode);
    void adjustCostDeltaForNodeRemoval(Symbol::Ptr remNode);
    
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
    Scene *bestSceneSoFar;
    string origFileName;
    double bestCostSoFar;
    
    
public:
    const static double ADDITIONAL_COMPONENT_PENALTY=200;
    const static double ADDITIONAL_PLANE_TERMINAL_PENALTY=-100;
    const static int NUM_MCMC_ITERATIONS=100000000;

/**
     * not supposed to deal with tree-move operation
     * just remove all the moves which referenced it
     * 
     * @param index : the index of tree which was removed
     */
    Forest(vector<Terminal *> & terminals, RulesDB::SPtr rulesDB)
    {
        this->rulesDB=rulesDB;
        curNegLogProb = 0;
        bestSceneSoFar=NULL;
        for (unsigned int i = 0; i < terminals.size(); i++)
        {
            terminals.at(i)->declareOptimal(false);
            addTree(terminals.at(i));
        }

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
        Scene::printAllScenes(trees);
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
//        curNegLogProb+=tree->getCost()+ADDITIONAL_COMPONENT_PENALTY;
        
//        if(tree->isOfSubClass<SupportComplex<Floor> >() )
//        {
//                        Scene * LHS= new Scene();
//                LHS->addChild(tree);
//                LHS->computeSpannedTerminals();
//                assert(tree->getNumPoints()!=0);
//                int numTerminalsNotExplained=NUMTerminalsToBeParsed-tree->getNumTerminals();
//                //LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained + extractedSym->getNumObjectsSpanned()*Params::objectCost);
//                LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained);
////                LHS->setAdditionalCost(0.5*(NUMPointsToBeParsed-extractedSym->getNumPoints()));
//                if (bestSceneSoFar == NULL || bestSceneSoFar->getCost() > LHS->getCost())
//                {
//                    delete bestSceneSoFar; // delete earlier best secene
//                    bestSceneSoFar = LHS;
//                }
//                else
//                    delete LHS;
//
//        }
    }
    
    /**
     * 
     * @param index
     * @param tree : tree not \in forest
     */
    void replaceTree(int index,Symbol::Ptr tree)
    {
//        Symbol::Ptr oldTree=trees.at(index);
//        curNegLogProb-=oldTree->getCost();
        updateMovesOnDeletion(index);
        trees.at(index)=tree;
//        curNegLogProb+=tree->getCost();
        addNewMoves(tree,index);
        
        if(tree->isOfSubClass<SupportComplex<Floor> >() )
        {
                        Scene * LHS= new Scene();
                LHS->addChild(tree);
                LHS->computeSpannedTerminals();
                assert(tree->getNumPoints()!=0);
                int numTerminalsNotExplained=NUMTerminalsToBeParsed-tree->getNumTerminals();
                //LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained + extractedSym->getNumObjectsSpanned()*Params::objectCost);
                LHS->setAdditionalCost(Params::missPenalty*numTerminalsNotExplained);
//                LHS->setAdditionalCost(0.5*(NUMPointsToBeParsed-extractedSym->getNumPoints()));
                if (bestSceneSoFar == NULL || bestSceneSoFar->getCost() > LHS->getCost())
                {
                    delete bestSceneSoFar; // delete earlier best secene
                    bestSceneSoFar = LHS;
                }
                else
                    delete LHS;

        }
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
            cerr<<"rand:"<<ret<<endl;
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
    
    int sampleNextMoveUniform()
    {
        
        int count=0;
        while(true)
        {
            int selectedMove = getRandInt(moves.size());
            count++;
            Move::SPtr selMove=moves.at(selectedMove);
            double q_old_to_new=1.0/moves.size();
            cerr<<"old #moves:"<<moves.size()<<endl;
            Forest newF=*this;
//            selMove->applyMove(newF);
            cerr<<"selMove:"<<selMove->toString()<<endl;
            newF.moves.at(selectedMove)->applyMove(newF);
            cerr<<"new #moves:"<<newF.moves.size()<<endl;
            
            double q_new_to_old=1.0/newF.moves.size();
            
            double factor1=exp(curNegLogProb-newF.curNegLogProb); // newProb/oldProb ; p(x')/p(x)
            cerr<<"factor1:"<<factor1<<endl;
            
            double factor2=q_new_to_old/q_old_to_new;
            
            double ratio=factor1*factor2;
            cerr<<"ratio:"<<ratio<<endl;
            
            if(ratio>1 || getRandFloat(1.0)<=ratio)
            {
                cerr<<"#trials= "<<count<<endl;
                return selectedMove;
            }
        }
    }
    
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
            cerr<<"rat:"<<ratio<<",#n:"<<trees.size()<<endl;
            
            if(ratio>1.0 || getRandFloat(1.0)<=ratio)
            {
                cerr<<"#tri= "<<count<<endl;
                cerr<<"sMv:"<<selMove->toString()<<endl;
                return selectedMove;
            }
            else
                cerr<<"rMv:"<<selMove->toString()<<endl;
                
        }
    }

    void runMCMC()
    {
        origFileName=fileName;
        bestCostSoFar=infinity();
        for(int i=0;i<NUM_MCMC_ITERATIONS;i++)
        {
            int nm=sampleNextMoveUniformApprox();
            Move::SPtr selMove=moves.at(nm);
            selMove->applyMove(*this);
            curNegLogProb+=(selMove->getCostDelta());
            if(bestCostSoFar>curNegLogProb)
            {
                bestCostSoFar=curNegLogProb;
                fileName=origFileName+"__"+boost::lexical_cast<string>(bestCostSoFar);
                print();
            }
          //  int iter=i*100/NUM_MCMC_ITERATIONS;
//            if( (i % (NUM_MCMC_ITERATIONS/100))==0)
//            {
//                if(bestSceneSoFar!=NULL)
//                {
//                        fileName=fileName+"_";
//                        bestSceneSoFar->printData();
//                }
//                else
//                {
//                    cerr<<i<<":NULL\n";
//                }
//                
//            }
        }
    }
};

void Move::adjustCostDeltaForNodeAddition(Symbol::Ptr newNode)
{
    costDelta += (newNode->getCost() + Forest::ADDITIONAL_COMPONENT_PENALTY);

    if (!(newNode->isOfSubClass<Terminal > () || newNode->isOfSubClass<Plane> ()))
        costDelta += Forest::ADDITIONAL_PLANE_TERMINAL_PENALTY;

}

void Move::adjustCostDeltaForNodeRemoval(Symbol::Ptr remNode)
{
    costDelta -= (remNode->getCost() + Forest::ADDITIONAL_COMPONENT_PENALTY);

    if (!(remNode->isOfSubClass<Terminal > () || remNode->isOfSubClass<Plane> ()))
        costDelta -= Forest::ADDITIONAL_PLANE_TERMINAL_PENALTY;
}

/**
 * store nodes(along with indices) also for typechecks
 */
class MergeMove: public Move
{
    int mergeIndex1,mergeIndex2;
    Symbol::Ptr mergeNode1,mergeNode2; // store nodes also for additional safety check
    RulePtr mergeRule;
    NonTerminal* mergeResult;
    
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
        if(!applied)
            delete mergeResult;
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
            cerr<<"mf:"<<toString()<<endl;
            return;
        }

        mergeResult->declareOptimal(false);
        
        //costDelta=mergeResult->getCost()-mergeNode1->getCost()-mergeNode2->getCost()-Forest::ADDITIONAL_COMPONENT_PENALTY;
        adjustCostDeltaForNodeAddition(mergeResult);
        adjustCostDeltaForNodeRemoval(mergeNode1);
        adjustCostDeltaForNodeRemoval(mergeNode2);
        setTransProbFromDelta();
        
    }
    
    virtual string toString()
    {
        return typeid(*mergeRule).name();
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
    int mergeIndex;
    Symbol::Ptr mergeNode; // store nodes also for additional safety check
    RulePtr mergeRule;
    Symbol::Ptr mergeResult;
    
public:
typedef  boost::shared_ptr<MergeMove> SPtr;
    
    vector<Symbol::Ptr> marshalParams()
    {
        vector<Symbol::Ptr> nodes;
        nodes.push_back(mergeNode);
        return nodes;
    }
    
    virtual ~SingleRuleMove() 
    {
        if(!applied)
            delete mergeResult;
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
        
        adjustCostDeltaForNodeAddition(mergeResult);
        adjustCostDeltaForNodeRemoval(mergeNode);
        setTransProbFromDelta();
        
    }
    
    virtual bool moveCreationSucceded()
    {
        return (mergeResult!=NULL);
    }
    
    virtual string toString()
    {
        return typeid(*mergeRule).name();
    }
    
    virtual void applyMove(Forest & cfor)
    {
        
        applied=true;
        assert(cfor.getTree(mergeIndex)==mergeNode);
        cerr<<"sr"<<mergeNode->getName()<<"->"<<mergeResult->getName()<<endl;
        
        cfor.replaceTree(mergeIndex,mergeResult);
        
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
        desc="del:"+string(typeid(*delNode).name());
        NonTerminal * nt=dynamic_cast<NonTerminal*>(delNode);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        adjustCostDeltaForNodeRemoval(delNode);
        
        {
            vector<Symbol*>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                adjustCostDeltaForNodeAddition(*it);
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
        NonTerminal * nt=dynamic_cast<NonTerminal*>(delNode);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        
        {
            vector<Symbol*>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                cfor.addTree(*it);
            }
        }
        delete nt; //TODO: memory leak possible
        
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


}

int main(int argc, char** argv)
{

    vector<Terminal *>  terminals;
    initParsing(argc,argv,terminals);
    
    Forest forest(terminals, RulesDB::SPtr(new RulesDB()));
    forest.runMCMC();
    return 0;
}

