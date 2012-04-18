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
    map<set<string>, RulePtr> childTypeToRule;
    
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
            childTypeToRule[(*it)->getChildrenTypesAsSet()]=(*it);
        }
        cerr<<"rules map has size: "<<childTypeToRule.size()<<endl;
        
    }
    
    RulePtr lookupRule(set<string> & childTypes)
    {
        map<set<string>, RulePtr>::iterator it= childTypeToRule.find(childTypes);
        if(it==childTypeToRule.end())
        {
            return RulePtr();
        }
        else
        {
            return it->second;
        }
    }
    
    RulePtr lookupSingleRule(Symbol::Ptr child)
    {
        set<string> childTypeSet;
        childTypeSet.insert(typeid(*child).name());
        return lookupRule(childTypeSet);
    }
    
    RulePtr lookupDoubleRule(Symbol::Ptr child1, Symbol::Ptr child2 )
    {
        set<string> childTypeSet;
        childTypeSet.insert(typeid(*child1).name());
        childTypeSet.insert(typeid(*child2).name());
        if(childTypeSet.size()==1)//both same
            return RulePtr();
        return lookupRule(childTypeSet);
    }
    
};

class Move
{
protected:
    double costDelta;
    double transProb;
    bool transProbSet;
    void setTransProbFromDelta()
    {
        transProb=exp(-costDelta);
        transProbSet=true;
    }
    
public:
    virtual string toString()=0;
    double getCostDelta()
    {
        return costDelta;
    }
    
    Move()
    {
        transProb=false;
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
    virtual double getTransitionProbUnnormalized(double curProb /* = 0 */)
    {
        assert(transProbSet);
        return transProb;
    }
    
    virtual bool isInvalidatedOnDeletion(int index)=0;
    virtual bool handleMove(int oldIndex, int newIndex)=0;
};


class Forest
{
    vector<Symbol::Ptr> trees;
    vector<Move::SPtr> moves;
    double curNegLogProb;
    RulesDB::SPtr rulesDB;
public:
    const static double ADDITIONAL_COMPONENT_PENALTY=50;
    const static int NUM_MCMC_ITERATIONS=1000000;

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
        for (unsigned int i = 0; i < terminals.size(); i++)
        {
            addTree(terminals.at(i));
        }

    }
    
    void updateMovesOnDeletion(int index)
    {

        int count = 0;
        int size = moves.size();
        while (count < size)
        {
            Move::SPtr mptr = moves.at(count);
            bool ret = mptr->isInvalidatedOnDeletion(index);
            if (ret)
            {
                fast_erase(moves, count);
                size--;
                assert(size == (int)moves.size());
            }

            count++;
        }
    }
    
    void deleteTree(int index)
    {
        
        Symbol::Ptr oldTree=trees.at(index);
        curNegLogProb-=(oldTree->getCost()+ADDITIONAL_COMPONENT_PENALTY);
        
        updateMovesOnDeletion(index);
        fast_erase(trees,index);
        int oldIndex=trees.size(); // the former last index ... the node there was moved to index
        
        {
            vector<Move::SPtr>::iterator it;
            
            for(it=moves.begin();it!=moves.end();it++)
            {
                (*it)->handleMove(oldIndex,index);
            }
        }
    }
    
    /**
     * create the new moves involving @param tree which was added at @param index
     */
    void addNewMoves(Symbol::Ptr tree, int index);
    
    void addTree(Symbol::Ptr tree)
    {
        trees.push_back(tree);
        addNewMoves(tree,trees.size()-1);
        curNegLogProb+=tree->getCost()+ADDITIONAL_COMPONENT_PENALTY;
    }
    
    /**
     * 
     * @param index
     * @param tree : tree not \in forest
     */
    void replaceTree(int index,Symbol::Ptr tree)
    {
        Symbol::Ptr oldTree=trees.at(index);
        curNegLogProb-=oldTree->getCost();
        updateMovesOnDeletion(index);
        trees.at(index)=tree;
        curNegLogProb+=tree->getCost();
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
    
    int getRandInt(int range)
    {
            return (range*rand())/RAND_MAX;        
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
    
    void runMCMC()
    {
        
        for(int i=0;i<NUM_MCMC_ITERATIONS;i++)
        {
            int nm=sampleNextMoveUniform();
            moves.at(nm)->applyMove(*this);
        }
    }
};



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
    
    vector<Symbol::Ptr> marshalParams()
    {
        vector<Symbol::Ptr> nodes;
        nodes.push_back(mergeNode1);
        nodes.push_back(mergeNode2);
        return nodes;
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
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
        
        
        if(mergeResult==NULL)
            return;

        mergeResult->declareOptimal(false);
        
        costDelta=mergeResult->getCost()-mergeNode1->getCost()-mergeNode2->getCost()-Forest::ADDITIONAL_COMPONENT_PENALTY;
        setTransProbFromDelta();
        
    }
    
    virtual string toString()
    {
        return typeid(*mergeRule).name();
    }
    
    virtual void applyMove(Forest & cfor)
    {
        
        // safety checks ... in the face of index updates
        assert(cfor.getTree(mergeIndex1)==mergeNode1);
        
        cerr<<"mer-move rule "<<typeid(*mergeRule).name()<<endl<<endl;
        
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
    
    virtual bool handleMove(int oldIndex, int newIndex )
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
    
    SingleRuleMove(Forest & cfor, int mergeIndex, RulePtr mergeRule)
    {
        this->mergeIndex=mergeIndex;
        this->mergeRule=mergeRule;
        
        mergeNode=cfor.getTree(mergeIndex);
        
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
      
        if(mergeResult==NULL)
            return;
          mergeResult->declareOptimal(false);
        
        costDelta=mergeResult->getCost()-mergeNode->getCost();
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
    
    virtual bool handleMove(int oldIndex, int newIndex )
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
    string desc;
public:
    
    SplitMove(Forest & cfor,int delIndex)
    {
        this->delIndex=delIndex;
        
        Symbol::Ptr delTree=cfor.getTree(delIndex);
        desc="del:"+string(typeid(*delTree).name());
        NonTerminal * nt=dynamic_cast<NonTerminal*>(delTree);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        costDelta=-(delTree->getCost()+Forest::ADDITIONAL_COMPONENT_PENALTY);
        
        {
            vector<Symbol*>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                costDelta += ((*it)->getCost()+Forest::ADDITIONAL_COMPONENT_PENALTY);
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
        Symbol::Ptr delTree=cfor.getTree(delIndex);
        cfor.deleteTree(delIndex);
        NonTerminal * nt=dynamic_cast<NonTerminal*>(delTree);
      //  delete delTree; //TODO: memory leak possible
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        
        {
            vector<Symbol*>::iterator it;
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
    
    virtual bool handleMove(int oldIndex, int newIndex /* = 0 */)
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

    RulePtr ruls = rulesDB->lookupSingleRule(tree);
    if (ruls)
    {
        moves.push_back(Move::SPtr(new SingleRuleMove(*this, index, ruls)));
    }


    // try to apply double rules
    for (vector<Symbol::Ptr>::iterator it = trees.begin(); it != trees.end(); it++)
    {
        if (tree != (*it))
        {
            cerr<<"db:"<<tree->getName()<<":"<<(*it)->getName()<<endl;
            assert(tree->isSpanExclusive((*it)));
            if (tree->isSpanExclusive((*it)->getNeigborTerminalBitset()))
            {
                
                RulePtr rul = rulesDB->lookupDoubleRule(tree, *it);
                if (rul)
                {
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

