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
        childTypeSet.insert(typeid(child).name());
        return lookupRule(childTypeSet);
    }
    
    RulePtr lookupDoubleRule(Symbol::Ptr child1, Symbol::Ptr child2 )
    {
        set<string> childTypeSet;
        childTypeSet.insert(typeid(child1).name());
        childTypeSet.insert(typeid(child2).name());
        return lookupRule(childTypeSet);
    }
    
};

class Move
{
public:
typedef  boost::shared_ptr<Move> SPtr;

    virtual void applyMove(Forest & cfor)=0;
    /**
     * 
     * @return Q(f',f) where f' is the forest which will be generated when
     * applyMove is called on f
     */
    virtual double getTransitionProb()=0;
    
    virtual bool isInvalidatedOnDeletion(int index)=0;
    virtual bool handleMove(int oldIndex, int newIndex)=0;
};


class Forest
{
    vector<Symbol::Ptr> trees;
    vector<Move::SPtr> moves;
    double partitionFunc;
    RulesDB rulesDB;
public:

    /**
     * not supposed to deal with tree-move operation
     * just remove all the moves which referenced it
     * 
     * @param index : the index of tree which was removed
     */
    Forest()
    {
        partitionFunc=0;
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
    
    void sampleNextMove()
    {
        
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
    Symbol::Ptr mergeResult;
    
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
        assert(mergeIndex1!=mergeIndex2);
        
        mergeNode1=cfor.getTree(mergeIndex1);
        mergeNode2=cfor.getTree(mergeIndex2);
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
        
    }
    
    virtual void applyMove(Forest & cfor)
    {
        int maxIndex=std::max(mergeIndex1,mergeIndex2); 
        int minIndex=std::min(mergeIndex1,mergeIndex2);
        
        // safety checks ... in the face of index updates
        assert(cfor.getTree(mergeIndex1)==mergeNode1);
        assert(cfor.getTree(mergeIndex2)==mergeNode2);
        
        cfor.replaceTree(minIndex,mergeResult);
        
        cfor.deleteTree(maxIndex);// max to reduce #index updates in moves
    }
    
    virtual double getTransitionProb()
    {
        
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
        
        mergeNode=cfor.getTree(mergeIndex);
        
        mergeResult=mergeRule->applyRuleMarshalledParams(marshalParams());
        
    }
    
    virtual void applyMove(Forest & cfor)
    {
        
        assert(cfor.getTree(mergeIndex)==mergeNode);
        
        cfor.replaceTree(mergeIndex,mergeResult);
        
    }
    
    virtual double getTransitionProb()
    {
        
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
    double transProb;
public:
    
    SplitMove(Forest & cfor,int delIndex)
    {
        this->delIndex=delIndex;
        
        double cost=0;
        Symbol::Ptr tree=cfor.getTree(delIndex);
        NonTerminal * nt=dynamic_cast<NonTerminal*>(tree);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        
        {
            vector<Symbol*>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                cost=cost+(*it)->getCost();
            }
        }
        
    }
    
    virtual void applyMove(Forest & cfor)
    {
        Symbol::Ptr tree=cfor.getTree(delIndex);
        cfor.deleteTree(delIndex);
        NonTerminal * nt=dynamic_cast<NonTerminal*>(tree);
        assert(nt!=NULL); // cannot delete a Terminal ... maybe a Hallucinated one later
        
        {
            vector<Symbol*>::iterator it;
            for (it = nt->children.begin(); it != nt->children.end(); it++)
            {
                cfor.addTree(*it);
            }
        }
        
    }
    
    virtual double getTransitionProb()
    {
        return transProb;
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
        
        // try         
        
        // try to apply double rules
        for(vector<Symbol::Ptr>::iterator it=trees.begin();it!=trees.end();it++)
        {
            if(tree!=(*it))
            {
                 assert(tree->isSpanExclusive((*it)));
                 if(tree->isSpanExclusive((*it)->getNeigborTerminalBitset()))
                 {
                     RulePtr rul= rulesDB.lookupDoubleRule(tree, *it);
                     if(rul)
                     {
                        moves.push_back(Move::SPtr(new MergeMove(*this, index, it-trees.begin() , rul)));
                     }
                 }
                
            }
        }
        
        // add the delete move for this new node
        moves.push_back(Move::SPtr(new SplitMove(*this, index)));
        
        
    }

int main(int argc, char** argv)
{

    return 0;
}

