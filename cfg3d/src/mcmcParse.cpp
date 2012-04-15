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
    
    void addNewMoves(Symbol::Ptr tree)
    {
        for(vector<Symbol::Ptr>::iterator it=trees.begin();it!=trees.end();it++)
        {
            if(tree!=(*it))
            {
                 assert(tree->isSpanExclusive((*it)));
                
            }
        }
    }
    
    void addTree(Symbol::Ptr tree)
    {
        trees.push_back(tree);
        addNewMoves(tree);
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
        addNewMoves(tree);
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
 * store nodes also for typechecks
 */
class MergeMove: public Move
{
    int mergeIndex1,mergeIndex2;
    Symbol::Ptr mergeNode1,mergeNode2; // store nodes also for additional safety check
    RulePtr mergeRule;
    Symbol::Ptr mergeResult;
    
public:
    
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

class SplitMove: public Move
{
    int delIndex;
    double transProb;
public:
    
    SplitMove(int delIndex,Forest & cfor)
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
  
int main(int argc, char** argv)
{

    return 0;
}

