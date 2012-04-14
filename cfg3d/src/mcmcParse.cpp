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

class Forest
{
    vector<Symbol::Ptr> trees;
public:
    void deleteTree(int index)
    {
        trees.erase(trees.begin()+index);
    }
    void addTree(Symbol::Ptr tree)
    {
        trees.push_back(tree);
    }
    
    void replaceTree(int index,Symbol::Ptr tree)
    {
        trees.at(index)=tree;
    }
    
    Symbol::Ptr getTree(int index)
    {
        return trees.at(index);
    }
};

class Move
{
public:
    virtual void applyMove(Forest & cfor)=0;
    /**
     * 
     * @return Q(f',f) where f' is the forest which will be generated when
     * applyMove is called on f
     */
    virtual double getTransitionProb()=0;
    
    /**
     * when some tree in the forest is deleted, indices in the moves might 
     * need to be updated. 
     * 
     * @param index : index of the tree which was deleted .. all inddices more 
     * than this should be decremented . if the index was same, the move will become
     * invalid . return true to indicate this
     * 
     * @return true if this move became invalid due to deletion
     */
    virtual bool handleDeletion(int index)=0;
    virtual bool handleReplaceMent(int index)=0;
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
    
    virtual bool handleDeletion(int index)
    {
        if(index==mergeIndex1 || index==mergeIndex2)
            return true;
        
        if(index<mergeIndex1)
            mergeIndex1--;

        if(index<mergeIndex2)
            mergeIndex2--;
     
        return false;
    }
    
    virtual bool handleReplaceMent(int index)
    {
        if(index==mergeIndex1 || index==mergeIndex2)
            return true;
     
        return false;
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

    virtual bool handleDeletion(int index)
    {
        if(index== delIndex)
            return true;
        
        if(index<delIndex)
            delIndex--;

     
        return false;
    }
    
    virtual bool handleReplaceMent(int index)
    {
        if(index== delIndex)
            return true;
        
     
        return false;
    }
};

int main(int argc, char** argv)
{

    return 0;
}

