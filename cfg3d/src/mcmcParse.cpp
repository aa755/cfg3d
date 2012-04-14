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
    
    MergeMove(Forest & cfor, int mergeIndex1, int mergeIndex2, RulePtr mergeRule)
    {
        this->mergeIndex1=mergeIndex1;
        this->mergeIndex2=mergeIndex2;
        mergeNode1=cfor.getTree(mergeIndex1);
        mergeNode2=cfor.getTree(mergeIndex2);
        
    }
    virtual void applyMove(Forest & cfor)
    {
        
    }
    virtual double getTransitionProb()
    {
        
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
};

int main(int argc, char** argv)
{

    return 0;
}

