/*
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
#include"point_types.h"
#include "point_struct.h"
#include<pcl/features/normal_3d.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<queue>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdlib.h>
#include <time.h>
#include <boost//lexical_cast.hpp>
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#define TABLE_HEIGHT .75
#include <stack>
#include "point_struct.h"
#include "utils.h"

//sac_model_plane.h

using namespace Eigen;
using namespace std;
typedef pcl::PointXYZRGBCamSL PointT;
/*
 *
 */

/**
 * does in-place set intersection ...O(n) amortized... result is in set_1
 * @param set_1
 * @param set_2
 */
template<typename T>
void setIntersection(std::set<T> & set_1, std::set<T> & set_2) {
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end())) {
        if (*it1 < *it2) {
            set_1.erase(it1++);
        } else if (*it2 < *it1) {
            ++it2;
        } else { // *it1 == *it2
            ++it1;
            ++it2;
        }
    }

    set_1.erase(it1, set_1.end());
}

template<typename T>
void setDifference(std::set<T> & set_1, std::set<T> & set_2) {
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end())) {
        if (*it1 < *it2) {
            //set_1.erase(it1++);
            it1++;
        } else if (*it2 < *it1) {
            ++it2;
        } else { // *it1 == *it2
            set_1.erase(it1++);
            ++it2;
        }
    }

}

class NonTerminal;
class Terminal;

class NTSetComparison {
public:
    bool operator() (NonTerminal * const & lhs, NonTerminal * const & rhs);
};

double infinity() {
    return numeric_limits<double>::infinity();
}

typedef set<NonTerminal*, NTSetComparison> NTSet;

pcl::PointCloud<PointT> scene;

class Symbol {
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG
     */
    double cost;
    double maxZ;
    double zSquaredSum;
    AdvancedDynamicBitset neighbors;
    vector<NonTerminal*> optimalParents;
    pcl::PointXYZ centroid;
    long numPoints; // later, pointIndices might not be computed;

    //    vector<NonTerminal*> parents;
public:
    virtual string getName()=0;

    void pushEligibleNonDuplicateOptimalParents(Symbol *extractedSym, stack<NonTerminal*> & eligibleNTs, long iterationNo);

    virtual bool isSpanExclusive(NonTerminal * nt) = 0;

    bool isNeighbor(int terminalIndex) {
        return neighbors.test(terminalIndex);
    }

    boost::dynamic_bitset<> & getNeigborTerminalBitset() {
        return neighbors;
    }
    
    virtual void computeCentroid()=0 ;

    virtual void printData() =0;
    
    virtual void computeMaxZ()=0;

    void resetNeighborIterator()
    {
        assert(neighbors.size()>0);
        neighbors.iteratorReset();
    }
    
    bool nextNeighborIndex( int & index)
    {
        return neighbors.nextOnBit(index);
    }
    //virtual void insertPoints(vector<int> & points)=0;

    virtual void unionMembership(boost::dynamic_bitset<> & set_membership) = 0;

    bool operator<(const Symbol& rhs) {
        return cost < rhs.cost;
    }

    double getCost() const {
        return cost;
    }

    virtual vector<int> & getPointIndices() = 0;

    virtual size_t getNumTerminals() = 0;

    virtual void getCentroid(pcl::PointXYZ & centroid_)
    {
        centroid_=centroid;
    }

    virtual bool declareOptimal( vector<Terminal*> & terminals) = 0;

    //virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
    //    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;

    virtual double getMaxZ() {
        return maxZ;
    }
    
    virtual void computeZSquaredSum() = 0;
    
    virtual double getZSquaredSum() {
        return zSquaredSum;
    }
        
    double computeZMinusCSquared(double c) 
    {
        return zSquaredSum - 2 * centroid.z * numPoints * c + numPoints * c*c;
    }
    
    void computeFeatures()
    {
        computeZSquaredSum();
        computeMaxZ();
        computeCentroid();
    }
    
    virtual int getId() = 0;

    /**
     * Adds node to the list of optimal parents of a node for bookkeeping.
     * @param node: node to add
     */
    void appendOptimalParents(NonTerminal* node) {
        optimalParents.push_back(node);
    }
    
    long getNumPoints() const
    {
        return numPoints;
    }
    //    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)=0;
};

class SymbolComparison {
public:

    bool operator() (Symbol * & lhs, Symbol * & rhs) const {
        return lhs->getCost() > rhs->getCost();
    }
};

//typedef priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> SymbolPriorityQueue;
/**
 * also supports hashing
 */
class SymbolPriorityQueue;

class Terminal : public Symbol {
protected:
    /** segment index
     */
    size_t index;
    vector<int> pointIndices;
    
public:
    static int totalNumTerminals;
    
    boost::dynamic_bitset<> & getNeighbors() {
        return neighbors;
    }

    string getName()
    {
        return boost::lexical_cast<std::string>(index+1);
    }
    
    /**
     * initialize the neighbors vector
     * @param nbrs : set of netgbor segment indices (terminal index=segIndex-1)
     * @param numTerminals : total Number of terminals
     */
    void setNeighbors(set<int> & nbrs, int numTerminals)
    {
        neighbors.resize(numTerminals,false);
        set<int>::iterator it;
        for(it=nbrs.begin();it!=nbrs.end();it++)
        {
            neighbors.set((*it)-1, true);
        }
    }
    
    bool isSpanExclusive(NonTerminal * nt);

    /**
     * will be only used  to compute cost of rules . CFG parsing functions should
     *  not use it
     * @return
     */
    vector<int> & getPointIndices() {
        return pointIndices;
    }
    
    void computeZSquaredSum() 
    {
        double costSum = 0;
        for (vector<int>::iterator it = pointIndices.begin(); it != pointIndices.end(); it++) {
            costSum = costSum + (scene.points[*it].z  * scene.points[*it].z);
        }
        zSquaredSum = costSum;
    }
    
    void computeMaxZ() {
        double greatestMaxZ = -infinity();
        double itMaxZ = 0;
        for (vector<int>::iterator it = pointIndices.begin(); it != pointIndices.end(); it++) {
            itMaxZ = scene.points[*it].z;
            if (itMaxZ > greatestMaxZ) {
                greatestMaxZ = itMaxZ;
            }
        }
        maxZ = greatestMaxZ;
    }
    
    void computeCentroid() {
        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
        for (size_t i = 0; i < pointIndices.size(); i++) {
            PointT & point = scene.points[pointIndices[i]];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        numPoints=pointIndices.size();
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;
    }
    
    
    void addPointIndex(int index)
    {
        pointIndices.push_back(index);
    }
    
    int getId() {
        return 0;
    }

    /**
     * add this terminal to the set
     * @param set_membership
     */
    void unionMembership(boost::dynamic_bitset<> & set_membership) {
        set_membership.set(index, true);
    }

    Terminal() {
        index = -1; /// for safety;
        cost = 0;
    }

    Terminal(int index_) {
        index = index_;
        cost = 0;
    }

    Terminal(int index_, double cost_) {
        index = index_;
        cost = cost_;
    }

    int getIndex() const {
        return index;
    }


    void printData() {
        cout << "t\t:" << index << endl;
    }

    bool declareOptimal(vector<Terminal*> & terminals) {
        return true;
    }

    //bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)    {
    //        return false; // each terminal can be derived in only 1 way
    /* contrast it with a set of terminals which can be derived in multiple way
     * (1 U 2) U 3  or  1 U (2 U 3 )
     */
    // }

    size_t getNumTerminals() {
        return 1;
    }

    /*   void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
       {
           thisAncestors=allAncestors[index];
       }
     */
};
Terminal * terminals;
int Terminal::totalNumTerminals = 0;

class NonTerminal : public Symbol {
protected:
    bool duplicate;
    size_t numTerminals;
    //    vector<bool> setOfPoints;
    AdvancedDynamicBitset spanned_terminals;
    
    int id;
    /**
     * version number
     */
    long lastIteration;
    static int id_counter;
    /** indices into the pointcloud
     */
    //will be populated only when extracted as min

    void computeCentroid() {
        pcl::PointXYZ childCent;
        numPoints=0;
        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
        
        for (size_t i = 0; i < children.size(); i++) {
            children.at(i)->getCentroid(childCent);
            long numPointsInChild=children.at(i)->getNumPoints();
            numPoints+=numPointsInChild;
            centroid.x += numPointsInChild*childCent.x;
            centroid.y += numPointsInChild*childCent.y;
            centroid.z += numPointsInChild*childCent.z;
        }
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;
    }

    bool isSpanExclusive(NonTerminal * nt) {
        return !(spanned_terminals.intersects(nt->spanned_terminals));
    }

    /**
     * compute leaves by concatenating
     * leaves of children */
    bool costSet;
public:
    vector<Symbol*> children;
    friend class RPlanePair_PlanePlane;
    friend class Terminal;
    
     void resetTerminalIterator()
    {
        assert(spanned_terminals.size()>0);
        spanned_terminals.iteratorReset();
    }
    
    bool nextTerminalIndex( int & index)
    {
        return spanned_terminals.nextOnBit(index);
    }
   
    size_t getNumChildren()
    {
        return children.size();
    }
    
    Symbol * getChild(int i)
    {
        return children.at(i);
    }
    
    string getName()
    {
        return string(typeid(*this).name()+1)+boost::lexical_cast<std::string>(id);
    }
    
    vector<int> pointIndices; // can be replaced with any sufficient statistic

    void computePointIndices(vector<Terminal*> & terminals) {
        if(pointIndices.size()>0)
            return ;
        // pointIndices.reserve(getNumTerminals());

        for (int i = 0; i < Terminal::totalNumTerminals; i++) {
            if (spanned_terminals.test(i))
                pointIndices.insert(pointIndices.end(), terminals.at(i)->getPointIndices().begin(), terminals.at(i)->getPointIndices().end());
        }
    }

    /**
     * is span(this) U span(nt) =all terminals 
     * @param nt
     * @return 
     */
    bool isMutuallyExhaustive(NonTerminal *nt)
    {
        
        boost::dynamic_bitset<> span_union=spanned_terminals | nt->spanned_terminals;
        //exhaustive <=> all 1s
        span_union.flip();
        //exhaustive <=> all 0's <=> none is 1
        return(span_union.none());
    }
    
    vector<int> & getPointIndices() {
        assert(pointIndices.size() > 0);
        return pointIndices;
    }

    bool intersects(NonTerminal * other) {
        return spanned_terminals.intersects(other->spanned_terminals);
    }

    int getId() {
        return id;
    }

    NonTerminal() {
        costSet = false;
        id = id_counter++;
        cout << "nNT: " << id << endl;
        numTerminals = 0;
        lastIteration = 0;
        duplicate=false;
    }
    
    friend class NTSetComparison;

    void printData() {
        cout << id << "\t:" << spanned_terminals << endl;
    }

    size_t getNumTerminals() {
        assert(numTerminals > 0);
        return numTerminals;
    }

    void setAdditionalCost(double additionalCost) {
        assert(additionalCost >= 0);
        cost = 0;
        for (size_t i = 0; i < children.size(); i++)
            cost += children[i]->getCost();
        cost += additionalCost;
        costSet = true;
        cout << "ac: " << additionalCost << " tc: " << cost << endl;
    }

    /**
     * caution: use this only when you can prove that this cost
     * larger than the cost of any children
     * @param absoluteCost
     */
    void setAbsoluteCost(double absoluteCost) {
        assert(absoluteCost >= 0);
        cost = absoluteCost;
        costSet = true;
        cout << "absc: " << cost << endl;
    }

    void addChild(Symbol * child) {
        assert(!costSet);
        children.push_back(child);
    }

    void computeSpannedTerminals() {
        spanned_terminals.resize(Terminal::totalNumTerminals, false);
        for (size_t i = 0; i < children.size(); i++) {
            children[i]->unionMembership(spanned_terminals);
        }
        numTerminals = spanned_terminals.count();
    }

    /***
     * Iterates through all neighbors of NonTerminal node's children,
     * computing the union of all children neighbors minus the span of the
     * NonTerminal node.
     */
    void computeNeighborTerminalSet() {
        neighbors.resize (Terminal::totalNumTerminals,false);
        vector<Symbol*>::iterator it;
        for (it = children.begin(); it != children.end(); ++it) {
            neighbors |= (*it)->getNeigborTerminalBitset();
        }
        neighbors -= spanned_terminals;
    }

    void computeMaxZ() {
        vector<Symbol*>::iterator it;
        double greatestMaxZ = -infinity();
        double itMaxZ;
        for (it = children.begin(); it != children.end(); ++it) {
            itMaxZ = (*it)->getMaxZ();
            if (itMaxZ > greatestMaxZ) {
                greatestMaxZ = itMaxZ;
            }
        }
        maxZ = greatestMaxZ;
    }
    
    void computeZSquaredSum() {
        double sum = 0;
        vector<Symbol*>::iterator it;
        for(it = children.begin(); it != children.end(); it++) {
                sum = sum + (*it)->getZSquaredSum();
        }
        zSquaredSum = sum;
    }
    
    void unionMembership(boost::dynamic_bitset<> & set_membership) {
        //        assert(pointIndices.size()>0);
        set_membership |= this->spanned_terminals;
    }

    /**
     * do some book-keeping
     * for efficiency it is not done when a new NT is created, but only when it is extracted
     * from priority queue
     */

    /**
     * declare it optimal and hence do the bookkeeping necesary for combining it with others
     * @param allNTs
     * @param terminals
     * @return
     */
    bool declareOptimal( vector<Terminal*> & terminals) {
        vector<Symbol*>::iterator it;

        for (it = children.begin(); it != children.end(); ++it) {
            (*it)->appendOptimalParents(this);
        }

        computeNeighborTerminalSet();
        assert(costSet); // cost must be set before adding it to pq
        computePointIndices(terminals);
        computeFeatures();
        additionalFinalize();
        return true;
    }


    /**
     * For a NonTerminal node X, we only need to call additionalFinalize() if the cost 
     * of X is not dependent on this statistic in a Rule X->YZ but is required in a Rule
     * A->XB where the cost of A is dependent on this statistic of X. 
     * 
     * In creating X->YZ, X is not necessarily optimal and statistic is not needed in creating X, 
     * but in A->XB, X is optimal and is needed in creating A, so we calculate the statistic when 
     * we declare X optimal.
     * 
     * Only when X is declared optimal and used to combine with other NTs is it necessary to use this statistic.
     */
    virtual void additionalFinalize() {}

    /**
     * We check that sym is not the same type as this NonTerminal nor spanning 
     * the same terminals as this NonTerminal.
     * @param sym
     * @return 
     */
    bool isDuplicate(NonTerminal * sym) {
        assert(spanned_terminals.size()>0);
        assert(sym->spanned_terminals.size()>0);
        
        if (typeid (*sym) != typeid (*this))
            return false;
        if (sym->spanned_terminals != spanned_terminals)
            return false;
        return true;
    }

    void setLastIteration(long lastIteration)
    {
        this->lastIteration = lastIteration;
    }

    long getLastIteration() const
    {
        return lastIteration;
    }

    void markDuplicate()
    {
        this->duplicate = true;
    }

    bool isMarkDuplicate() const
    {
        return duplicate;
    }

    /*    void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
        {
            thisAncestors=allAncestors[pointIndices[0]];
            for(size_t i=1;i<getNumPoints();i++)
            {
                 set<NonTerminal*> & temp=allAncestors[pointIndices.at(i)];
                 thisAncestors.insert(temp.begin(),temp.end());
            }
        }
     */
};

bool NTSetComparison::operator() (NonTerminal * const & lhs, NonTerminal * const & rhs) {
    //start with MSBs
    for (int i = lhs->spanned_terminals.num_blocks() - 1; i >= 0; i--) {
        if (lhs->spanned_terminals.m_bits[i] > rhs->spanned_terminals.m_bits[i])
            return true;
        else if (lhs->spanned_terminals.m_bits[i] < rhs->spanned_terminals.m_bits[i])
            return false;
        // else look the lower significant block

    }
    return false; // actually they are equal
}

/**
 * Check that the Terminal nodes spanned by both NonTerminals are disjoint.
 * @param nt
 * @return 
 */
bool Terminal::isSpanExclusive(NonTerminal* nt) {
    return !(nt->spanned_terminals.test(index));
}

void Symbol::pushEligibleNonDuplicateOptimalParents(Symbol *extractedSym, stack<NonTerminal*> & eligibleNTs, long iterationNo) {
//    assert(1 == 2); // check duplicate
    for (size_t i = 0; i < optimalParents.size(); i++)
    {
        if (optimalParents.at(i)->getLastIteration() < iterationNo && extractedSym->isSpanExclusive(optimalParents.at(i)))
        {
            optimalParents.at(i)->setLastIteration(iterationNo);
            eligibleNTs.push(optimalParents.at(i));
        }
    }
}

int NonTerminal::id_counter = 0;

/**
 * when an NT is extracted, this class will help in finding possible candidates
 * for combination. it uses a stack now, because, it might be useful
 * later to ignore all the parents of an (bad)NT
 */
class FindNTsToCombineWith {
    stack<NonTerminal*> eligibleNTs;
    long iterationNo;
    Symbol *extractedSym;

public:
    FindNTsToCombineWith(Symbol * sym, vector<Terminal*> & allTerminals, long iterationNo_) {
        extractedSym = sym;
        iterationNo = iterationNo_;
        for (int i = 0; i < Terminal::totalNumTerminals; i++) {
            if (sym->isNeighbor(i)) {
                allTerminals.at(i)->pushEligibleNonDuplicateOptimalParents(sym, eligibleNTs,iterationNo);
            }
        }

    }

    NonTerminal * nextEligibleNT() {
        if (eligibleNTs.empty())   
            return NULL;

        NonTerminal *ret = eligibleNTs.top(); // only unvisited and eligible items were pushed to stack
        eligibleNTs.pop();

        //push it's parents which are eligible
        // to add a possibility of ignoring all it's ancestors, postpone
        //next step to nest call by temporarily storing return value
        ret->pushEligibleNonDuplicateOptimalParents(extractedSym, eligibleNTs,iterationNo);
        return ret;
    }
};

/**
 * abstraction of a priority queue which can do additional book-keeping later
 * duplicate check needs to be done on all members, even those whioch have been extracted from PQ
 * check for duplicate:
 *      if duplicate in sth already extracted from PQ
 *              don't do anything
 *      else if found in PQ
 *              if cost is more than that in PQ
 *                      ignore
 *              else
 *                      add to PQ
 *                      if possible, delete the copy which was present in PQ
 *
 *
 * consider using Trie
 *
 */
class SymbolPriorityQueue {
    /**these 2 contain same elements ... stored in a different way
     */
    priority_queue<Symbol *, vector<Symbol *>, SymbolComparison> costSortedQueue; // optimized to give least cost
    map<string,vector<NTSet> > NTsetsInPQ; // optimized for duplicate check
    
   
    map<string,vector<NTSet> > NTsetsExtracted;

    NTSet & getBin(NonTerminal * sym, map<string,vector<NTSet> > & bins)
    {
        int numTerminals = sym->getNumTerminals();
        assert(numTerminals > 0);
        vector<NTSet> & typeBin=bins[string(typeid(*sym).name())];

        if(typeBin.size()==0)
            typeBin.resize(totalNumTerminals,NTSet()); // first time => allocate bins
        
        return typeBin.at(numTerminals-1);
        
    }

    NTSet & getBinOfExtractedSymbols(NonTerminal * sym)
    {
        return getBin(sym,NTsetsExtracted);
    }
    
    NTSet & getBinOfPQSymbols(NonTerminal * sym)
    {
        return getBin(sym,NTsetsInPQ);        
    }

    bool CheckIfBetterDuplicateExists(NonTerminal * sym) {
        assert(4==2); //not used for now
        NTSet & bin = getBinOfExtractedSymbols(sym);
        NTSet::iterator lb;
        lb = bin.lower_bound(sym);
        if (lb != bin.end() && (*lb)->isDuplicate(sym)) {
            //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            return true;
        }

        NTSet & bin1 = getBinOfPQSymbols(sym);
        lb = bin1.lower_bound(sym);
        if (lb != bin1.end() && (*lb)->isDuplicate(sym)) {
            //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            if (sym->getCost() >= (*lb)->getCost())
                return true;
        }
        return false;
    }


    int totalNumTerminals;
public:

    
    
    SymbolPriorityQueue(int totalNumTerminals_) //: NTsetsExtracted(numTerminals, NTSet()), NTsetsInPQ(numTerminals, NTSet()) {
    {
        totalNumTerminals=totalNumTerminals_;
    }


    /**
     * extracted before => was better or same cost
     *
     */
    bool CheckIfBetterDuplicateWasExtracted(NonTerminal * sym) {
        NTSet & bin = getBinOfExtractedSymbols(sym);
        NTSet::iterator lb;
        lb = bin.lower_bound(sym);
        if (lb != bin.end() && (*lb)->isDuplicate(sym)) {
            return true;
        }

        return false;
    }

    /**
     * also check for duplicate and don't add if a duplicate with smaller cost
     * exists
     * @param sym
     * @return true if inserted
     */
    bool pushIfNoBetterDuplicateExistsUpdateIfCostHigher(NonTerminal * sym) {
        if (CheckIfBetterDuplicateWasExtracted(sym))
            return false;

        NTSet & bin1 = getBinOfPQSymbols(sym);
        NTSet::iterator lb;
        lb = bin1.lower_bound(sym);
        if (lb != bin1.end() && (*lb)->isDuplicate(sym)) {
            //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            if (sym->getCost() < (*lb)->getCost())
            {
                // duplicate of higher cost already exixted => update
                //PQ order can't be updated so add an duplicate pointer to the same object
                // when this object will be extracted, set the optimal field
                (*lb)->markDuplicate();
                bin1.erase(lb);
                bin1.insert(--lb,sym);
                //(*lb)=sym; // the set is not sorted by cost, it is sorted by bitset => order is same
                costSortedQueue.push(sym);
                return true;
            }
            
            //else there is already a duplicate of lower cost => no change required
            return false ; //in both cases, the pointer should be deleted
        }
        else
        {
            bin1.insert(sym);
            costSortedQueue.push(sym);
            return true;
        }

//        std::pair<NTSet::iterator, bool> ret =getBinOfPQSymbols(sym).insert(sym); // will be inserted only if not duplicate
//        if(ret.second)
//            costSortedQueue.push(sym);
//        return ret.second; // true if inserted, else duplicate
    }

    /**
     * no need to check for duplicate
     * @param sym
     */
    void pushTerminal(Terminal * sym) {
        costSortedQueue.push(sym);
    }

    Symbol * pop(bool & duplicate) {
        if(costSortedQueue.empty())
            return NULL;
        Symbol * top = costSortedQueue.top();
        costSortedQueue.pop();
        duplicate=false;
        if (typeid (*top) != typeid (Terminal)) {
            NonTerminal * nt = dynamic_cast<NonTerminal *> (top);
            
            if(nt->isMarkDuplicate())
            {
                duplicate=true;
                //cout<<"mdup"<<endl;
                return top;
            }
            
            std::pair<NTSet::iterator, bool> res=getBinOfExtractedSymbols(nt).insert(nt); // set => no duplicates
            assert(res.second);
            getBinOfPQSymbols(nt).erase(nt);
        }

        return top;
    }

    size_t size() {
        return costSortedQueue.size();
    }

};

class Rule {
public:
    /**
     * @param extractedSym : the extracted Symbol, guaranteed not to be a duplicate of anything was already extracted ... 
     * @param pqueue : the priority queue
     * @param terminals : set of all terminals
     */
    virtual void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */) = 0;

    virtual void addToPqueueIfNotDuplicate(NonTerminal * newNT, SymbolPriorityQueue & pqueue) {
      //  newNT->computeSetMembership(); // required for duplicate check
        if (!pqueue.pushIfNoBetterDuplicateExistsUpdateIfCostHigher(newNT))
            delete newNT;
    }
};

template<typename LHS_Type, typename RHS_Type1, typename RHS_Type2 >
class DoubleRule : public Rule
{
    //    template<typename RHS_Type1, typename RHS_Type2>

    void combineAndPushForParam1(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {

        RHS_Type1 * RHS_extracted = dynamic_cast<RHS_Type1 *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL)
        {
            if (typeid (*nt) == typeid (RHS_Type2))
            {
                RHS_Type2 * RHS_combinee = dynamic_cast<RHS_Type2 *> (nt);
                addToPqueueIfNotDuplicate(applyRule (RHS_extracted, RHS_combinee), pqueue);
            }
            nt = finder.nextEligibleNT();
        }
    }

    //    template<typename RHS_Type1, typename RHS_Type2>

    void combineAndPushForParam2(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {

        RHS_Type2 * RHS_extracted = dynamic_cast<RHS_Type2 *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL)
        {
            if (typeid (*nt) == typeid (RHS_Type1))
            {
                RHS_Type1 * RHS_combinee = dynamic_cast<RHS_Type1 *> (nt);
                addToPqueueIfNotDuplicate(applyRule(RHS_combinee, RHS_extracted), pqueue);
            }
            nt = finder.nextEligibleNT();
        }
    }

    //    template<typename RHS_Type1, typename RHS_Type2>

    void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) == typeid (RHS_Type1))
        {
            combineAndPushForParam1(extractedSym, pqueue, terminals, iterationNo);
        }
        else if (typeid (*extractedSym) == typeid (RHS_Type2))
        {
            combineAndPushForParam2(extractedSym, pqueue, terminals, iterationNo);
        }
    }

public:
    /**
     * This must be overriden by the inheriting class as each Rule will have its own specific cost function.
     * @param output
     * @param input
     */
    void setCost(LHS_Type* output, RHS_Type1 * RHS1, RHS_Type2 * RHS2)
    {
        assert(3 == 2); // needs specialization
    }
        
    NonTerminal* applyRule(RHS_Type1 * RHS1, RHS_Type2 * RHS2)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS1);
        LHS->addChild(RHS2);
        LHS->computeSpannedTerminals();
        setCost(LHS,RHS1,RHS2);
        return LHS;
    }

    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        combineAndPushGeneric (extractedSym, pqueue, terminals, iterationNo);
    }
}; 

template<typename LHS_Type, typename RHS_Type>
class SingleRule : public Rule
{
    void combineAndPushForParam(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        RHS_Type* RHS_extracted = dynamic_cast<RHS_Type *>(extractedSym);
        addToPqueueIfNotDuplicate(applyRule(RHS_extracted), pqueue);
    }

    void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) == typeid (RHS_Type))
        {
            combineAndPushForParam(extractedSym, pqueue, terminals, iterationNo);
        }
    }

public:
    
     /**
     * This must be overriden by the inheriting class as each Rule will have its own specific cost function.
     * @param output
     * @param input
     */
    void setCost(LHS_Type* output, RHS_Type* input)
    {
        assert(3 == 2);
    }
        
    NonTerminal* applyRule(RHS_Type* RHS)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS);
        LHS->computeSpannedTerminals();
        setCost(LHS, RHS);
        return LHS;
    }

    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        combineAndPushGeneric(extractedSym, pqueue, terminals, iterationNo);
    }
}; 

   
double sqr(double d) {
    return d*d;
}

class Plane : public NonTerminal {
protected:
    float curvature;
    bool planeParamsComputed;
    Eigen::Vector4f planeParams;
public:

    double zSquaredSum;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Plane() : NonTerminal() {
        planeParamsComputed = false;
    }

    double getNorm() {
        return (planeParams[0] * planeParams[0] + planeParams[1] * planeParams[1] + planeParams[2] * planeParams[2]);
    }

    void computePlaneParams() {
        pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        assert(fabs(getNorm()-1)<0.05);
     //   double norm = getNorm();
     //   planeParams /= norm;
        planeParamsComputed = true;
    }
    
    Eigen::Vector3d getPlaneNormal() {
        return Vector3d(planeParams[0], planeParams[1], planeParams[2]);
    }
    
    Eigen::Vector4f getPlaneParams() {
        return planeParams;
    }
    
    double coplanarity(Plane * plane2) {
        return fabs(planeParams[0] * plane2->planeParams[0] + planeParams[1] * plane2->planeParams[1] + planeParams[2] * plane2->planeParams[2]);
    }

    double costOfAddingPoint(PointT & p) {
        if (pointIndices.size() == 2) {
            /*return it's min distance to either points
             */
            double d1 = pcl::euclideanDistance<PointT, PointT > (p, scene.points[pointIndices[0]]);
            double d2 = pcl::euclideanDistance<PointT, PointT > (p, scene.points[pointIndices[1]]);
            if (d1 > d2)
                return d1;
            else
                return d2;
        } else if (pointIndices.size() >= 3) {
            assert(planeParamsComputed);
            //            return exp(100*pcl::pointToPlaneDistance<PointT>(p,planeParams))-1;
            return getNumTerminals()*(exp(pcl::pointToPlaneDistance<PointT > (p, planeParams)) - 1);
        } else
            assert(4 == 2);
    }

    virtual void setCost() {
        setAbsoluteCost(sumDistancesSqredToPlane(this));
    }
    
    /**
     * computes the sum of distances of all points spanned by symbol t from this plane
     * using distance squared since, the plane parameters are obtained by using a least squares fit
     * http://www.ros.org/doc/unstable/api/pcl/html/feature_8h.html
     * @param t
     * @return 
     */
    double sumDistancesSqredToPlane(Symbol * t) {
            assert(planeParamsComputed);
            double sum=0;
        for (unsigned int i = 0; i < t->getPointIndices().size(); i++) {
            PointT & p=scene.points[t->getPointIndices().at(i)];
                sum+=sqr(pcl::pointToPlaneDistance<PointT > (p, planeParams));
        }
            return sum;
            
    }
    
};

class Floor : public Plane {
public: 
    /**
     * The cost of considering a Plane as the Floor is simply the distance of the Plane
     * to the canonical z = 0 Plane. How are we calculating Segment to Plane "distances"?
     */
    void setCost() {
        Plane* plane = dynamic_cast<Plane*> (children.at(0));
        setAbsoluteCost(plane->getZSquaredSum());
    }
};

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
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePointIndices(terminals);
        LHS->computePlaneParams();
        LHS->setCost();
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

class RPlane_Seg : public Rule {
public:


    NonTerminal* applyRule(Terminal *RHS_seg,vector<Terminal*> & terminals) {
        Plane * LHS = new Plane();
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePointIndices(terminals);
        LHS->computePlaneParams();
        LHS->setCost();
                
        return LHS;
    }

    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */) {
        if (typeid (*extractedSym) == typeid (Terminal))
        {
                Terminal * term=dynamic_cast<Terminal*> (extractedSym);
                NonTerminal *newNT=applyRule(term,terminals);
                addToPqueueIfNotDuplicate(newNT,pqueue);
        }
    }

    
};

class PlanePair : public NonTerminal {
protected:
    Eigen::Vector3d crossProduct;
public:
    /**
     * Computes the cross product between two planes.
     */
    void computeCrossProduct() {
        Plane * RHS_plane1=dynamic_cast<Plane *>(children.at(0));
        Plane * RHS_plane2=dynamic_cast<Plane *>(children.at(1));
                
        Eigen::Vector4f aNormal = RHS_plane1->getPlaneParams();
        Eigen::Vector4f bNormal = RHS_plane2->getPlaneParams();
        Vector3d v(aNormal[0],aNormal[1],aNormal[2]);
        Vector3d w(bNormal[0],bNormal[1],bNormal[2]);
        crossProduct=v.cross(w);
    }

    PlanePair() : NonTerminal() {

    }

     Eigen::Vector3d getCrossProduct() const {
         return crossProduct;
     }
     
     /**
      * The cross product of PlanePair needed for calculating cost of applying Rule: 
      * Corner->PlanePair Plane (only need to calculate cost(Corner) if PlanePair
      * is optimal so we call additionalFinalize() only when we have declared PlanePair optimal
      * to reduce unnecessary computation.
      */
     void additionalFinalize() {
         computeCrossProduct();
     }
};

class RPlanePair_PlanePlane : public Rule {
public:
    NonTerminal* applyRule(Plane * RHS_plane1, Plane * RHS_plane2)
    {
        PlanePair * LHS=new PlanePair();
        LHS->addChild(RHS_plane1);
        LHS->addChild(RHS_plane2);
        LHS->setAdditionalCost(RHS_plane1->coplanarity(RHS_plane2)/*+exp(10*deficit)*/); // more coplanar => bad
        LHS->computeSpannedTerminals();
        return LHS;
    }
    
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals , long iterationNo /* = 0 */)
    {
        if(typeid(*extractedSym)==typeid(Plane))
        {
                    Plane * RHS_plane1=dynamic_cast<Plane *>(extractedSym);
                FindNTsToCombineWith finder(extractedSym,terminals,iterationNo);
                NonTerminal * nt=finder.nextEligibleNT();
                
                //int count=0;
                while(nt!=NULL)
                {
//                    nt->printData(); //checked that duplicates not extracted, and exhaustive
                  //  count++;
//                    if(typeid(*nt)==typeid(Plane) &&  nt->isMutuallyExhaustive(RHS_plane1))
                    if(typeid(*nt)==typeid(Plane) )
                    {
                        Plane * RHS_plane2=dynamic_cast<Plane *>(nt);
                        addToPqueueIfNotDuplicate(applyRule(RHS_plane1,RHS_plane2),pqueue);
                    }
                    nt=finder.nextEligibleNT();
                }
              //  cout<<"nnc: "<<count<<endl;
        }
    }

};

class Corner : public NonTerminal {
    // TODO: Do we need anything in here? This class feels cold and empty inside.
};

class RCorner_PlanePairPlane : public Rule {
public:
    /**
     * Creates a new Corner object, setting Corner's additional cost to equal the dot product
     * of PlanePair's cross product and Plane's normal.
     */
    NonTerminal* applyRule(PlanePair* RHS_planePair, Plane* RHS_plane, vector<Terminal*>& terminals) {
        Corner* LHS=new Corner();
        LHS->addChild(RHS_planePair);
        LHS->addChild(RHS_plane);
        Vector3d planePairCrossProduct = RHS_planePair->getCrossProduct();
        Vector3d planeNormal(RHS_plane->getPlaneNormal());
        LHS->setAdditionalCost(1-fabs(planePairCrossProduct.dot(planeNormal)));
        LHS->computeSpannedTerminals();
       // LHS->computePointIndices(terminals);
        return LHS;
    }

    /**
     * Iterates through all possible NTs to combine with and combines 
     * extractedSym with Plane if extractedSym is PlanePair and combines
     * extractedSym with PlanePair if extractedSym is Plane.
     * @param extractedSym
     * @param pqueue
     * @param terminals
     * @param iterationNo
     */
    void combineAndPush(Symbol* extractedSym, SymbolPriorityQueue& pqueue, 
        /**
         * We can definitely try to reduce redundancy in code here.
         */
        vector<Terminal*>& terminals, long iterationNo) {
        if(typeid(*extractedSym) == typeid(PlanePair)) {
            PlanePair* RHS_planePair = dynamic_cast<PlanePair*>(extractedSym);
            FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
            NonTerminal* nt = finder.nextEligibleNT();

            while(nt != NULL) {
                if(typeid(*nt) == typeid(Plane)) {
                    Plane* RHS_plane = dynamic_cast<Plane*>(nt);
                    addToPqueueIfNotDuplicate(applyRule(RHS_planePair, RHS_plane, terminals), pqueue);
                }
                nt = finder.nextEligibleNT();
            }
        } else if (typeid(*extractedSym) == typeid(Plane)) {
            Plane* RHS_plane = dynamic_cast<Plane*>(extractedSym);
            FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
            NonTerminal* nt = finder.nextEligibleNT();

            while(nt != NULL) {
                if(typeid(*nt) == typeid(PlanePair)) {
                    PlanePair* RHS_planePair = dynamic_cast<PlanePair*>(nt);
                    addToPqueueIfNotDuplicate(applyRule(RHS_planePair, RHS_plane, terminals), pqueue);
                }
                nt = finder.nextEligibleNT();
            }
        }
    }
};

class RFloor_Plane : public Rule {
    
public:
    /**
     * Creates a new Floor object, setting Floor's absolute cost to equal 
     * the Plane's points' distances to the canonical z-plane.
     */
    NonTerminal* applyRule(Plane* RHS_plane, vector<Terminal*>& terminals) {
        Floor* LHS = new Floor();
        LHS->addChild(RHS_plane);
        LHS->computeSpannedTerminals();
        LHS->computePointIndices(terminals);
        LHS->setAbsoluteCost(RHS_plane->getZSquaredSum());
        return LHS;
    }
    
        /** 
     * Simply checks if extractedSym is of type Plane and creates a Plane object 
         * assigning to the Plane object the cost of considering the Plane as a Floor.
     * @param extractedSym
     * @param pqueue
     * @param terminals
     * @param iterationNo
     */
    void combineAndPush(Symbol* extractedSym, SymbolPriorityQueue& pqueue, 
        vector<Terminal*>& terminals, long iterationNo) {
        if(typeid(*extractedSym) == typeid(Plane)) {
            Plane* RHS_plane = dynamic_cast<Plane*>(extractedSym);
            addToPqueueIfNotDuplicate(applyRule(RHS_plane, terminals), pqueue);
        } 
    }
};

class Scene : public NonTerminal {
    // the printData below should be used only for the Goal NT type
    void printData() {
        pcl::PointCloud<pcl::PointXYZRGBCamSL> sceneOut;
        sceneOut=scene;
        std::ofstream graphvizFile;
        std::ofstream NTmembershipFile;
        graphvizFile.open("tree.dot", ios::out);
        NTmembershipFile.open("membership.txt", ios::out);
        stack<NonTerminal*> parseTreeNodes;
        parseTreeNodes.push(this);
        
        graphvizFile<<"digraph g{\n";
        while(!parseTreeNodes.empty())
        {
            NonTerminal *curNode=parseTreeNodes.top();
            string curName=curNode->getName();
            parseTreeNodes.pop();
            printNodeData(NTmembershipFile,curNode);
            for(size_t i=0;i<curNode->getNumChildren();i++)
            {
                Symbol * childs=curNode->getChild(i);
                
                graphvizFile<<curName<<" -> "<<childs->getName()<<" ;\n";
                if(typeid(*childs)!=typeid(Terminal))
                {
                    NonTerminal * child=dynamic_cast<NonTerminal*>(childs);
                    parseTreeNodes.push(child);
                }
            }
            
        }
        
        
        graphvizFile <<"}\n";
        graphvizFile.close();
        NTmembershipFile.close();
        
    }
    
    void printNodeData(std::ofstream & membershipFile, NonTerminal *node)
    {
        if(node==this) // will always cover full scene
            return;
        
        membershipFile<<node->getId();
        

        node->resetTerminalIterator();
        int index;
        while(node->nextTerminalIndex(index))
        {
            membershipFile<<","<<index+1;
        }
        membershipFile<<endl;
    }
    
};
class Boundary : public NonTerminal
{
    
};

class Wall : public NonTerminal
{
    
};

template<typename SceneRHS_Type1, typename SceneRHS_Type2>
class RScene : public Rule {
    template<typename TypeExtracted, typename TypeCombinee>
    void combineAndPushGivenTypes(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {

        TypeExtracted * RHS_extracted = dynamic_cast<TypeExtracted *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL) {
            //                    nt->printData(); //checked that duplicates not extracted, and exhaustive
            //  count++;
            //                    if(typeid(*nt)==typeid(TypeCombinee) )
            if (typeid (*nt) == typeid (TypeCombinee) && nt->isMutuallyExhaustive(RHS_extracted)) {
                TypeCombinee * RHS_combinee = dynamic_cast<TypeCombinee *> (nt);
                addToPqueueIfNotDuplicate(applyRule<TypeExtracted,TypeCombinee>(RHS_extracted, RHS_combinee), pqueue);
            }
            nt = finder.nextEligibleNT();
        }

        //  cout<<"nnc: "<<count<<endl;
    }

    template<typename RHS_Type1, typename RHS_Type2>
    void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {
        if(typeid(*extractedSym)==typeid(RHS_Type1))
        {
            combineAndPushGivenTypes<RHS_Type1,RHS_Type2>(extractedSym,pqueue,terminals,iterationNo);
        }
        else if(typeid(*extractedSym)==typeid(RHS_Type2))
        {
            combineAndPushGivenTypes<RHS_Type2,RHS_Type1>(extractedSym,pqueue,terminals,iterationNo);            
        }
            
        
    }

public:
    
    
    template<typename RHS_Type1, typename RHS_Type2>
    NonTerminal* applyRule(RHS_Type1 * RHS_unordered1, RHS_Type2 * RHS_unordered2)
    {
        Scene * LHS=new Scene();
        LHS->addChild(RHS_unordered1);
        LHS->addChild(RHS_unordered2);
        LHS->setAdditionalCost(0);
        LHS->computeSpannedTerminals();
        cout<<"S->fc\n";        
        cerr<<"S->fc: cost "<<LHS->getCost()<<"\n";        
//        cerr<<RHS_plane1->set_membership<<"\n";        
//        cerr<<RHS_plane2->set_membership<<"\n";        
        return LHS;
    }
    
    
    
     void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals , long iterationNo /* = 0 */)
    {
         combineAndPushGeneric<SceneRHS_Type1,SceneRHS_Type2>(extractedSym,pqueue,terminals,iterationNo);
    }

};

class Leg : public NonTerminal
{
    
};

class Table : public NonTerminal {
};

class Legs: public NonTerminal {
    vector<Leg*> legs;
public: 
    vector<Leg*> getLegs() {
        return legs;
    }
};

class TableTop: public Plane {
    /*
     * TODO: add fields to store convex hull
     */
    
    /**
     * computes the convex hull of 2D points obtained by gettign rid of Z 
     * coordinates
     */
    void compute2DConvexHull()
    {
        // getPointIndices() gives all the indices of points in this tabletop
        // scene.points[i]  gives you the ith point
        //compute their convex hull
    }
    
    /*
     * TODO: add fields to store rectangle
     */
    void computeRectangleParams()
    {
        compute2DConvexHull();
        // use the computed convex hull to compute rectangle and store it
    }
    
    
        
    /**
     * here, we are trying to find out whether the input tableTopCandidate is
     * a good candidate for being a tableTop
     * 
     * use the rectangle params and convex hull
     * @param tableTopCandidate
     */
    double computeSelfCost(Plane *tableTopCandidate)
    {
        computeRectangleParams();
        return 0;
        
    }
    
    
    /**
     * here, we are trying to find out whether the given legs fit this top 
     * use the convex hull of this tableTop(guaranteed to be already computed)
     * compute convex of legs
     * compute areas increase 
     * @param tableTopCandidate
     * @param legs
     * @return 
     */
    double computeCostOfAddingLegs(Legs *legs)
    {
        computeRectangleParams();
        return 0;
    }
};

template<>
    void DoubleRule<Boundary, Floor, Wall> :: setCost(Boundary * output, Floor * RHS_unordered1, Wall * RHS_unordered2)
    {
 //       cerr<<"correct cost"; // needs specialization
        output->setAdditionalCost(0);
    }

template<>
    void SingleRule<Wall, Plane> :: setCost(Wall* output, Plane* input)
    {
        Vector4f planeParams = input->getPlaneParams();
        output->setAdditionalCost(fabs(planeParams[2]));
    }

template<>
    void SingleRule<Leg, Plane> :: setCost(Leg* output, Plane* input)
    {
        Vector4f planeParams = input->getPlaneParams();
        output->setAdditionalCost(fabs(planeParams[2]) + fabs(input->getMaxZ() - TABLE_HEIGHT));
    }

template<>
    void SingleRule<Legs, Leg> :: setCost(Legs* output, Leg* input)
    {
        output->setAdditionalCost(0);
    }

template<>
    void SingleRule<TableTop, Plane> :: setCost(TableTop* output, Plane* input) {
        output->setAdditionalCost(0);
    }

double computeLegLegCost(Leg* leg1, Leg* leg2) {
    Plane* leg1Plane = dynamic_cast<Plane*>(leg1->children.at(0));
    Vector3d leg1PlaneNormal = leg1Plane->getPlaneNormal();
    Plane* leg2Plane = dynamic_cast<Plane*>(leg2->children.at(0));
    Vector3d leg2PlaneNormal = leg2Plane->getPlaneNormal();
    double cosine = fabs(leg1PlaneNormal.dot(leg2PlaneNormal));
    if (cosine <= .1) {
        return 0;
    } else if (.1 < cosine && cosine <= .8) {
        return 1000000;
    } else {
        pcl::PointXYZ leg1Centroid;
        leg1Plane->getCentroid(leg1Centroid);
        pcl::PointXYZ leg2Centroid;
        leg2Plane->getCentroid(leg2Centroid);
        Vector3d vectorBetweenCentroids(leg1Centroid.x - leg2Centroid.x, 
                leg1Centroid.y - leg2Centroid.y, leg1Centroid.z - leg2Centroid.z);
        double coplanarity = (vectorBetweenCentroids).dot(leg1PlaneNormal);
        if (coplanarity > .2) {
            return 1.0/coplanarity;
        } else {
            return 1000000;
        }
    }
}

template<>
    void DoubleRule<Legs, Legs, Leg> :: setCost(Legs* output, Legs* input1, Leg* input2)
    {
        vector<Leg*> legs = input1->getLegs();
        vector<Leg*>::iterator it;
        double costCount = 0;
        for (it = legs.begin(); it != legs.end(); it++) {
            costCount = costCount + computeLegLegCost(*it, input2);
        }
        output->setAdditionalCost(costCount);
    }

typedef boost::shared_ptr<Rule> RulePtr;

void appendRuleInstances(vector<RulePtr> & rules) {
    rules.push_back(RulePtr(new RPlane_Seg()));
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));
    rules.push_back(RulePtr(new RPlanePair_PlanePlane()));
    rules.push_back(RulePtr(new RFloor_Plane()));
    rules.push_back(RulePtr(new RCorner_PlanePairPlane()));
    rules.push_back(RulePtr(new RScene<Corner,Boundary>()));
    rules.push_back(RulePtr(new DoubleRule<Boundary,Floor,Wall>()));
    rules.push_back(RulePtr(new SingleRule<Wall, Plane>()));
}

void log(int iter, Symbol * sym) {
    if (sym->getNumTerminals() == 1)
        return;
    pcl::PointCloud<pcl::PointXYZRGBIndex> sceneOut;
    sceneOut.points.resize(scene.size());
    for (size_t i = 0; i < scene.size(); i++) {
        sceneOut.points[i].x = scene.points[i].x;
        sceneOut.points[i].y = scene.points[i].y;
        sceneOut.points[i].z = scene.points[i].z;
        sceneOut.points[i].rgb = scene.points[i].rgb;
    }

    std::ofstream logFile;
    NonTerminal *plane1 = dynamic_cast<NonTerminal *> (sym);
    for (size_t i = 0; i < plane1->pointIndices.size(); i++) {
        sceneOut.points[plane1->pointIndices[i]].index = 1;
    }
    pcl::io::savePCDFile("fridge_out" + boost::lexical_cast<std::string > (iter) + ".pcd", sceneOut, true);

}

void runParse(map<int, set<int> > & neighbors, int maxSegIndex) {
    vector<RulePtr> rules;
    appendRuleInstances(rules);

    //    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());

    SymbolPriorityQueue pq(maxSegIndex);

    vector<Terminal *> terminals;

    Terminal * temp;
    for (int i = 1; i <= maxSegIndex; i++) {
        temp = new Terminal(i-1); // index is segment Number -1 
        temp->setNeighbors( neighbors[i],maxSegIndex);
        terminals.push_back(temp);
        pq.pushTerminal(temp);
    }

    for(unsigned int i=0;i<scene.size();i++)
    {
        int segIndex=scene.points[i].segment;
  //      cout<<"seg "<<segIndex<<endl;
        if(segIndex>0)
            terminals.at(segIndex-1)->addPointIndex(i);
    }
    
    for(unsigned int i=0;i<terminals.size();i++)
    {
        terminals[i]->computeFeatures();
    }
    
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
    while (true) {
        min = pq.pop(alreadyExtracted);

        if(alreadyExtracted)
        {
            delete min;
            cout << "dup" << endl;
            // since there are no parent links yet(not yet declared optimal),
            // and it was not a child of anyone (not yet combined)
            // and it was repoved from PQ
            // and it was not in NTSetsExtracted,
            // deleting does not cause dangling pointers
            continue;
        }
        
        if(min==NULL)
        {
            cerr<<"parsing failed. goal is not derivable from the given rules ... fix the rules\n";
            exit(-1);
        }
        
        cout << "\n\n\niter: " << count++ << " cost:" << min->getCost() << " id: " << min->getId() <<" typ:"<<typeid(*min).name()<< endl;

        if (typeid (*min) == typeid (Scene)) {
            cout << "goal reached!!" << endl;
            min->printData();
            return;
        }
        if (typeid (*min) == typeid (Terminal) || !alreadyExtracted) {
            min->declareOptimal(terminals);
            min->printData();

            for (size_t i = 0; i < rules.size(); i++) {

                rules[i]->combineAndPush(min, pq, terminals,rulecount++); // combine with the eligible NT's to form new NTs and add them to the priority queue
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

/**
 * 
 * @param file
 * @param neighbors
 * @return : max segment index
 */
int parseNbrMap(char * file,map<int, set<int> > & neighbors) {
        std::ifstream labelFile;
    std::string line;
    labelFile.open(file);

    vector<int> nbrs;
    
    int max=0;
    if (labelFile.is_open()) {
        while (labelFile.good()) {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0)
                break;
            
            getTokens(line, nbrs);
            int segIndex=nbrs.at(0);
            set<int> temp;
            neighbors[segIndex]=temp;
            if(max<segIndex)
                max=segIndex;
            for(size_t i=1;i<nbrs.size();i++)
            {
                neighbors[segIndex].insert(nbrs.at(i));
                cout<<"adding "<<nbrs.at(i)<<" as a neighbos of "<<segIndex<<endl;
            }
        }
    } else {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }

    return max;

}

int main(int argc, char** argv) {

    
    if(argc!=3)
    {
        cerr<<"usage: "<<argv[0]<<" <pcdFile> <nbrMap> "<<endl;
    }
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
        map<int, set<int> > neighbors;
       int maxSegIndex= parseNbrMap(argv[2],neighbors);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
   runParse(neighbors,maxSegIndex);
    
    return 0;
    
}
