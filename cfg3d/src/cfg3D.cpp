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
#include<pcl/point_types.h>
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
#include <boost/dynamic_bitset.hpp>
#include <stack>
#include "point_struct.h"
#include "utils.h"

//sac_model_plane.h
 
using namespace std;
typedef pcl::PointXYZRGB PointT;
/*
 * 
 */
/**
 * does in-place set intersection ...O(n) amortized... result is in set_1
 * @param set_1
 * @param set_2
 */
template<typename T>
void setIntersection(std::set<T> & set_1, std::set<T> & set_2)
{
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end()))
    {
        if (*it1 < *it2)
        {
            set_1.erase(it1++);
        }
        else if (*it2 < *it1)
        {
            ++it2;
        }
        else
        { // *it1 == *it2
            ++it1;
            ++it2;
        }
    }
    
    set_1.erase(it1, set_1.end());
}

template<typename T>
void setDiffernce(std::set<T> & set_1, std::set<T> & set_2)
{
    typename set<T>::iterator it1 = set_1.begin();
    typename set<T>::iterator it2 = set_2.begin();
    while ((it1 != set_1.end()) && (it2 != set_2.end()))
    {
        if (*it1 < *it2)
        {
            //set_1.erase(it1++);
            it1++;
        }
        else if (*it2 < *it1)
        {
            ++it2;
        }
        else
        { // *it1 == *it2
            set_1.erase(it1++);
            ++it2;
        }
    }
    
}


class NonTerminal;
class Terminal;

class NTSetComparison
{
public:
  bool operator() (NonTerminal * const & lhs, NonTerminal * const &  rhs);
};

typedef set<NonTerminal *,NTSetComparison> NTSet;

pcl::PointCloud<PointT> scene;
class Symbol
{
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG 
     */
    double cost; 
    boost::dynamic_bitset<> neighbors;
    vector<NonTerminal*> optimalParents;
    
//    vector<NonTerminal*> parents;
public:
    void pushEligibleOptimalParents(Symbol *extractedSym, stack<NonTerminal*> & eligibleNTs)
    {
        assert(1==2); // push to the stack: the optimal parents which dont span anything already spanned by extractedSym
        for(size_t i=0;i<optimalParents.size();i++)
        {
            if(extractedSym->isSpanExclusive(optimalParents.at(i)))
            {
                eligibleNTs.push(optimalParents.at(i));
            }
        }
    }
    
    virtual bool isSpanExclusive( NonTerminal * nt )=0;
    
    bool isNeighbor(int terminalIndex)
    {
        return neighbors.test(terminalIndex);
    }
        boost::dynamic_bitset<> & getNeigborTerminalBitset()
        {
            return  neighbors;
        }
        
        virtual void printData()
        {
            
        }
        
    //virtual void insertPoints(vector<int> & points)=0;
    
    virtual void unionMembership(boost::dynamic_bitset<> & set_membership)=0;
    bool operator < (const Symbol& rhs)
    {
        return cost <rhs.cost;
    }

    double getCost() const {
        return cost;
    }

    virtual vector<int> & getPointIndices()=0;
    
    virtual size_t getNumTerminals()=0;

    virtual void getCentroid(pcl::PointXYZ & centroid)=0;
    
    virtual bool declareOptimal(vector<NTSet> & planeSet,vector<Terminal*> & terminals)=0;
    
    //virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
//    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;
    
    virtual int getId()=0;

    /**
     * Adds node to the list of optimal parents of a node for bookkeeping.
     * @param node: node to add
     */
    void appendOptimalParents(NonTerminal* node) {
        optimalParents.push_back(node);
    }
//    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)=0;
};

class SymbolComparison
{
public:
  bool operator() (Symbol * & lhs, Symbol * & rhs) const
  {
      return lhs->getCost()>rhs->getCost();
  }
};


//typedef priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> SymbolPriorityQueue;
/**
 * also supports hashing
 */
class SymbolPriorityQueue;

class Terminal : public Symbol
{
protected:
    /** segment index
     */
    size_t index;
    vector<int> pointIndices;
public:
    
    static int totalNumTerminals;
    
    boost::dynamic_bitset<> & getNeighbors()
    {
        return neighbors;
    }
    
    bool isSpanExclusive( NonTerminal * nt );
    
    /**
     * will be only used  to compute cost of rules . CFG parsing functions should
     *  not use it
     * @return 
     */
    vector<int> & getPointIndices()
    {
        return pointIndices;
    }
    
    int getId()
    {
        return 0;
    }
    
    /**
     * add this terminal to the set
     * @param set_membership
     */
    void unionMembership(boost::dynamic_bitset<> & set_membership)
    {
        set_membership.set(index,true);
    }
    
    Terminal()
    {
        index=-1; /// for safety;
        cost=0;
    }

    Terminal(int index_)
    {
        index=index_;
        cost=0;
    }

    Terminal(int index_,double cost_)
    {
        index=index_;
        cost=cost_;
    }
    
    int getIndex() const
    {
        return index;
    }
    

    
    void getCentroid(pcl::PointXYZ & centroid)
    {
        assert(3==2); // need to be implemented
    }

        virtual void printData()
        {
            cout<<"t\t:"<<index<<endl;
        }
        
    virtual bool declareOptimal(vector<NTSet> & planeSet,vector<Terminal*> & terminals){return true;}
    
    //bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)    {
//        return false; // each terminal can be derived in only 1 way
        /* contrast it with a set of terminals which can be derived in multiple way
         * (1 U 2) U 3  or  1 U (2 U 3 )
         */
   // }
    
    size_t getNumTerminals()
    {
        return 1;
    }
    
 /*   void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)
    {
        thisAncestors=allAncestors[index];
    }
  */
};
Terminal * terminals;
int Terminal::totalNumTerminals=0;

class NonTerminal : public Symbol
{
protected:
    size_t numTerminals;
//    vector<bool> setOfPoints;
    boost::dynamic_bitset<> spanned_terminals;
    vector<Symbol*> children;
    pcl::PointXYZ centroid;
    int id;
    /**
     * version number
     */
    long lastIteration;
    static int id_counter;
    /** indices into the pointcloud
     */
    //will be populated only when extracted as min
    void computeCentroid()
    {
        assert(pointIndices.size()>0);
        PointT point;
        centroid.x=0;
        centroid.y=0;
        centroid.z=0;
        for (size_t i = 0; i < pointIndices.size(); i++)
        {
            point = scene.points[pointIndices[i]];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
            centroid.x /=pointIndices.size();
            centroid.y /=pointIndices.size();
            centroid.z /=pointIndices.size();                
    }
  
    
   void computePointIndices(vector<Terminal*> & terminals)
    {
       pointIndices.clear();
      // pointIndices.reserve(getNumTerminals());

       for(int i=0;i<Terminal::totalNumTerminals;i++)
       {
           if(spanned_terminals.test(i))
               pointIndices.insert(pointIndices.end(),terminals.at(i)->getPointIndices().begin(),terminals.at(i)->getPointIndices().end());
       }
    }
   
    bool isSpanExclusive( NonTerminal * nt )
    {        
        return spanned_terminals.intersects(nt->spanned_terminals);
    }

   /** 
     * compute leaves by concatenating 
     * leaves of children */
    bool costSet;
public:
    friend class RS_PlanePlane;
    friend class Terminal;
    vector<int> pointIndices; // can be replaced with any sufficient statistic
    
    vector<int> & getPointIndices()
    {
        assert(pointIndices.size()>0);
        return pointIndices;
    }
    
    bool intersects(NonTerminal * other)
    {
        return spanned_terminals.intersects(other->spanned_terminals);
    }
        int getId()
    {
        return id;
    }

    NonTerminal()
    {
        costSet=false;
        id=id_counter++;
        cout<<"new NT: "<<id<<endl;
        numTerminals=0;
        lastIteration=0;
    }

    friend class NTSetComparison;
    
        virtual void printData()
        {
            cout<<id<<"\t:"<<spanned_terminals<<endl;
        }
        
    size_t getNumTerminals()
    {
        assert(numTerminals>0);
        return numTerminals;
    }

    void setAdditionalCost(double additionalCost)
    {
        assert(additionalCost>=0);
        cost=0;
        for(size_t i=0;i<children.size();i++)
            cost+=children[i]->getCost();
        cost+=additionalCost;
        costSet=true;
        cout<< "ac: "<<additionalCost<<" tc: "<<cost<<endl;
    }
    
    void addChild(Symbol * child)
    {
        assert(!costSet);
        children.push_back(child);
    }
    
    void computeSetMembership()
    {
        spanned_terminals.resize(Terminal::totalNumTerminals,false);
        for(size_t i=0;i<children.size();i++)
        {
            children[i]->unionMembership(spanned_terminals);
        }
        numTerminals=spanned_terminals.count();
    }

    /***
     * Iterates through all neighbors of NonTerminal node's children,
     * computing the union of all children neighbors minus the span of the
     * NonTerminal node.
     */
    void computeNeighborTerminalSet()
    {
        boost::dynamic_bitset<> temp(spanned_terminals.size());
        vector<Symbol*>::iterator it;
        for (it = children.begin(); it != children.end(); ++it) {
            temp |= (*it)->getNeigborTerminalBitset();
        }
        temp -= spanned_terminals;
        neighbors = temp;
    }
    
    void unionMembership(boost::dynamic_bitset<> & set_membership)
    {
//        assert(pointIndices.size()>0);
        set_membership|=this->spanned_terminals;
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
    bool declareOptimal(vector<NTSet> & allNTs,vector<Terminal*> & terminals)
    {
        vector<Symbol*>::iterator it;
        for (it = children.begin(); it != children.end(); ++it) {
            (*it)->appendOptimalParents(this);
        }

        assert(costSet); // cost must be set before adding it to pq
        computePointIndices(terminals);
   //     std::pair< set<NonTerminal*>::iterator , bool > resIns;
        
/*        for(size_t i=0;i<pointIndices.size();i++)
        {
            resIns=ancestors[pointIndices[i]].insert(this); // too many duplicate inserts
            assert(resIns.second);
        }   */
        
        allNTs[numTerminals].insert(this);// indexed by size 
        // S wont ever be finalized , ... so it will only contain planes
                                
        // computeCentroid();
        additionalFinalize();
        return true;
    }
    
    void getCentroid(pcl::PointXYZ & centroid1)
    {
        centroid1=centroid;
    }
    
    /**
     * subclasses can override this method to do compute and store additional statistic
     */
    virtual void additionalFinalize()
    {
        
    }
    
    bool checkDuplicate(NonTerminal * sym)
    {
        if(typeid(*sym)!=typeid(*this))
            return false;
        if(sym->spanned_terminals!=spanned_terminals)
            return false;
        return true;
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
  bool NTSetComparison::operator() (NonTerminal * const & lhs, NonTerminal * const &  rhs)
  {
      //start with MSBs
      for(int i=lhs->spanned_terminals.num_blocks()-1;i>=0;i--)
      {
          if(lhs->spanned_terminals.m_bits[i] > rhs->spanned_terminals.m_bits[i])
              return true;
          else if(lhs->spanned_terminals.m_bits[i]  <  rhs->spanned_terminals.m_bits[i])
              return false;
          // else look the lower significant block
                      
      }
      return false; // actually they are equal
  }
  
    bool Terminal::isSpanExclusive( NonTerminal * nt )
    {
        return !(nt->spanned_terminals.test(index));
    }

int NonTerminal::id_counter=0;

/**
 * when an NT is extracted, this class will help in finding possible candidates 
 * for combination. it uses a stack now, because, it might be useful
 * later to ignore all the parents of an (bad)NT
 */
class FindNTsToCombineWith
{
    stack<NonTerminal*> elibibleNTs;
    long iterationNo;
    Symbol *extractedSym;
    FindNTsToCombineWith(Symbol * sym,vector<Terminal*> & allTerminals, long iterationNo_)
    {
        extractedSym=sym;
        iterationNo=iterationNo_;
       for(int i=0;i<Terminal::totalNumTerminals;i++)
       {
           if(sym->isNeighbor(i))
           {
               allTerminals.at(i)->pushEligibleOptimalParents(sym,elibibleNTs);
           }
       }
        
    }
    
    NonTerminal * nextEligibleNT()
    {
        if(elibibleNTs.empty())
            return NULL;
        
        NonTerminal *ret=elibibleNTs.top(); // only unvisited and eligible items were pushed to stack
        elibibleNTs.pop();
        
        //push it's parents which are eligible
        // to add a possibility of ignoring all it's ancestors, postpone
        //next step to nest call by temporarily storing return value
        ret->pushEligibleOptimalParents(extractedSym,elibibleNTs);
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
class SymbolPriorityQueue
{
    priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> costSortedQueue;
    vector<NTSet> NTsetsExtracted;
    vector<NTSet> NTsetsInPQ;
public:
    
    SymbolPriorityQueue(size_t numTerminals) : NTsetsExtracted(numTerminals,NTSet()), NTsetsInPQ(numTerminals,NTSet())
    {
        
    }
    
    bool CheckIfBetterDuplicateExists(NonTerminal * sym)
    {
        int numTerminals=sym->getNumTerminals();
        assert(numTerminals>0);
        NTSet & bin=NTsetsExtracted[numTerminals];
        NTSet::iterator lb;
        lb=bin.lower_bound(sym);
        if(lb!=bin.end() && (*lb)->checkDuplicate(sym))
        {
         //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            return true;
        }
        
        NTSet & bin1=NTsetsExtracted[numTerminals];
        lb=bin1.lower_bound(sym);
        if(lb!=bin1.end() && (*lb)->checkDuplicate(sym))
        {
         //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            if(sym->getCost()>=(*lb)->getCost())
                return true;
        }
        return false;
    }
    
    /**
     * extracted before => was better or same cost
     * 
     */
    bool CheckIfBetterDuplicateWasExtracted(NonTerminal * sym)
    {
        int numTerminals=sym->getNumTerminals();
        assert(numTerminals>0);
        NTSet & bin=NTsetsExtracted[numTerminals];
        NTSet::iterator lb;
        lb=bin.lower_bound(sym);
        if(lb!=bin.end() && (*lb)->checkDuplicate(sym))
        {
         //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            return true;
        }
        
        return false;
    }

    /** 
     * also check for duplicate and don't add if a duplicate with smaller cost
     * exists 
     */
    bool pushIfNoBetterDuplicateExists(NonTerminal * sym)
    {
        if(CheckIfBetterDuplicateExists(sym))
            return false;
        
        costSortedQueue.push(sym);
        NTsetsInPQ[sym->getNumTerminals()].insert(sym);
        return true;
    }

    /**
     * no need to check for duplicate
     * @param sym
     */
    void pushTerminal(Terminal * sym)
    {
        costSortedQueue.push(sym);        
    }
    
    Symbol * pop()
    {
        Symbol * top=costSortedQueue.top();
        costSortedQueue.pop();
        if(typeid(*top)!=typeid(Terminal))
        {
                NonTerminal * nt=dynamic_cast<NonTerminal *>(top);
                NTsetsExtracted[top->getNumTerminals()].insert(nt); // set => no duplicates
                NTsetsInPQ[top->getNumTerminals()].erase(nt);
        }
        
        return top;
    }
    
    size_t size()
    {
        return costSortedQueue.size();
    }
    
};

class Rule
{    
public:
    /**
     * @param sym : the extracted Symbol
     * @param neighbosTerminals : it's neighbors : needs to be deleted ... use sym->getNeigborBitset()
     * @param pqueue : the priority queue
     * @param planeSet : set of all Non Terminals
     * @param terminals : set of all terminals
     */
    virtual void combineAndPush(Symbol * sym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */)=0;
    
    virtual void addToPqueueIfNotDuplicate(NonTerminal * newNT, vector<NTSet> & planeSet, SymbolPriorityQueue & pqueue)
    {
        newNT->computeSetMembership(); // required for duplicate check
            if(!pqueue.pushIfNoBetterDuplicateExists(newNT))
                    delete newNT;
    }
};

double sqr(double d)
{
    return d*d;
}


class Plane : public NonTerminal
{
protected:
//    friend class RPlane_PlanePoint;
    float curvature;
    bool planeParamsComputed;
    Eigen::Vector4f planeParams;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Plane():NonTerminal()
    {
        planeParamsComputed=false;
    }
    
    double getNorm()
    {
         return (planeParams[0]*planeParams[0]  +  planeParams[1]*planeParams[1]  +  planeParams[2]*planeParams[2]);
    }
    
    void computePlaneParams()
    {
        pcl::NormalEstimation<PointT,pcl::Normal> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        double norm=getNorm();
        planeParams/=norm;
        planeParamsComputed=true;
    }
    
    
    double coplanarity(Plane * plane2)
    {
        return  fabs(planeParams[0]*plane2->planeParams[0]  +  planeParams[1]*plane2->planeParams[1]  +  planeParams[2]*plane2->planeParams[2]) ;
    }
    
    double costOfAddingPoint(PointT p)
    {
        if(pointIndices.size()==2)
        {
            /*return it's min distance to either points
             */
            double d1=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[0]]);
            double d2=pcl::euclideanDistance<PointT,PointT>(p,scene.points[pointIndices[1]]);
            if(d1>d2)
                return d1;
            else
                return d2;
        }
        else if(pointIndices.size()>=3)
        {
            assert(planeParamsComputed);
//            return exp(100*pcl::pointToPlaneDistance<PointT>(p,planeParams))-1;
            return getNumTerminals()*(exp(pcl::pointToPlaneDistance<PointT>(p,planeParams))-1);
        }
        else
            assert(4==2);
    }
    
    void additionalFinalize()
    {
        if(pointIndices.size()>=3)
            computePlaneParams();
    }
};

class RPlane_PlaneSeg : public Rule
{
public:
    int get_Nof_RHS_symbols()
    {
        return 2;
    }
    
    void  get_typenames(vector<string> & names)
    {
        names.push_back(typeid(Plane ).name());
        names.push_back(typeid(Terminal ).name());
    }
    
    NonTerminal* applyRule(Plane * RHS_plane, Terminal *RHS_seg)
    {
        //lowest priority
        assert(1==2); // avg. distance of points in RHS_seg to RHS_plane
        Plane * LHS=new Plane();
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
       
     // reimplement it   LHS->setAdditionalCost(RHS_plane->costOfAddingPoint(RHS_point->getPoint()));
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */)
    {
        set<int>::iterator it;
            //all terminals have cost=0, all NT's have cost>0,
            //so when a terminal is extracted, no non-terminal(plane)
            //has been extracted yet
            //so, if sym is of type Terminal, it cannot be combined with a plane
        if (typeid(*sym)==typeid(Plane ))
        {
/*           for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                    vector<Symbol*> temp;
                    temp.push_back(sym);
                    temp.push_back(terminals.at(*it)); // must be pushed in order
                    addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    cout<<"applied rule p->pt\n";
            }
  */           
        }
    }    
};


class Goal_S : public NonTerminal
{
protected:
public:
    Goal_S():NonTerminal()
    {
        
    }
    
    void printData()
    {
        pcl::PointCloud<pcl::PointXYZRGBIndex> sceneOut;
        sceneOut.points.resize(scene.size());
        for(size_t i=0;i<scene.size();i++)
        {
            sceneOut.points[i].x=scene.points[i].x;
            sceneOut.points[i].y=scene.points[i].y;
            sceneOut.points[i].z=scene.points[i].z;
            sceneOut.points[i].rgb=scene.points[i].rgb;
        }

        std::ofstream logFile;
        logFile.open("log.txt",ios::out);
        NonTerminal *plane1=dynamic_cast<NonTerminal *>(children[0]);
        NonTerminal *plane2=dynamic_cast<NonTerminal *>(children[1]);
        for(size_t i=0;i<plane1->pointIndices.size();i++)
        {
            logFile<<","<<plane1->pointIndices[i];
            sceneOut.points[plane1->pointIndices[i]].index=1;
        }
        logFile<<endl;
        for(size_t i=0;i<plane2->pointIndices.size();i++)
        {
            logFile<<","<<plane2->pointIndices[i];
            sceneOut.points[plane2->pointIndices[i]].index=2;
        }
        logFile<<endl;
        logFile.close();
    pcl::io::savePCDFile("fridge_out.pcd",sceneOut,true);
    }
};


typedef boost::shared_ptr<Rule>  RulePtr;
void appendRuleInstances(vector<RulePtr> & rules)
{
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));    
}

void log(int iter, Symbol *  sym)
{
    if(sym->getNumTerminals()==1)
        return;
        pcl::PointCloud<pcl::PointXYZRGBIndex> sceneOut;
        sceneOut.points.resize(scene.size());
        for(size_t i=0;i<scene.size();i++)
        {
            sceneOut.points[i].x=scene.points[i].x;
            sceneOut.points[i].y=scene.points[i].y;
            sceneOut.points[i].z=scene.points[i].z;
            sceneOut.points[i].rgb=scene.points[i].rgb;
        }

        std::ofstream logFile;
        NonTerminal *plane1=dynamic_cast<NonTerminal *>(sym);
        for(size_t i=0;i<plane1->pointIndices.size();i++)
        {
            sceneOut.points[plane1->pointIndices[i]].index=1;
        }
    pcl::io::savePCDFile("fridge_out"+boost::lexical_cast<std::string>(iter)+".pcd",sceneOut,true);
    
}
void runParse()
{
    vector<RulePtr> rules;
    appendRuleInstances(rules);
    int numPoints=scene.size();
    SymbolPriorityQueue pq(numPoints);
    
//    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());
    
    vector<NTSet> allExtractedNTs(numPoints,NTSet());
    
    vector<Terminal *> terminals;
   
    Terminal * temp;
    for(int i=0;i<numPoints;i++)
    {
        temp=new Terminal(i);
        terminals.push_back(temp);
        pq.pushTerminal(temp);
    }
    Terminal::totalNumTerminals=terminals.size();
    
    Symbol *min;
    int count=0;
    while(true)
    {
        min=pq.pop();
        
        cout<<"\n\n\niter: "<<count++<<" cost was:"<<min->getCost()<<" id was "<<min->getId()<<endl;
        
        if(typeid(*min)==typeid(Goal_S))
        {
            cout<<"goal reached!!"<<endl;
            min->printData();
            return;
        }
        if(typeid(*min)==typeid(Terminal) || !pq.CheckIfBetterDuplicateWasExtracted(dynamic_cast<NonTerminal *>(min)))
        {
            min->declareOptimal(allExtractedNTs,terminals);
        min->printData();
            
            for(size_t i=0;i<rules.size();i++)
            {
                
                assert(1==2); // combineAndPush needs to be reimplemented
                //rules[i]->combineAndPush(min, combineCandidates, pq, allExtractedNTs, terminals); // combine with the eligible NT's to form new NTs and add them to the priority queue
                //an eligible NT should not span any terminal already in min
                //an eligible NT should contain atleast 1 terminal in combneCandidates
            }
            
        }
        else
        {
            delete min; 
            cout<<"duplicate detected"<<endl;
            // since there are no parent links, and it was not a child of anyone
            // deleting does not cause dangling pointers
            
        }
        
        
        
        //pq.pop();
        
    }
}
void subsample(pcl::PointCloud<PointT> & inp,pcl::PointCloud<PointT> & out)
{
    out.points.clear();
    out.header=inp.header;
    for(size_t i=0;i<inp.size();i++)
    {
        if(rand() % 5==1)
        {
            out.points.push_back(inp.points[i]);
        }
    }
}

int main(int argc, char** argv) 
{
    boost::dynamic_bitset<> x(5);
    boost::dynamic_bitset<> y(5);
    x[1]=1;
    x[3]=1;
    y[0]=1;
    y[2]=1;
    y[4]=1;
    cout<<x<<endl;
    cout<<y<<endl;
    x|=y;
    cout<<"new x: "<<x<<endl;
//    pcl::io::loadPCDFile<PointT>(argv[1], scene);
//    pcl::PointCloud<PointT> temp;
//    subsample(scene,temp);
//    pcl::io::savePCDFile("fridge_sub500.pcd",temp,true);
//    cout<<"scene has "<<scene.size()<<" points"<<endl;
//   runParse();
//    return 0;
}
