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
    vector<Symbol*> optimalParents;
    
//    vector<NonTerminal*> parents;
public:
        boost::dynamic_bitset<> & getNeigborTerminalBitset()
        {
            return  neighbors;
        }
        
        virtual void printData()
        {
            
        }
        
    //virtual void insertPoints(vector<int> & points)=0;
    
    virtual void unionMembersip(boost::dynamic_bitset<> & set_membership)=0;
    bool operator < (const Symbol &  rhs)
    {
        return cost <rhs.cost;
    }

    double getCost() const {
        return cost;
    }

    virtual vector<int> & getPointIndices()=0;
    
     virtual size_t getNumTerminals()=0;

    virtual void getCentroid(pcl::PointXYZ & centroid)=0;
    
    virtual bool finalize_if_not_duplicate(vector<NTSet> & planeSet,vector<Terminal*> & terminals)=0;
    
    //virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
//    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;
    
    virtual int getId()=0;

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
    vector<Terminal*> neighbors;
    vector<int> pointIndices;
public:
    
    static int totalNumTerminals;
    
    vector<Terminal*> & getNeighbors()
    {
        return neighbors;
    }
    
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
    void unionMembersip(boost::dynamic_bitset<> & set_membership)
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
        
    virtual bool finalize_if_not_duplicate(vector<NTSet> & planeSet,vector<Terminal*> & terminals){return true;}
    
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
       pointIndices.reserve(getNumTerminals());

       for(size_t i=0;i<scene.size();i++)
       {
           if(spanned_terminals.test(i))
               pointIndices.insert(pointIndices.end(),terminals.at(i)->getPointIndices().begin(),terminals.at(i)->getPointIndices().end());
       }
    }
   
   
   /** 
     * compute leaves by concatenating 
     * leaves of children */
    bool costSet;
public:
    friend class RS_PlanePlane;
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
            children[i]->unionMembersip(spanned_terminals);
        }
        numTerminals=spanned_terminals.count();
    }

    void computeNeighborTerminalSet()
    {
        //priority 1
        assert(1==2);
    }
    
    void unionMembersip(boost::dynamic_bitset<> & set_membership)
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
    bool finalize_if_not_duplicate(vector<NTSet> & allNTs,vector<Terminal*> & terminals)
    {
        //priority 2
        assert(1==2);
        // visit all children and make parent links
        assert(costSet); // cost must be set before adding it to pq
        if(checkDuplicate(allNTs))
            return false;
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
    
    bool checkDuplicate(vector<NTSet> & allNTs)
    {
        //priority 3
        assert(1==2); //need to also check if the type is same
        assert(numTerminals>0);
        NTSet & bin=allNTs[numTerminals];
        NTSet::iterator lb;
        lb=bin.lower_bound(this);
        if(lb!=bin.end() && (*lb)->spanned_terminals==spanned_terminals)
        {
         //   cout<<"duplicate:\n"<<set_membership<<"\n"<<(*lb)->set_membership<<endl;
            return true;
        }
        else
            return false;
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

int NonTerminal::id_counter=0;

/**
 * abstraction of a priority queue which can do additional book-keeping later
 * duplicate check needs to be done on all members, even those whioch have been extracted from PQ
 */
class SymbolPriorityQueue
{
    priority_queue<Symbol *,vector<Symbol *>,SymbolComparison> costSortedQueue;
    vector<NTSet> NTsets;
public:
    
    SymbolPriorityQueue(size_t numPoints) : NTsets(numPoints,NTSet())
    {
        
    }
    /** 
     * also check for duplicate and don't add if a duplicate with smaller cost
     * exists 
     */
    void push(NonTerminal * sym)
    {
        costSortedQueue.push(sym);
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
    /** eg for A->BCD return 3
     */
    virtual int get_Nof_RHS_symbols()=0;
    
    /**
     * this function will be used to check applicability of a rule
     * @param RHS_symbolIndex 0 based index of the RHS symbol
     * @return typename of this symbol
     */
    virtual void  get_typenames(vector<string> & names)=0;
    
    /**
     *  applies this rule on the given params(RHS of rule)
     * computes the cos of resulting NT
     */
    virtual NonTerminal* applyRule(vector<Symbol*> & RHS)=0;
    
    bool checkApplicabilty(vector<Symbol*> & RHS)
    {
        vector<string> typenames;
        get_typenames(typenames);
        for(int i=0;i<get_Nof_RHS_symbols();i++)
        {
            if(typeid(*(RHS.at(i))).name()!=typenames.at(i))
                return false;
        }
        return true;
    }

    /**
     * 
     * @param sym : the extracted Symbol
     * @param neighbosTerminals : it's neighbors : needs to be deleted ... use sym->getNeigborBitset()
     * @param pqueue : the priority queue
     * @param planeSet : set of all Non Terminals
     * @param terminals : set of all terminals
     */
    virtual void combineAndPush(Symbol * sym, set<int> & neighbosTerminals , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)=0;
    
    virtual void addToPqueueIfNotDuplicate(NonTerminal * newNT, vector<NTSet> & planeSet, SymbolPriorityQueue & pqueue)
    {
        newNT->computeSetMembership();
        if(!newNT->checkDuplicate(planeSet))
            pqueue.push(newNT);
        else
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
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        //lowest priority
        assert(1==2); // avg. distance of points in RHS_seg to RHS_plane
        Plane * LHS=new Plane();
        Plane * RHS_plane=dynamic_cast<Plane *>(RHS.at(0));
        Terminal * RHS_seg=dynamic_cast<Terminal *>(RHS.at(1));
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
       
     // reimplement it   LHS->setAdditionalCost(RHS_plane->costOfAddingPoint(RHS_point->getPoint()));
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)
    {
        set<int>::iterator it;
            //all terminals have cost=0, all NT's have cost>0,
            //so when a terminal is extracted, no non-terminal(plane)
            //has been extracted yet
            //so, if sym is of type Terminal, it cannot be combined with a plane
        if (typeid(*sym)==typeid(Plane ))
        {
            for(it=combineCandidates.begin();it!=combineCandidates.end();it++)
            {
                    vector<Symbol*> temp;
                    temp.push_back(sym);
                    temp.push_back(terminals.at(*it)); // must be pushed in order
                    addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    cout<<"applied rule p->pt\n";
            }
            
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

class RS_PlanePlane : public Rule
{
public:
    void addToPqueueIfNotDuplicate(NonTerminal * newNT, vector<NTSet> & planeSet, SymbolPriorityQueue & pqueue)
    {
        newNT->computeSetMembership();
   //     if(!newNT->checkDuplicate(planeSet)) // no need to check for duplicates
            pqueue.push(newNT);
    }
    
    int get_Nof_RHS_symbols()
    {
        return 2;
    }
    
    void  get_typenames(vector<string> & names)
    {
        names.push_back(typeid(Plane).name());
        names.push_back(typeid(Plane).name());
    }
    
    NonTerminal* applyRule(vector<Symbol*> & RHS)
    {
        Goal_S * LHS=new Goal_S();
        Plane * RHS_plane1=dynamic_cast<Plane *>(RHS.at(0));
        Plane * RHS_plane2=dynamic_cast<Plane *>(RHS.at(1));
        LHS->addChild(RHS_plane1);
        LHS->addChild(RHS_plane2);
        //int deficit=scene.size()-RHS_plane1->getNumPoints()-RHS_plane2->getNumPoints();
        LHS->setAdditionalCost(RHS_plane1->coplanarity(RHS_plane2)/*+exp(10*deficit)*/); // more coplanar => bad
        cout<<"applied rule S->pp\n";        
        cerr<<"applied rule S->pp: cost "<<LHS->cost<<"\n";        
        cerr<<RHS_plane1->spanned_terminals<<"\n";        
        cerr<<RHS_plane2->spanned_terminals<<"\n";        
        return LHS;
    }
    
     void combineAndPush(Symbol * sym, set<int> & combineCandidates , SymbolPriorityQueue & pqueue, vector<NTSet> & planeSet /* = 0 */, vector<Terminal*> & terminals /* = 0 */)
    {
        
        if(typeid(*sym)==typeid(Plane))
        {
            size_t targetSize=scene.size()-sym->getNumTerminals();
                NTSet & bin=planeSet[targetSize];
                if(bin.size()==0)
                    return;
                
            boost::dynamic_bitset<> neighbors(scene.size());
            set<int>::iterator nit;
            
            for(nit=combineCandidates.begin();nit!=combineCandidates.end();nit++)
            {
                neighbors.set(*nit,true);
            }
            
                NTSet::iterator it;
                    Plane * RHS_plane1=dynamic_cast<Plane *>(sym);
//                    Plane * RHS_plane2=dynamic_cast<Plane *>(*it);
                assert(!RHS_plane1->spanned_terminals.intersects(neighbors));
                for(it=bin.begin();it!=bin.end();it++)
                {
                    if(  (!RHS_plane1->intersects((*it)))   && (*it)->spanned_terminals.intersects(neighbors)  )
                    {
                        vector<Symbol*> temp;
                        temp.push_back(*it); // must be pushed in order
                        temp.push_back(sym);
                        addToPqueueIfNotDuplicate(applyRule(temp),planeSet,pqueue);
                    }
                }
                    
        }
    }    
};

typedef boost::shared_ptr<Rule>  RulePtr;
void appendRuleInstances(vector<RulePtr> & rules)
{
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));    
//    rules.push_back(RulePtr(new RPlane_SegSeg()));    
    rules.push_back(RulePtr(new RS_PlanePlane()));    
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
        if(min->finalize_if_not_duplicate(allExtractedNTs,terminals))
        {
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
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
//    pcl::PointCloud<PointT> temp;
//    subsample(scene,temp);
//    pcl::io::savePCDFile("fridge_sub500.pcd",temp,true);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
   runParse();
    return 0;
}
