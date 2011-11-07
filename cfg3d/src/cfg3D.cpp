/*
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include <opencv/cv.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <typeinfo>
#include "point_types.h"
#include "point_struct.h"
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>
#include <stdlib.h>
#include <time.h>
#include <boost//lexical_cast.hpp>
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#define TABLE_HEIGHT .70
#define HIGH_COST 100
#include <stack>
#include "point_struct.h"
#include "utils.h"
#include "color.cpp"
#include "pcl/features/feature.h"

//sac_model_plane.h

using namespace Eigen;
using namespace std;
typedef pcl::PointXYZRGBCamSL PointT;
#define MAX_SEG_INDEX 30
#include "OccupancyMap.h"
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
OccupancyMap<PointT> * occlusionChecker;
//pcl::PointCloud<pcl::PointXY> scene2D;
//pcl::PCLBase<pcl::PointXY>::PointCloudConstPtr scene2DPtr;

class Symbol {
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG
     */
    bool featuresComputed;
    double cost;
    double zSquaredSum;
    AdvancedDynamicBitset neighbors;
    vector<NonTerminal*> optimalParents;
    pcl::PointXYZ centroid;
    pcl::PointXYZ minxyz;
    pcl::PointXYZ maxxyz;
    
    long numPoints; // later, pointIndices might not be computed;
    float avgColor; 
     vector<cv::Point2f> rectConvexHull;  // The convex hull points in openCV.

//    pcl::PointCloud<pcl::PointXY> rectConvexHull;  // the convex hull in ROS.
    Eigen::Matrix3d covarianceMatrixWoMean;

    //    vector<NonTerminal*> parents;
    virtual void computeCovarianceMatrixWoMean()=0;

    double getSigmaCoordinate(int coordinateIndex)
    {
        return centroid.data[coordinateIndex]*numPoints;
    }

    void computeMeanTA(const pcl::PointXYZ & centr, Eigen::Matrix3d & ans)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans(i, j) = centr.data[i] * getSigmaCoordinate(j);

    }
        
    void computeMeanTMean(const pcl::PointXYZ & centr, Eigen::Matrix3d & ans)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans(i, j) = centr.data[i] * centr.data[j] * numPoints;

    }
public:

    void computeCovarianceMat( Eigen::Matrix3d & ans)
    {
        computeMeanCovOtherMean(centroid,ans);
    }
    
    void computeMeanCovAddition(const pcl::PointXYZ & centr, Eigen::Matrix3d & ans)
    {
        assert(featuresComputed);
        Eigen::Matrix3d  temp;
        computeMeanTA(centr, temp);
        ans=-temp;
        ans-=temp.transpose();
        
        computeMeanTMean(centr, temp);
        ans+=temp;
    }
    
    void computeSelfMeanCovAddition(Eigen::Matrix3d & ans)
    {
        assert(featuresComputed);
        Eigen::Matrix3d  temp;
        computeMeanTA(centroid, temp);
        ans=-2*temp;
        
        computeMeanTMean(centroid, temp);
        ans+=temp;
    }
    
    void computeMeanCovOtherMean(const pcl::PointXYZ & othercentr, Eigen::Matrix3d & ans)
    {
        computeSelfMeanCovAddition(ans);
        ans+=covarianceMatrixWoMean;
    }
    
    const vector<cv::Point2f> & getConvexHull() const
    {
        assert(rectConvexHull.size()>0);
        return rectConvexHull;
    }

    vector<cv::Point2f>  cloneConvexHull() const
    {
        assert(rectConvexHull.size()>0);
        return rectConvexHull;
    }
    
    virtual string getName()=0;

    Symbol()
    {
        featuresComputed=false;
    }
    void pushEligibleNonDuplicateOptimalParents(Symbol *extractedSym, stack<NonTerminal*> & eligibleNTs, long iterationNo);

    virtual bool isSpanExclusive(NonTerminal * nt) = 0;

    bool isNeighbor(int terminalIndex) {
        return neighbors.test(terminalIndex);
    }

    boost::dynamic_bitset<> & getNeigborTerminalBitset() {
        return neighbors;
    }
    
    virtual void computeCentroidAndColorAndNumPoints()=0 ;
    
    virtual void printData() =0;
    
    virtual void computeMinMaxXYZ()=0;
    
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


    virtual size_t getNumTerminals() = 0;
    
     virtual void compute2DConvexHull()=0;


    void getCentroid(pcl::PointXYZ & centroid_)
    {
        assert(featuresComputed);
        centroid_=centroid;
    }

    ColorRGB getAvgColor()
    {
        return ColorRGB(avgColor);
    }
    
    virtual bool declareOptimal( vector<Terminal*> & terminals) = 0;

    //virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
    //    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;

    double getMaxX() {
        assert(featuresComputed);
        return maxxyz.x;
    }
    
    double getMinX() {
        assert(featuresComputed);
        return minxyz.x;
    }
    
    double getMaxY() {
        assert(featuresComputed);
        return maxxyz.y;
    }
    
    double getMinY() {
        assert(featuresComputed);
        return minxyz.y;
    }
    
    double getMaxZ() {
        assert(featuresComputed);
        return maxxyz.z;
    }
    
    double getMinZ() {
        assert(featuresComputed);
        return minxyz.z;
    }

    double getMaxCoord(int coordIndex) {
        assert(featuresComputed);
        return maxxyz.data[coordIndex];
    }
    
    double getMinCoord(int coordIndex) {
        assert(featuresComputed);
        return minxyz.data[coordIndex];
    }
    
    virtual void computeZSquaredSum() = 0;
    
    double getZSquaredSum() {
        assert(featuresComputed);
        return zSquaredSum;
    }
        
    double computeZMinusCSquared(double c) 
    {
        assert(featuresComputed);
        return zSquaredSum - 2 * centroid.z * numPoints * c + numPoints * c*c;
    }
    
    void computeFeatures()
    {
        if(featuresComputed)
            return;
        featuresComputed=true;
        computeZSquaredSum();
        computeMinMaxXYZ();
        computeCentroidAndColorAndNumPoints();
        compute2DConvexHull();
        computeCovarianceMatrixWoMean();
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
        assert(featuresComputed);
        return numPoints;
    }

    const Eigen::Matrix3d & getCovarianceMatrixWoMean() const
    {
        return covarianceMatrixWoMean;
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

    double getPointCoordinate(int pointIndex, int coordinateIndex)
    {
        return scene.points[pointIndex].data[coordinateIndex];
    }

    class Terminal : public Symbol 
{
protected:
    vector<int> pointIndices;
    /** segment index
     */
    size_t index;

    void computeCovarianceMatrixWoMean()
    {
        covarianceMatrixWoMean = Eigen::Matrix3d::Zero();
        for (vector<int>::iterator it = pointIndices.begin(); it != pointIndices.end(); it++)
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    covarianceMatrixWoMean(i, j) += (getPointCoordinate(*it, i) * getPointCoordinate(*it, j));

    }
    
public:
    vector<int> & getPointIndices() {
        assert(pointIndices.size() > 0);
        return pointIndices;
    }

    boost::shared_ptr <const std::vector<int> > getPointIndicesBoostPtr() {
        assert(pointIndices.size() > 0);
        return createStaticShared<std::vector<int> >(& pointIndices);
    }
    
    void compute2DConvexHull()
    {
        if(rectConvexHull.size()>0)
            return; // convex hull wont change ... so compute only once
        
        vector<int> & indices = getPointIndices();
         vector<cv::Point2f> cvPoints;
         cvPoints.resize(indices.size());
         for(size_t i=0;i<indices.size();i++)
         {
             cvPoints.at(i).x=scene.points[indices[i]].x;
             cvPoints.at(i).y=scene.points[indices[i]].y;
         }
        cv::convexHull(cv::Mat(cvPoints), rectConvexHull);
    }
    
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

    
    void computeZSquaredSum() 
    {
        double costSum = 0;
        for (vector<int>::iterator it = pointIndices.begin(); it != pointIndices.end(); it++) {
            costSum = costSum + (scene.points[*it].z  * scene.points[*it].z);
        }
        zSquaredSum = costSum;
    }
    
    void computeMinMaxXYZ() {
        vector<Symbol*>::iterator it;
        for (int i = 0; i < 3; i++)
        {
             maxxyz.data[i] = -infinity();
             minxyz.data[i] = infinity();
        }
        
        double itVal;
        for (int i = 0; i < 3; i++)
        {
            for (vector<int>::iterator it = pointIndices.begin(); it != pointIndices.end(); it++) 
            {
                itVal = scene.points.at(*it).data[i];
                if (itVal > maxxyz.data[i])
                {
                    maxxyz.data[i] = itVal;
                }
                
                if (itVal < minxyz.data[i])
                {
                    minxyz.data[i] = itVal;
                }
            }
        }
    }
    
    
    void computeCentroidAndColorAndNumPoints() {
        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
        ColorRGB avg(0,0,0);
        for (size_t i = 0; i < pointIndices.size(); i++) {
            PointT & point = scene.points[pointIndices[i]];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
            avg+=ColorRGB(point.rgb);
        }
        numPoints=pointIndices.size();
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;
        avgColor/=numPoints;
        avgColor=avg.getFloatRep();
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

    Terminal() : Symbol()
    {
        index = -1; /// for safety;
        cost = 0;
    }

    Terminal(int index_) {
        index = index_;
        cost = 0;
    }

    Terminal(vector<pcl::PointXYZ> & points) {
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


    size_t getNumTerminals() {
        return 1;
    }

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
    
    /**
     * convex hull of this = convex hull of union of points in convex hull of 
     * children
     */
    void compute2DConvexHull()
    {
        if(rectConvexHull.size()>0)
            return; // convex hull wont change ... so compute only once
        
        
        vector<cv::Point2f> unionHullUnion;
        
        Symbol *child;
        child=(children.at(0));
        unionHullUnion=child->cloneConvexHull();
        
        for(unsigned int i=1;i<children.size();i++)
        {
            child=children.at(i);
            const vector<cv::Point2f> & childHull = child->getConvexHull();
            unionHullUnion.insert(unionHullUnion.end(), childHull.begin(), childHull.end() );
        }

        cv::convexHull(cv::Mat(unionHullUnion), rectConvexHull);
      
    }

    void computeCentroidAndColorAndNumPoints() {
        pcl::PointXYZ childCent;
        ColorRGB avg(0,0,0);
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
            avg+=(children.at(i)->getAvgColor()*numPointsInChild);
        }
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;
        avg/=numPoints;
        avgColor=avg.getFloatRep();
    }
    
    void computeCovarianceMatrixWoMean()
    {
        covarianceMatrixWoMean=Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < children.size(); i++)
        {
            covarianceMatrixWoMean+=children.at(i)->getCovarianceMatrixWoMean();
        }
        
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
    friend class Terminal;
    
    /**
     * computes the convex hull of 2D points obtained by gettign rid of Z 
     * coordinates
     */
    
    
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
        const char * name=typeid(*this).name();
        int count=0;
        while(isdigit(*name)&&count<5)
        {
            name++;
            count++;
        }
        return string(name)+boost::lexical_cast<std::string>(children.size())+string("i")+boost::lexical_cast<std::string>(id);
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
    
    
    bool intersects(NonTerminal * other) {
        return spanned_terminals.intersects(other->spanned_terminals);
    }

    int getId() {
        return id;
    }

    NonTerminal() : Symbol()
    {
        costSet = false;
        id = id_counter++;
        cout << "nNT: " << id << endl;
        numTerminals = 0;
        lastIteration = 0;
        duplicate=false;
    }
    
    friend class NTSetComparison;

//    void sumChildrenCovOtherMean(const pcl::PointXYZ & othercentr, Eigen::Matrix3d & ans)
//    {
//        Eigen::Matrix3d  temp;
//        children.at(0)->computeMeanCovAddition(othercentr,ans);
//        
//        for(unsigned int i=1;i<children.size();i++)
//        {
//             children.at(0)->computeMeanCovAddition(othercentr,temp);
//             ans+=temp;
//        }
//        
//    }

    void printData() {
        cout << id << "\t:" << spanned_terminals << endl;
        for (uint i = 0; i < spanned_terminals.size(); i++) {
            if(spanned_terminals.test(i)) {
                cout<<i+1<<", ";
            }
        }
        cout<<endl;
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
        vector<Symbol*>::iterator it;
        for (it = children.begin(); it != children.end(); it++) {
            assert(absoluteCost >= (*it)->getCost());
        }
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
        for (it = children.begin(); it != children.end(); it++) {
            neighbors |= (*it)->getNeigborTerminalBitset();
        }
        neighbors -= spanned_terminals;
    }

    void computeMinMaxXYZ()
    {
        vector<Symbol*>::iterator it;
        for (int i = 0; i < 3; i++)
        {
             maxxyz.data[i] = -infinity();
             minxyz.data[i] = infinity();
        }
        
        double itMax,itMin;
        for (int i = 0; i < 3; i++)
        {
            for (it = children.begin(); it != children.end(); it++)
            {
                itMax = (*it)->getMaxCoord(i);
                itMin = (*it)->getMinCoord(i);
                if (itMax > maxxyz.data[i])
                {
                    maxxyz.data[i] = itMax;
                }
                
                if (itMin < minxyz.data[i])
                {
                    minxyz.data[i] = itMin;
                }
            }
        }
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

        for (it = children.begin(); it != children.end(); it++) {
            (*it)->appendOptimalParents(this);
        }

        computeNeighborTerminalSet();
        assert(costSet); // cost must be set before adding it to pq
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
        if (sym->getCost() >= HIGH_COST) {
            return false;
        }
        
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
        assert(top!=NULL);
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
        if(newNT==NULL)
            return;
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
                addToPqueueIfNotDuplicate(applyRule (RHS_extracted, RHS_combinee,terminals), pqueue);
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
                addToPqueueIfNotDuplicate(applyRule(RHS_combinee, RHS_extracted,terminals), pqueue);
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
    bool setCost(LHS_Type* output, RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        assert(3 == 2); // needs specialization
    }
        
    NonTerminal* applyRule(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS1);
        LHS->addChild(RHS2);
        LHS->computeSpannedTerminals();
        if(setCost(LHS,RHS1,RHS2, terminals)) {
            return LHS;
        }
        else
        {
            delete LHS;
            return NULL;
        }
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
        NonTerminal * newNT=applyRule(RHS_extracted,terminals);
        
        if(newNT!=NULL)
        {
                addToPqueueIfNotDuplicate(newNT, pqueue);
        }
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
    bool setCost(LHS_Type* output, RHS_Type* input, vector<Terminal*> & terminals)
    {
        assert(3 == 2);
    }
        
    NonTerminal* applyRule(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS);
        LHS->computeSpannedTerminals();
        if(setCost(LHS, RHS,terminals))
             return LHS;
        else
        {
            delete LHS;
            return NULL;
        }
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

    bool isHorizontalEnough() {
        return getZNormal() >= .88;
    }

    double getNorm() {
        return (planeParams[0] * planeParams[0] + planeParams[1] * planeParams[1] + planeParams[2] * planeParams[2]);
    }
    
    double getZNormal() {
        return fabs(planeParams[2]);
    }

    void computePlaneParams()
    {
        if (planeParamsComputed)
            return;

        computeFeatures();
        Eigen::Vector4f xyz_centroid_;

        for (int i = 0; i < 3; i++)
            xyz_centroid_(i) = centroid.data[i];
        xyz_centroid_(3) = 1;

        Eigen::Matrix3d covMat; //
        computeCovarianceMat(covMat);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm(covMat);
        EIGEN_ALIGN16 Eigen::Vector3d eigen_values = ei_symm.eigenvalues();
        EIGEN_ALIGN16 Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors();

        planeParams[0] = eigen_vectors(0, 0);
        planeParams[1] = eigen_vectors(1, 0);
        planeParams[2] = eigen_vectors(2, 0);
        planeParams[3] = 0;

        // Hessian form (D = nc . p_plane (centroid here) + p)
        planeParams[3] = -1 * planeParams.dot(xyz_centroid_);
        planeParamsComputed = true;

        double sumSquaredDistances = eigen_values(0);
        setAbsoluteCost(sumSquaredDistances);
    }
    
    Eigen::Vector3d getPlaneNormal() {
        return Vector3d(planeParams[0], planeParams[1], planeParams[2]);
    }
    
    Eigen::Vector4f getPlaneParams() {
        return planeParams;
    }
    
    // If this quantity is greater, then the two planes are more parallel
    double dotProduct(Plane * plane2) {
        return fabs(planeParams[0] * plane2->planeParams[0] + planeParams[1] * plane2->planeParams[1] + planeParams[2] * plane2->planeParams[2]);
    }

    double costOfAddingPoint(PointT & p) {
            assert(planeParamsComputed);
            double ret=pcl::pointToPlaneDistance<PointT > (p, planeParams);
            return ret;
    }

    bool isCloseEnough(PointT& p) {
        if (costOfAddingPoint(p) > .3) {
            return false;
        } else {
            return true;
        }
    }
    
    bool isAllCloseEnough(Terminal* terminal) {
        vector<int>& termPointIndices = terminal->getPointIndices();
        for(vector<int>::iterator it = termPointIndices.begin(); it != termPointIndices.end(); it++) {
            if (!isCloseEnough(scene.points[*it])) {
                return false;
            }
        }
        return true;
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
     
     double getPlane1Z() {
         return dynamic_cast<Plane*>(children.at(0))->getZNormal();
     }
     
     double getPlane2Z() {
         return dynamic_cast<Plane*>(children.at(1))->getZNormal();
     }
     
     double getSumOfZs() {
         double z1 = dynamic_cast<Plane*>(children.at(0))->getZNormal();
         double z2 = dynamic_cast<Plane*>(children.at(1))->getZNormal();
         return z1 + z2;
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

class Corner : public NonTerminal {
    // TODO: Do we need anything in here? This class feels cold and empty inside.
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

class Computer : public PlanePair{};

class Monitor : public NonTerminal{};

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
    
    void setLegs(vector<Leg*> newLegs) {
        legs = newLegs;
    }
    
    void appendLeg(Leg* leg) {
        legs.push_back(leg);
    }
};

class TableTopSurface: public Plane {

public:

    void additionalFinalize()
    {
        if(corners.size()==0)
                initialize();
    }
    
    void initialize()
    {

        computeRectangleParams();
    }

    vector<cv::Point2f> & getCorners()
    {
        return corners;
    }

    float getAngle()
    {
        return angle;
    }

    float getWidth()
    {
        return width;
    }

    float getHeight()
    {
        return height;
    }

    pcl::PointXY getCenter()
    {
        return center;
    }

    float getRectArea()
    {
        return width * height;
    }

    float getConvexHullArea()
    {
        return convexHullArea;
    }


    void computeRectangleParams()
    {
        corners.resize(4);
        cv::RotatedRect rect = cv::minAreaRect(cv::Mat(getConvexHull()));
        angle = rect.angle;
        //cv::Point2f cvCorners[4];
         rect.points(corners.data());
//        corners(corners.data());
//        for (int i = 0; i < 4; ++i) {
//            cv::Point2f pt;
//            pt.x = cvCorners[i].x;
//            pt.y = cvCorners[i].y;
//            corners.push_back(pt);
//        }

        center.x = rect.center.x;
        center.y = rect.center.y;

        height = rect.size.height;
        width = rect.size.width;

        convexHullArea = _getPolygonArea(getConvexHull());
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
        assert(1==2); //what do these numbers mean?
        
        // Define height and width cost
        float heightCost = 0;
        float widthCost = 0;

        if (height < 0.5)
        {
            heightCost = 0.5 - height;
        }
        else if (height > 1.5)
        {
            heightCost = height - 1.5;
        }

        if (width < 1)
        {
            widthCost = 1 - width;
        }
        else if (width > 3)
        {
            widthCost = width - 3;
        }

        // To get the error rate of the fitting rectangle, get the ratio
        // of the area of the fitting rectangle to the plane's convex hull.
        
        assert(false) ; // is the rectangle inside convex hull?
        float fittingCost = getRectArea() / convexHullArea;

        float cost = heightCost + widthCost + fittingCost;
        return cost;
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
        // Get the ratio of the area of the new convex hull to the plane's
        // convex hull. We define the new convex hull as the original plane's
        // points plus the legs' points.
      
        // Compute the convex hull for legs first
        vector<cv::Point2f> combined2dCv;
        vector<cv::Point2f> concatenatedConvexHull;
//        assert(false);  // add scene 2D to global
        // TODO: type wrong
        // computeConvexHull.setIndices(legs->getPointIndices());
        concatenatedConvexHull=legs->cloneConvexHull();
        concatenatedConvexHull.insert(concatenatedConvexHull.end(),corners.begin(),corners.end());
        

        // pcl::ConvexHull<pcl::PointXY> computeConvexHull;
        cv::convexHull(cv::Mat(concatenatedConvexHull), combined2dCv);

        float combinedArea = _getPolygonArea(combined2dCv);

        float rectArea=getRectArea();
        assert(rectArea>0);
        float cost = 200*(combinedArea / rectArea -1);
      //  cerr<<"areas "<<combinedArea<<","<<cost<<","<<width<<","<<height<<endl;
        if(cost>-0.001)
            cost=0; // minor numerical errors
        assert(cost>=0);
        return cost;
    }

private:

    vector<cv::Point2f> corners;
    float angle;
    pcl::PointXY center;
    float width;
    float height;
    float convexHullArea;


    /*
     * Convert ros 2D point cloud to CV Mat.
    */
//    void _rosToOpenCv(const pcl::PointCloud<pcl::PointXY>& pc2D, vector<cv::Point2f> cv2D)
//    {
//        cv2D.clear();
//        pcl::PointCloud<pcl::PointXY>::const_iterator pc;
//        for (pc = pc2D.begin(); pc != pc2D.end(); ++pc)
//        {
//            cv::Point2f curr(pc->x, pc->y);
//            cv2D.push_back(curr);
//        }
//    }

    /* Get Area */
    float _getPolygonArea(const vector<cv::Point2f>& cv2D)
    {
        vector<cv::Point2f> contour;
        cv::approxPolyDP(cv::Mat(cv2D), contour, 0.001, true);
        return fabs(cv::contourArea(cv::Mat(contour)));
    }
};

class TableTopObjects : public NonTerminal{};

class TableTop : public NonTerminal
{
public:
    TableTopSurface * getTableTopsurface()
    {
        return dynamic_cast<TableTopSurface*>(children.at(0));
    }
    
     double computeCostOfAddingLegs(Legs *legs)
     {
         return getTableTopsurface()->computeCostOfAddingLegs(legs);
     }
    
};

//template<>
//    bool DoubleRule<Plane, Plane, Terminal> :: setCost(Plane * output, Plane * input1, Terminal * input2, vector<Terminal*> & terminals)
//    {
//        output->computePointIndices(terminals);  
//        output->computePlaneParams();
//        output->setCost();
//        cout<<"COST OF PLANE->PLANESEG: "<<output->getCost()<<endl;
//        return true;
//    }

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
        if (!RHS_plane->isAllCloseEnough(RHS_seg)) {
            return NULL;
        }
        
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParams();
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

/// Templated Rules Marker TRM
//template<>
//    bool DoubleRule<PlanePair, Plane, Plane> :: setCost(PlanePair * output, Plane * input1, Plane * input2, vector<Terminal*> & terminals)
//    {
//        double parallelity = input1->dotProduct(input2);
//        if (parallelity < .2) {
//            output->setAdditionalCost(parallelity);
//            return true;
//        }
//        else {
//            return false;
//        }
//    }

template<>
    bool DoubleRule<Corner, PlanePair, Plane> :: setCost(Corner * output, PlanePair * input1, Plane * input2, vector<Terminal*> & terminals)
    {
        Vector3d planePairCrossProduct = input1->getCrossProduct();
        Vector3d planeNormal(input2->getPlaneNormal());
        output->setAdditionalCost(1-fabs(planePairCrossProduct.dot(planeNormal)));
        return true;
    }

template<>
    bool DoubleRule<Boundary, Floor, Wall> :: setCost(Boundary * output, Floor * input1, Wall * input2, vector<Terminal*> & terminals)
    {
 //       cerr<<"correct cost"; // needs specialization
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool SingleRule<Plane, Terminal> :: setCost(Plane* output, Terminal* input, vector<Terminal*> & terminals)
    {
        output->computePlaneParams();
        return true;
    }

template<>
    bool SingleRule<Wall, Plane> :: setCost(Wall* output, Plane* input, vector<Terminal*> & terminals)
    {
        Vector4f planeParams = input->getPlaneParams();
        double additionalCost=fabs(planeParams[2]);
        if(additionalCost>0.2)
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost);
            return true;
        }           
    }

template<>
    bool SingleRule<Leg, Plane> :: setCost(Leg* output, Plane* input, vector<Terminal*> & terminals)
    {
    //cerr<<"called"<<fabs(input->getMaxZ() - TABLE_HEIGHT)<<","<<(input->getMaxZ())<<endl;
        Vector4f planeParams = input->getPlaneParams();
        double normalZ=fabs(planeParams[2]);
        double maxZDiff=fabs(input->getMaxZ() - TABLE_HEIGHT);

        if(normalZ>.25 || maxZDiff >0.30)
//        if( maxZDiff >0.2)
            return false;
        else 
        {
            output->setAdditionalCost(normalZ + maxZDiff);
//            output->setAdditionalCost( maxZDiff);
            return true;
        }
    }

template<>
    bool SingleRule<Legs, Leg> :: setCost(Legs* output, Leg* input, vector<Terminal*> & terminals)
    {
        output->appendLeg(input);
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool SingleRule<TableTopSurface, Plane> :: setCost(TableTopSurface* output, Plane* input, vector<Terminal*> & terminals) {
        double additionalCost=input->computeZMinusCSquared(TABLE_HEIGHT);
        if(additionalCost>(0.3*0.3)*input->getNumPoints() || ! input->isHorizontalEnough())
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost);
            return true;
        }                   
    }

template<>
    bool SingleRule<Floor, Plane> :: setCost(Floor * output, Plane* input, vector<Terminal*> & terminals) {
        double additionalCost=input->getZSquaredSum();
        if(additionalCost>(0.2*0.2)*input->getNumPoints() || ! input->isHorizontalEnough() )
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost);
            return true;
        }                   
}

void solveLinearEquationPair(const Vector2f& v1, const Vector2f& v2, const Vector2f& b, Vector2f& solution) {
    Matrix2f A;
    float v10 = v1[0];
    float v11 = v1[1];
    float v20 = v2[0];
    float v21 = v2[1];
    A << v10,v11, v20,v21;
    solution = A.colPivHouseholderQr().solve(b);
}

Vector2f getDirection(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
    Vector2f v1(p1.x, p1.y);
    Vector2f v2(p2.x, p2.y);
    return v1 - v2;
}

pcl::PointXYZ getFarthestInDirection(Plane& plane, const Vector2f direction) {
    float farthestX;
    float farthestY;
    
    if (direction[0] < 0) {
        farthestX = plane.getMinX();
    } else {
        farthestX = plane.getMaxX();
    }
    
    if (direction[1] < 0) {
        farthestY = plane.getMinY();
    } else {
        farthestY = plane.getMaxY();
    }
    
    return pcl::PointXYZ(farthestX, farthestY, 0);
}

pcl::PointXYZ getPlanePlaneOcclusionPoint(Plane& plane1, Plane& plane2, pcl::PointXYZ& c1, pcl::PointXYZ& c2, Vector2f& d1, Vector2f& d2) {
    Vector4f p1Params = plane1.getPlaneParams();
    Vector4f p2Params = plane2.getPlaneParams();
    Vector2f intersectionPoint;
    Vector2f v1(p1Params[0],p1Params[1]);
    Vector2f v2(p2Params[0],p2Params[1]);
    Vector2f v4(p1Params[3],p2Params[3]);
    solveLinearEquationPair(v1, v2, v4, intersectionPoint);
        
    pcl::PointXYZ centroid1;
    plane1.getCentroid(centroid1);
    pcl::PointXYZ centroid2;
    plane2.getCentroid(centroid2);
    
    pcl::PointXYZ i(intersectionPoint[0], intersectionPoint[1], 0);
    
    // Where d1 and d2 are the directions away from the intersection point of the two planes.
    d2 = getDirection(centroid1, i);
    d1 = getDirection(centroid2, i);
    
    
    c1 = getFarthestInDirection(plane1, d1);
    c2 = getFarthestInDirection(plane2, d2);
    
    Vector2f b(c2.x - c1.x, c2.y - c1.y);
    Vector2f row1(d1[0], -d2[0]);
    Vector2f row2(d1[1], -d2[1]);
    Vector2f r;
    solveLinearEquationPair(row1, row2, b, r);
    
    float x_p = c2.x + r[0] * d2[0];
    float y_p = c2.y + r[1] * d2[1];
    return pcl::PointXYZ(x_p, y_p, 0);
}

bool isOccluded(pcl::PointXYZ& point) {
    assert(3==2);
    return false;
}

vector<pcl::PointXYZ*> getPointsToSample(pcl::PointXYZ& c1, pcl::PointXYZ& occlusionPoint, Plane& plane, float sampleFactor) {
    vector<pcl::PointXYZ*> samplePoints;
    float xStep = fabs(c1.x - occlusionPoint.x)/sampleFactor;
    float yStep = fabs(c1.y - occlusionPoint.y)/sampleFactor;
    float zStep = fabs(plane.getMaxZ() - plane.getMinZ())/sampleFactor;
    float currentX = min(c1.x, occlusionPoint.x);
    float currentY = min(c1.y, occlusionPoint.y);
    float samplesTaken = 0;
    while(samplesTaken < sampleFactor) {
        for (int k = plane.getMinZ(); k < plane.getMaxZ(); k+=zStep) {
            pcl::PointXYZ samplePoint(currentX, currentY, k);
            samplePoints.push_back(&samplePoint);
        }
        currentX = currentX + xStep;
        currentY = currentY + yStep;
        samplesTaken = samplesTaken + 1;
    }
    return samplePoints;
}

bool canHallucinatePlane(Plane& plane1, Plane& plane2) {
    float sampleFactor = 5;
    float occlusionThreshold = .9;
    pcl::PointXYZ c1;
    pcl::PointXYZ c2;
    Vector2f d1;
    Vector2f d2;
    pcl::PointXYZ occlusionPoint = getPlanePlaneOcclusionPoint(plane1, plane2, c1, c2, d1, d2);
    vector<pcl::PointXYZ*> samplePoints = getPointsToSample(c1, occlusionPoint, plane1, sampleFactor);
    float numOccludedPoints = 0;
    vector<pcl::PointXYZ*>::iterator it;
    for (it = samplePoints.begin(); it != samplePoints.end(); it++) {
        if (isOccluded(**it)) {
            numOccludedPoints = numOccludedPoints + 1;
        }
    }
    return numOccludedPoints / (sampleFactor * sampleFactor) > occlusionThreshold;
}

// Checks if x is on top of y
bool isOnTop(Symbol* x, Symbol* y) {
    if (x->getMinZ() - y->getMaxZ() < -0.1) 
    {
       // cerr<<"ontop violated";
        return false;
    }
    else
    {
        return true;
    }
}

bool isVerticalEnough(Plane* plane) {
    return plane->getZNormal() <= .25;
}

bool isZCloseEnough(double value, double height) {
    return fabs(value - height) <= .3;
}

template<>
    bool DoubleRule<Table, TableTop, Legs> :: setCost(Table* output, TableTop* input1, Legs* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input1, input2)) 
        {
            output->setAdditionalCost(input1->computeCostOfAddingLegs(input2));
            return true;
        } 
        else 
        {
            return false;
        }
    }

template<>
    bool DoubleRule<Table, TableTopSurface, Legs> :: setCost(Table* output, TableTopSurface* input1, Legs* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input1, input2)) 
        {
            output->setAdditionalCost(input1->computeCostOfAddingLegs(input2));
          //  cerr<<"table id"<< output->getId() <<"created w/ cost"<<output->getCost()<<endl;
            return true;
        } 
        else 
        {
          //  cerr<<"table rejected w/ cost"<<output->getCost()<<endl;
            return false;
        }
    }

bool isCloseEnoughToTableHeight(Plane* input) {
    return isZCloseEnough(input->getMinZ(), TABLE_HEIGHT);
}

bool isCloseEnoughToCompMonTop(Plane* input) {
    return isZCloseEnough(input->getMaxZ(), 1.1);
}

// Assumes all computers are above table_height
template<>
    bool DoubleRule<Computer, Plane, Plane> :: setCost(Computer* output, Plane* input1, Plane* input2, vector<Terminal*> & terminals) {

    if (!isVerticalEnough(input1) || !isVerticalEnough(input2)) {
        return false;
    } else {
        double minZOfBothPlanes = min(input1->getMinZ(), input2->getMinZ());
        
        if (!isCloseEnoughToTableHeight(input1) || !isCloseEnoughToTableHeight(input2) ||
            !isCloseEnoughToCompMonTop(input1) || !isCloseEnoughToCompMonTop(input2)) {
            return false;
        } else {
            double distanceFromTable = fabs(minZOfBothPlanes - TABLE_HEIGHT);
            double zNormal1 = input1->getZNormal();
            double zNormal2 = input2->getZNormal();
            output->setAdditionalCost(distanceFromTable + zNormal1 + zNormal2);
            
            //TODO: maybe add costs for maxZ?
            return true;
        }
    }
}

// Assumes all monitors are above table_height
template<>
    bool SingleRule<Monitor, Plane> :: setCost(Monitor* output, Plane* input, vector<Terminal*> & terminals) {
        if (!isVerticalEnough(input)) {
            return false;
        } else {
            if (!isCloseEnoughToTableHeight(input) || 
                !isCloseEnoughToCompMonTop(input)) {
                return false;
            } else {
                double distanceFromTable = fabs(input->getMinZ() - TABLE_HEIGHT);
                output->setAdditionalCost(distanceFromTable + input->getZNormal());
                return true;
            }
        }
    }

template<>
    bool DoubleRule<TableTopObjects, Computer, Monitor> :: setCost(TableTopObjects* output, Computer* input1, Monitor* input2, vector<Terminal*> & terminals) {
        output->setAdditionalCost(0);
        return true;
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
        return HIGH_COST;
    } else {
        pcl::PointXYZ leg1Centroid;
        leg1Plane->getCentroid(leg1Centroid);
        pcl::PointXYZ leg2Centroid;
        leg2Plane->getCentroid(leg2Centroid);
        Vector3d vectorBetweenCentroids(leg1Centroid.x - leg2Centroid.x, 
                leg1Centroid.y - leg2Centroid.y, leg1Centroid.z - leg2Centroid.z);
        double coplanarity = fabs((vectorBetweenCentroids).dot(leg1PlaneNormal));
        if (coplanarity > .2) {
            return 0.01/coplanarity;
        } else {
            return HIGH_COST;
        }
    }
}

template<>
    bool DoubleRule<Legs, Legs, Leg> :: setCost(Legs* output, Legs* input1, Leg* input2, vector<Terminal*> & terminals)
    {
        output->setLegs(input1->getLegs());
        output->appendLeg(input2);
        vector<Leg*> legs = input1->getLegs();
        vector<Leg*>::iterator it;
        double costCount = 0;
        for (it = legs.begin(); it != legs.end(); it++) {
            costCount = costCount + computeLegLegCost(*it, input2);
        }
        output->setAdditionalCost(costCount);
        return true;
    }

template<>
    bool DoubleRule<TableTop, TableTopSurface, TableTopObjects> :: setCost(TableTop* output, TableTopSurface* input1, TableTopObjects* input2, vector<Terminal*> & terminals) {

        if (isOnTop(input2, input1)) 
        {
            output->setAdditionalCost(0);
            return true;
        } 
        else 
        {
            return false;
        }
    }

typedef boost::shared_ptr<Rule> RulePtr;

void appendRuleInstances(vector<RulePtr> & rules) {
    

//    rules.push_back(RulePtr(new DoubleRule<PlanePair, Plane, Plane>()));
    //rules.push_back(RulePtr(new DoubleRule<Corner, PlanePair, Plane>()));
    
    // planes
    rules.push_back(RulePtr(new SingleRule<Plane, Terminal>()));
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));
    
    // boundary, wall, floor
    rules.push_back(RulePtr(new SingleRule<Floor, Plane>()));
    rules.push_back(RulePtr(new SingleRule<Wall, Plane>()));
    rules.push_back(RulePtr(new DoubleRule<Boundary,Floor,Wall>()));

    // table
    rules.push_back(RulePtr(new SingleRule<TableTopSurface, Plane>()));
    rules.push_back(RulePtr(new SingleRule<Leg, Plane>()));
    rules.push_back(RulePtr(new SingleRule<Legs,Leg>()));
    rules.push_back(RulePtr(new DoubleRule<Legs,Legs,Leg>()));
    
    
    // computer
    rules.push_back(RulePtr(new DoubleRule<Computer, Plane, Plane>()));
    
    // monitor
    rules.push_back(RulePtr(new SingleRule<Monitor, Plane>()));  
    
    // whole scene
    rules.push_back(RulePtr(new RScene<Table,Boundary>()));
    
    // table
    rules.push_back(RulePtr(new DoubleRule<TableTopObjects, Computer, Monitor>()));
    rules.push_back(RulePtr(new SingleRule<TableTopSurface, Plane>()));
    rules.push_back(RulePtr(new DoubleRule<TableTop, TableTopSurface, TableTopObjects>()));
    rules.push_back(RulePtr(new DoubleRule<Table,TableTop,Legs>()));
    rules.push_back(RulePtr(new DoubleRule<Table,TableTopSurface,Legs>()));
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
        if(rand()%10 != 1)
            continue;
        int segIndex=scene.points[i].segment;
  //      cout<<"seg "<<segIndex<<endl;
        if(segIndex>0 && segIndex<=maxSegIndex)
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
            cerr<<"parsing failed. goal is not derivable from the given rules ... fix the rules or PQ insertion threshold ... or rules' thershold\n";
            exit(-1);
        }
        
        cout << "\n\n\niter: " << count++ << " cost:" << min->getCost() <<" typ:"<<min->getName()<< endl;

        if (typeid (*min) == typeid (Scene)) {
            cout << "goal reached!!" << endl;
            min->printData();
            return;
        }
        if (typeid (*min) == typeid (Terminal) || !alreadyExtracted) {
            min->declareOptimal(terminals);
            min->printData();
 //           cout<<"mz"<<min->getMaxZ()<<endl;
            
            for (size_t i = 0; i < rules.size(); i++) {
                rules.at(i)->combineAndPush(min, pq, terminals,rulecount++); // combine with the eligible NT's to form new NTs and add them to the priority queue
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
            
            if(segIndex>MAX_SEG_INDEX)
                continue;
            
            set<int> temp;
            neighbors[segIndex]=temp;
            if(max<segIndex)
                max=segIndex;
            for(size_t i=1;i<nbrs.size();i++)
            {

                if(nbrs.at(i)>MAX_SEG_INDEX)
                        continue;
                
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
    void convertToXY(const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<pcl::PointXY> & cloudxy)
    {
        cloudxy.points.resize(cloud.size());
        cloudxy.sensor_origin_=cloud.sensor_origin_;
        for (size_t i = 0; i < cloud.size(); i++)
        {
            cloudxy.points[i].x = cloud.points[i].x;
            cloudxy.points[i].y = cloud.points[i].y;
        }
    }
    
int main(int argc, char** argv) {
    if(argc!=3)
    {
        cerr<<"usage: "<<argv[0]<<" <pcdFile> <nbrMap> "<<endl;
    }
    pcl::io::loadPCDFile<PointT>(argv[1], scene);

    occlusionChecker = new OccupancyMap<PointT>(scene);

    //convertToXY(scene,scene2D);
  //  scene2DPtr=createStaticShared<pcl::PointCloud<pcl::PointXY> >(&scene2D);
    map<int, set<int> > neighbors;
    int maxSegIndex= parseNbrMap(argv[2],neighbors);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
    runParse(neighbors,maxSegIndex);
    
    return 0;
    
}
