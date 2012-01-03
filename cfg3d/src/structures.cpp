#ifndef STRUCTURES_CPP
#define	STRUCTURES_CPP

#include "utils.h"
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
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#define TABLE_HEIGHT .73
#define HIGH_COST 100
#include <stack>
#include "point_struct.h"
#include "color.cpp"
#include "pcl/features/feature.h"
#include <math.h>

//sac_model_plane.h

using namespace Eigen;
using namespace std;
typedef pcl::PointXYZRGBCamSL PointT;
#define MAX_SEG_INDEX 30
#include "OccupancyMap.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/random/normal_distribution.hpp>
using boost::math::normal;
string fileName;
class NonTerminal;
class Terminal;
class NTSetComparison {
public:
    bool operator() (NonTerminal * const & lhs, NonTerminal * const & rhs);
};


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

class ProbabilityDistribution {
};

class Gaussian : ProbabilityDistribution {
public: 
    double mean;
    double variance;
    double min;
    double max;
    
    normal nd;
    Gaussian() {
        mean = 1;
        variance = 0;
        normal ndTemp(mean, variance);
        nd = ndTemp;
    }
    
    Gaussian(double mean, double variance, double min, double max) {
        this->mean = mean;
        this->variance = variance;
        this->min = min;
        this->max = max;
        normal ndTemp(mean, variance);
        nd = ndTemp;
    }
};

double infinity() {
    return numeric_limits<double>::infinity();
}
Eigen::Matrix<float ,Eigen::Dynamic,  Eigen::Dynamic> segMinDistances;

//double sqr(double value) {
//    return value * value;
//}

double pointPointDistance(PointT& point1, PointT& point2) {
    return sqrt(sqr(point1.x - point2.x) + sqr(point1.y - point2.y) + sqr(point1.z - point2.z));
}

Vector3d pointPointVector(pcl::PointXYZ& point1, pcl::PointXYZ& point2) {
    return Vector3d(point1.x - point2.x, point1.y - point2.y, point1.z - point2.z);
}

double pointPointDistance(pcl::PointXYZ& point1, pcl::PointXYZ& point2) {
    return sqrt(sqr(point1.x - point2.x) + sqr(point1.y - point2.y) + sqr(point1.z - point2.z));
}

typedef set<NonTerminal*, NTSetComparison> NTSet;

pcl::PointCloud<PointT> scene;

PointT getPointFromScene(pcl::PointCloud<PointT> fromScene, int pointIndex) {
    return fromScene.points[pointIndex];
}

int NUMPointsToBeParsed;
OccupancyMap<PointT> * occlusionChecker;
//pcl::PointCloud<pcl::PointXY> scene2D;
//pcl::PCLBase<pcl::PointXY>::PointCloudConstPtr scene2DPtr;

void printPoint(pcl::PointXYZ point) {
    cout<<"("<<point.x<<","<<point.y<<","<<point.z<<") "<<endl;
}

class Symbol {
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG
     */
//    bool featuresComputed;
    double cost;
    double zSquaredSum;
    AdvancedDynamicBitset neighbors;
    vector<NonTerminal*> optimalParents;
    pcl::PointXYZ centroid;
    pcl::PointXYZ minxyz;
    pcl::PointXYZ maxxyz;
    
    long numPoints; // later, pointIndices might not be computed;
    float avgColor; 
     vector<cv::Point2f> horzConvexHull;  // The convex hull points in openCV.

//    pcl::PointCloud<pcl::PointXY> rectConvexHull;  // the convex hull in ROS.
    Eigen::Matrix3d covarianceMatrixWoMean;
    float horzArea;

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

    static float _getPolygonArea(const vector<cv::Point2f>& cv2D)
    {
        vector<cv::Point2f> contour;
        cv::approxPolyDP(cv::Mat(cv2D), contour, 0.01, true);
        return fabs(cv::contourArea(cv::Mat(contour)));
    }
    
    float getMinDistance(Symbol * other)
    {
        resetSpannedTerminalIterator();
        other->resetSpannedTerminalIterator();
        
        int index1,index2;
        float minDistance=numeric_limits<float>::infinity();
        while(nextSpannedTerminal(index1))
        {
            while(other->nextSpannedTerminal(index2))
            {
                float distance=segMinDistances(index1,index2);
                if(distance<minDistance)
                {
                    minDistance=distance;
                }
                    
            }
            
        }
        return minDistance;
    }
    
    const pcl::PointXYZ & getCentroid() const
    {
        return centroid;
    }
    
    pcl::PointXY getHorizontalCentroid()
    {
        pcl::PointXY temp;
        temp.x=centroid.x;
        temp.y=centroid.y;
        return temp;
    }

    
    
    virtual void appendFeatures(vector<float> & features)
    {
        assert(featuresComputed);
      //  features.push_back(centroid.z);
     //   features.push_back(minxyz.z);
        features.push_back(zSquaredSum/numPoints-sqr(centroid.z)); // variance along z
        features.push_back(maxxyz.z-minxyz.z);
        
        
        ColorRGB avgColorO(avgColor);
        features.push_back(avgColorO.H);
        features.push_back(avgColorO.S);
        features.push_back(avgColorO.V);
        
        // horizontal convex hull area
        features.push_back(horzArea);
        
        // move eigen vector code to here
        // linearness, planarness, scatter
        
        appendAdditionalFeatures(features);
    }
    
    virtual void appendAdditionalFeatures(vector<float> & features)
    {
    }
    
    float centroidDistance(Symbol * other)
    {
        return pcl::euclideanDistance<pcl::PointXYZ,pcl::PointXYZ>(centroid,other->centroid);
    }
    
    void pushColorDiffFeatures(vector<float> & features, Symbol * other)
    {
        ColorRGB avgColorThis(avgColor);
        ColorRGB avgColorOther(other->avgColor);
        features.push_back(avgColorThis.H - avgColorOther.H);
        features.push_back(avgColorThis.S - avgColorOther.S);
        features.push_back(avgColorThis.V - avgColorOther.V);
        
    }
    
    float centroidHorizontalDistance(Symbol * other)
    {
        return sqrt(sqr(centroid.x-other->centroid.x)+sqr(centroid.y-other->centroid.y));
    }
    
    bool featuresComputed;
    virtual void labelSubtree()=0;
    virtual void labelSubtree(int label)=0;

    bool isATerminal();

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
        assert(horzConvexHull.size()>0);
        return horzConvexHull;
    }

    vector<cv::Point2f>  cloneConvexHull() const
    {
        assert(horzConvexHull.size()>0);
        return horzConvexHull;
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

    virtual void expandIntermediates(vector<Symbol*> & nonIntermediateChildren)
    {
        nonIntermediateChildren.push_back(this);
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
    
    virtual bool declareOptimal() = 0;

    //virtual void getComplementPointSet(vector<int> & indices /* = 0 */)=0;
    //    virtual void getSetOfAncestors(set<NonTerminal*> & thisAncestors , vector<set<NonTerminal*> > & allAncestors)=0;

    pcl::PointXYZ getMaxPoint() {
        assert(featuresComputed);
        return maxxyz;
    }
    
    pcl::PointXYZ getMinPoint() {
        assert(featuresComputed);
        return minxyz;
    }
    
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
        if(featuresComputed) {
            return;
        }
        computeZSquaredSum();
        computeMinMaxXYZ();
        computeCentroidAndColorAndNumPoints();
        compute2DConvexHull();
        computeCovarianceMatrixWoMean();
        featuresComputed=true;
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
    
    virtual void resetSpannedTerminalIterator()=0;
    virtual bool nextSpannedTerminal(int & index)=0;

    float getHorzArea() const
    {
        return horzArea;
    }
    //    bool checkDuplicate(vector<set<NonTerminal*> > & ancestors)=0;
};

class SymbolComparison {
public:

    bool operator() (Symbol * & lhs, Symbol * & rhs) const {
        return lhs->getCost() > rhs->getCost();
    }
};

/**
 * also supports hashing
 */
class SymbolPriorityQueue;

    double getPointCoordinate(int pointIndex, int coordinateIndex)
    {
        return scene.points[pointIndex].data[coordinateIndex];
    }
    
float getSmallestDistance (pcl::PointCloud<PointT> &scene, boost::shared_ptr <std::vector<int> >  indices1, boost::shared_ptr <std::vector<int> >  indices2)
{
  float min_distance = FLT_MAX;
  boost::shared_ptr <std::vector<int> > small_cloud;
  boost::shared_ptr <std::vector<int> >  big_cloud;
  if (indices1->size() > indices2->size()){
    small_cloud = indices2;
    big_cloud = indices1;
  }else {
    small_cloud = indices1;
    big_cloud = indices2;
  }

  pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);//= boost::make_shared<pcl::KdTreeFLANN<PointT> > ();
  pcl::PointCloud<PointT>::Ptr scenePtr=createStaticShared<pcl::PointCloud<PointT> >(&scene);

  tree->setInputCloud (scenePtr,big_cloud);
  std::vector<int> nn_indices;
  nn_indices.resize(2);
  std::vector<float> nn_distances;
  nn_distances.resize(2);
  //float tolerance = 0.3;

  std::vector<int>::iterator it;
  for (it=small_cloud->begin(); it != small_cloud->end(); it++)
  {

        //if (!tree->radiusSearch ((*small_cloud).points[i], tolerance, nn_indices, nn_distances))
        tree->nearestKSearch (scene.points[*it], 2 , nn_indices, nn_distances);

                 if (min_distance > nn_distances[1])
                 {
                     min_distance = nn_distances[1];
                 }
  }
  return min_distance;
}


class Terminal : public Symbol 
{
protected:
    map<int,float> minDistances;
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
    bool start;
public:
    string label;
    
    void setLabel(string newLabel) {
        label = newLabel;
    }
    
    string getLabel() {
        return label;
    }
    
    virtual void resetSpannedTerminalIterator()
    {
        start=true;
    }
    virtual bool nextSpannedTerminal(int & tindex)
    {
        if(start)
        {
            tindex=index;
            return true;
        }
        else
        {
            return false;
        }
            
    }
    
    void computeMinDistanceBwNbrTerminals(vector<Terminal*> & terminals)
    {
        neighbors.iteratorReset();
        int nindex;
        float distance;
        while(neighbors.nextOnBit(nindex))
        {
            if(terminals.at(nindex)->getMinDistanceFromTerminal(index,distance)) // minDistance is symmetric, check if it was already computed
            {
                // neighbor relation is not symmetric
                neighbors[nindex]=distance;
            }
            else
            {
                neighbors[index]=getSmallestDistance(scene, getPointIndicesBoostPtr(),terminals.at(nindex)->getPointIndicesBoostPtr());
            }
        }
    }
    
    bool getMinDistanceFromTerminal(int index0Based, float & distance_ret)
    {
        map<int,float>::iterator it;
        it=minDistances.find(index0Based);
        if(it==minDistances.end())
            return false;
        else
        {
            distance_ret=it->second;
            return true;
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
    
    vector<int> & getPointIndices() {
        assert(pointIndices.size() > 0);
        return pointIndices;
    }
    
    virtual void colorScene()
    {
        
    }

    boost::shared_ptr <std::vector<int> > getPointIndicesBoostPtr() {
        assert(pointIndices.size() > 0);
        return createStaticShared<std::vector<int> >(& pointIndices);
    }
    
    void compute2DConvexHull()
    {
        if(horzConvexHull.size()>0)
            return; // convex hull wont change ... so compute only once
        
        vector<int> & indices = getPointIndices();
         vector<cv::Point2f> cvPoints;
         cvPoints.resize(indices.size());
         for(size_t i=0;i<indices.size();i++)
         {
             cvPoints.at(i).x=scene.points[indices[i]].x;
             cvPoints.at(i).y=scene.points[indices[i]].y;
         }
        cv::convexHull(cv::Mat(cvPoints), horzConvexHull);
        
    }
    
    static int numHallucinatedTerminals;
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
//        cout<<"pointIndices.size(): "<<pointIndices.size()<<endl;
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
    virtual void unionMembership(boost::dynamic_bitset<> & set_membership) {
        set_membership.set(index, true);
    }

    Terminal() : Symbol()
    {
        index = -1; /// for safety;
        cost = 0;
    }

    Terminal(int index_) : Symbol()
    {
        index = index_;
        cost = 0;
    }


    Terminal(int index_, double cost_) : Symbol()
    {
        index = index_;
        cost = cost_;
    }

    int getIndex() const {
        return index;
    }


    virtual void printData() {
        cout << "t\t:" << index << endl;
    }

    bool declareOptimal() {
        return true;
    }


    size_t getNumTerminals() {
        return 1;
    }

    virtual void labelSubtree()
    {
    }
    
    virtual void labelSubtree(int label)
    {
        //update evaluation counts
    }
    
};

class HallucinatedTerminal : public Terminal {
public: 
    HallucinatedTerminal(vector<pcl::PointXYZ> & points) : Terminal(totalNumTerminals+numHallucinatedTerminals++)
    {
            ColorRGB green(0.0,0.0,1.0);
        neighbors.resize(totalNumTerminals,false);
        int start=scene.size();
        pointIndices.resize(points.size());
        scene.points.resize(start+points.size());
        for(unsigned int i=0;i<points.size();i++)
        {
            pointIndices.at(i)=start+i;
            scene.points.at(start+i).x=points.at(i).x;
            scene.points.at(start+i).y=points.at(i).y;
            scene.points.at(start+i).z=points.at(i).z;
            scene.points.at(start+i).rgb=green.getFloatRep();
            scene.points.at(start+i).segment=index;
        }
        assert(points.size()>=3);
        featuresComputed=false;
        computeFeatures();
    }
    
    void unionMembership(boost::dynamic_bitset<> & set_membership) {
        
        //do nothing
    }

    virtual void colorScene()
    {
        float greenColor=ColorRGB(0,1,0).getFloatRep();
        for(vector<int>::iterator it=pointIndices.begin(); it!=pointIndices.end();it++)
        {
            scene.points[*it].rgb=greenColor;
        }
    }    

};

    bool Symbol :: isATerminal()
    {
        Terminal * term= dynamic_cast<Terminal *>(this);
        return (term!=NULL);
    }

Terminal * terminals;
int Terminal::totalNumTerminals = 0;
int Terminal::numHallucinatedTerminals = 0;

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
        if(horzConvexHull.size()>0)
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

        cv::convexHull(cv::Mat(unionHullUnion), horzConvexHull);
        horzArea=_getPolygonArea(horzConvexHull);
      
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
    virtual void resetSpannedTerminalIterator()
    {
        spanned_terminals.iteratorReset();
    }
    
    virtual bool nextSpannedTerminal(int & tindex)
    {
        return spanned_terminals.nextOnBit(tindex);
    }
    
        
    virtual float getMinLength()
    {
        return 0.01; 
    }
    
    virtual float getMaxLength()
    {
        return 60; 
    }

    virtual float getMinWidth()
    {
        return 0.01; 
    }
    
    virtual float getMaxWidth()
    {
        return 60; 
    }
    
    virtual void labelSubtree()
    {
        for (size_t i = 0; i < children.size(); i++)
        {
            children.at(i)->labelSubtree();
        }        
    }
    
    virtual void labelSubtree(int label)
    {
        for (size_t i = 0; i < children.size(); i++)
        {
            children.at(i)->labelSubtree(label);
        }                
    }
    
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

    virtual void printData() {
        //cout << id << "\t:" << spanned_terminals << endl;
        spanned_terminals.print1BasedIndices();
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
        cout<<absoluteCost<<endl;
        assert(absoluteCost >= (0 - .001));
        if (absoluteCost >= (0 - .001) && absoluteCost < 0) {
            absoluteCost = 0;
        }
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
    bool declareOptimal() {
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

int NonTerminal::id_counter = 0;

class NonTerminalIntermediate : public NonTerminal
{
public:
    virtual void appendFeatures(vector<float> & features)
    {
        // intermediate types dont contribute to features
    }
    
    virtual void expandIntermediates(vector<Symbol*> & nonIntermediateChildren)
    {
        vector<Symbol*>::iterator it;
        for(it=children.begin();it!=children.end();it++)
        {
            NonTerminal * child = dynamic_cast<NonTerminal*>(*it);
            assert(child!=NULL);
            child->expandIntermediates(nonIntermediateChildren);
        }
    }

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

class RecursiveNonTerminal : public NonTerminal
{
protected:
    Symbol *eldestChild;
public:
    void SetEldestChild(Symbol* eldestChild)
    {
        this->eldestChild = eldestChild;
    }

    Symbol* GetEldestChild() const
    {
        return eldestChild;
    }
};


class Plane : public NonTerminal {
protected:
    float curvature;
    Eigen::Vector3d eigenValsAscending;
    
    /**
     * planeParams[0]x+planeParams[1]x+planeParams[2]x+planeParams[3]=0
     */    
    
    Eigen::Vector4f planeParams;
public:
    bool planeParamsComputed;
    vector<int> pointIndices;
    double zSquaredSum;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Plane() : NonTerminal() {
        planeParamsComputed = false;
    }

    void addPointIndices(vector<int> newPointIndices)
    {
        vector<int>::iterator it;
        for (it = newPointIndices.begin(); it != newPointIndices.end(); it++) {
            pointIndices.push_back(*it);
        }
    }
    
    void addPointIndex(int index)
    {
        pointIndices.push_back(index);
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

    Vector3d projectionOntoNormal(Vector3d otherVector) {
        return otherVector.dot(getPlaneNormal()) * getPlaneNormal();
    }
        
    double getCentroidProximity(Symbol& other) {
        pcl::PointXYZ otherCentroid;
        other.getCentroid(otherCentroid);
        
        pcl::PointXYZ thisCentroid = centroid;
        
        Vector3d vectorBetweenCentroids = pointPointVector(otherCentroid, thisCentroid);
        
        return projectionOntoNormal(vectorBetweenCentroids).norm();
    }
    
    bool isPlanarEnough(float distance)
    {
        if(eigenValsAscending(0)>0.02*0.02*getNumPoints())
            return false;
        
        return true;
    }
    
    double planeDeviation(Plane& plane) {
        Eigen::Vector4f otherPlaneParam = plane.getPlaneParams();
        return fabs(planeParams[0] - otherPlaneParam[0]) + 
                fabs(planeParams[1] - otherPlaneParam[1]) +
                fabs(planeParams[2] - otherPlaneParam[2]);
    }
    
    void computePlaneParams() {
        pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;
        normalEstimator.computePointNormal(scene, pointIndices, planeParams, curvature);
        assert(fabs(getNorm()-1)<0.05);
        planeParamsComputed = true;
    }
    
    virtual void appendAdditionalFeatures(vector<float> & features)
    {
        //assert(featuresComputed);
        features.push_back(getLength());
        features.push_back(getWidth());
    }
    
    bool checkSize(NonTerminal * candidate)
    {
        if(getLength()<candidate->getMinLength())
            return false;
        if(getLength()>candidate->getMaxLength())
            return false;
        if(getWidth()<candidate->getMinWidth())
            return false;
        if(getWidth()>candidate->getMaxWidth())
            return false;
        
        return true;
    }
    
    
     virtual void additionalFinalize() {
         computePlaneParamsAndSetCost();
     }
    
    void computePlaneParamsAndSetCost()
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

        eigenValsAscending=eigen_values;
        double sumSquaredDistances = eigen_values(0);
        setAbsoluteCost(sumSquaredDistances);
    }
    
    double returnPlaneParams()
    {
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
        return sumSquaredDistances;
    }
    
    Eigen::Vector3d getPlaneNormal() {
        return Vector3d(planeParams[0], planeParams[1], planeParams[2]);
    }
    
    Eigen::Vector4f getPlaneParams() {
        return planeParams;
    }
    
    float getLength()
    {
        assert(planeParamsComputed);
        return 2*sqrt(eigenValsAscending(2)/(float)getNumPoints()); // rms
    }
    float getWidth()
    {
        assert(planeParamsComputed);
        return 2*sqrt(eigenValsAscending(1)/(float)getNumPoints()); // rms
    }
    
    // If this quantity is greater, then the two planes are more parallel
    double dotProduct(Plane * plane2) {
        return fabs(planeParams[0] * plane2->planeParams[0] + planeParams[1] * plane2->planeParams[1] + planeParams[2] * plane2->planeParams[2]);
    }

    bool isParallelEnough(Plane& plane, double value) {
        return dotProduct(&plane) > value;
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

bool isVerticalEnough(Plane* plane) {
    return plane->getZNormal() <= .25; // normal makes 75 degrees or more with vertical
}

/**
 * Checks if x is above value.
 * @param x
 * @param value
 * @return 
 */ 
bool isOnTop(Symbol* x, float value) {
    if (x->getMinZ() - value < -0.1) 
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * Checks if x is on top of y.
 * @param x
 * @param y
 * @return 
 */ 
bool isOnTop(Symbol* x, Symbol* y) {
    return isOnTop(x,(float)y->getMaxZ());
}

/**
 * Checks if x is above value.
 * @param x
 * @param value
 * @return 
 */ 
bool isOnTop(float value, Symbol * x) {
    if ( value -x->getMaxZ() < -0.1) 
    {
        return false;
    }
    else
    {
        return true;
    }
}

class FloorSurface : public Plane {
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
};

class KeyboardTray : public NonTerminal {
};

class Scene : public NonTerminal {
    // the printData below should be used only for the Goal NT type
    void printData() {
        pcl::PointCloud<pcl::PointXYZRGBCamSL> sceneOut;
        sceneOut=scene;
        std::ofstream graphvizFile;
        std::ofstream NTmembershipFile;
        string treeFileName=fileName+".dot";
        graphvizFile.open(treeFileName.data(), ios::out);
        string membersipFileName=fileName+"_membership.txt";
        NTmembershipFile.open(membersipFileName.data(), ios::out);
        stack<NonTerminal*> parseTreeNodes;
        parseTreeNodes.push(this);
        
        scene.width=1;
        scene.height=scene.size();
        pcl::io::savePCDFile<PointT>("hallucinated.pcd", scene,true);
        
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
                Terminal *childT= dynamic_cast<Terminal *>(childs);
                if(childT==NULL) // not of type Terminal
                {
                 //   assert(childs!=NULL);
                    NonTerminal * child=dynamic_cast<NonTerminal*>(childs);
                    assert(child!=NULL);
                    parseTreeNodes.push(child);
                }
                else
                {
                    childT->colorScene();
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

class Computer : public NonTerminal{};

class Monitor : public NonTerminal
{
    virtual float getMinLength()
    {
        return 0.15; 
    }
    
    virtual float getMaxLength()
    {
        return 0.4; 
    }

    virtual float getMinWidth()
    {
        return 0.10; 
    }
    
    virtual float getMaxWidth()
    {
        return 0.35; 
    }
    
};

class Boundary : public NonTerminal
{
    
};

class SceneGeneric : public Scene
{
    
};

class Wall : public NonTerminal
{
    virtual float getMinLength()
    {
        return 1.0; 
    }
    
    virtual float getMaxLength()
    {
        return 10; 
    }

    virtual float getMinWidth()
    {
        return 0.5; 
    }
    
    virtual float getMaxWidth()
    {
        return 10; 
    }
    
};

/**
 * 
 * @param extractedSym
 * @param pqueue
 * @param terminals
 * @param iterationNo
 */
//template<typename SceneRHS_Type>
//class RScene : public Rule {
//    
//    public:
//    template<typename RHS_Type>
//    NonTerminal* applyRule(RHS_Type* RHS_unordered)
//    {
//        Scene* LHS = new Scene();
//        LHS->addChild(RHS_unordered);
//        LHS->setAdditionalCost(0);
//        LHS->computeSpannedTerminals();
//        cout<<"S->fc\n";        
//        cerr<<"S->fc: cost "<<LHS->getCost()<<"\n";    
//        return LHS;
//    }
//    
//    template<typename TypeExtracted>
//    void combineAndPushGivenTypes(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {
//        TypeExtracted * RHS_extracted = dynamic_cast<TypeExtracted *> (extractedSym);
//        addToPqueueIfNotDuplicate(applyRule<TypeExtracted,>(RHS_extracted), pqueue);
//    }
//
//    template<typename RHS_Type>
//    void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {
//        combineAndPushGivenTypes<RHS_Type>(extractedSym,pqueue,terminals,iterationNo);
//    }
//    
//     void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals , long iterationNo /* = 0 */)
//    {
//         combineAndPushGeneric<SceneRHS_Type>(extractedSym,pqueue,terminals,iterationNo);
//    }
//
//};

class Leg : public NonTerminal
{
public:
    double computeLegLegCost(Leg* leg2) {
        Vector3d leg1PlaneNormal = dynamic_cast<Plane*>(children.at(0))->getPlaneNormal();
        Plane* leg2Plane = dynamic_cast<Plane*>(leg2->children.at(0));
        Vector3d leg2PlaneNormal = leg2Plane->getPlaneNormal();
        double cosine = fabs(leg1PlaneNormal.dot(leg2PlaneNormal));
        if (cosine <= .1) {
            return 0;
        } else if (.1 < cosine && cosine <= .8) {
            return HIGH_COST;
        } else {
            pcl::PointXYZ leg1Centroid;
            getCentroid(leg1Centroid);
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
};

class Table : public NonTerminal {
};

class Floor : public RecursiveNonTerminal {
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
    virtual float getMinLength()
    {
        return 0.6; 
    }
    
    virtual float getMaxLength()
    {
        return 2.5; 
    }

    virtual float getMinWidth()
    {
        return 0.2; 
    }
    
    virtual float getMaxWidth()
    {
        return 0.7; 
    }

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
    
//    void printData()
//    {
//        NonTerminal::printData();
//        Plane * child = dynamic_cast<Plane *> (children.at(0));
//        cout<<"length:"<<child->getLength()<<endl;
//        cout<<"width:"<<child->getWidth()<<endl;
//    }
    
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
};

class TableTopObjects : public NonTerminal{};

class TableTop : public RecursiveNonTerminal
{
public:
    TableTopSurface * getTableTopsurface()
    {
        return dynamic_cast<TableTopSurface*>(GetEldestChild());
    }
    
     double computeCostOfAddingLegs(Legs *legs)
     {
         return getTableTopsurface()->computeCostOfAddingLegs(legs);
     }
    
};

class PlaneTriplet : public NonTerminal {};


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
protected:
    vector<float> features;
    ofstream featureFile;
public:
    
    vector<Gaussian> g;
    
    void readDistribution(string filename) {
        ifstream inputStream;
        filename.append(".out");
        inputStream.open(filename.data());
        if (inputStream.is_open()) {
            string line;
            while (inputStream.good()) {
                getline(inputStream, line);
                vector<float> nbrs;
                getTokens(line, nbrs);
                Gaussian tempGaussian(nbrs.at(0), nbrs.at(1), nbrs.at(2), nbrs.at(3));
                g.push_back(tempGaussian);
            }
        }
    }
    
    /**
     * Calculate probability of feature vector x in the observed mixture of Gaussians.
     * @param x
     * @return 
     */
    float getProbability(vector<float> x) {
        double product = 1;
        int counter = 0;
        BOOST_FOREACH(Gaussian gaussian, g) {
            product = product * pdf(gaussian.nd, x.at(counter));
            counter = counter + 1;
        }
        return product;
    }
    
    const vector<float> & getFeatures() const
    {
        return features;
    }
    
//    virtual void computeFeatures(){};
    
    void writeFeaturesToFile()
    {
        vector<float>::iterator it;
        for(it=features.begin();it!=features.end();it++)
        {
            featureFile<<*it<<",";
        }
        featureFile<<endl;
    }
    
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
                addToPqueueIfNotDuplicate(applyRuleInference (RHS_extracted, RHS_combinee,terminals), pqueue);
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
                addToPqueueIfNotDuplicate(applyRuleInference(RHS_combinee, RHS_extracted,terminals), pqueue);
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
    DoubleRule(bool learning=false)
    {
        string filename=string(string("rule_")+typeid(LHS_Type).name())+"__"+string(typeid(RHS_Type1).name())+"_"+string(typeid(RHS_Type2).name());
        if(learning)
        {
            // LHS__RHS1_RHS2
                featureFile.open(filename.data(),ios::app); // append to file
        } else {
            readDistribution(filename);
        }
        
        
        
    }

//    bool setCost(LHS_Type* output, RHS_Type1* RHS1, RHS_Type2* RHS2, vector<Terminal*> & terminals)
//    {
//        assert(3 == 2); // needs specialization
//    }

    void appendPairFeatures(Symbol * rhs1, Symbol * rhs2)
    {
        features.push_back(rhs1->getMinZ()-rhs2->getMaxZ());
        features.push_back(rhs1->getMaxZ()-rhs2->getMinZ());
        features.push_back(rhs1->centroidDistance(rhs2));
        features.push_back(rhs1->centroidHorizontalDistance(rhs2));
        features.push_back(rhs1->getCentroid().z-rhs2->getCentroid().z);
        features.push_back(rhs1->getMinDistance(rhs2));
        
        rhs1->pushColorDiffFeatures(features,rhs2);
        Plane * plane1=dynamic_cast<Plane *>(rhs1);
        Plane * plane2=dynamic_cast<Plane *>(rhs2);
        if(plane1!=NULL && plane2 !=NULL)
        {
            features.push_back(plane1->dotProduct(plane2));
        }
        //get horizontal area ratio
        //area ratio on plane defined by normal
    }
    
    void appendAllFeatures(LHS_Type* output, RHS_Type1 * rhs1, RHS_Type2 * rhs2, vector<Terminal*> & terminals)
    {
        features.clear();
        
        vector<Symbol*> RHS1Expanded;
        vector<Symbol*> RHS2Expanded;
        rhs1->expandIntermediates(RHS1Expanded);
        rhs2->expandIntermediates(RHS2Expanded);
        vector<Symbol*>::iterator it1;
        vector<Symbol*>::iterator it2;
        
        for(it1=RHS1Expanded.begin();it1!=RHS1Expanded.end();it1++)
        {
            for(it2=RHS2Expanded.begin();it2!=RHS2Expanded.end();it2++)
            {
                // *it1 is of type Symbol* but the appendPairFeatures(RHS_Type1 * rhs1, RHS_Type2 * rhs2))
                appendPairFeatures(*it1, *it2);
            } 
        }
        
        output->computeFeatures();
     //   Symbol * rhs1;
      //  Symbol * rhs2;
        float rhs1Area=rhs1->getHorzArea();
        float rhs2Area=rhs2->getHorzArea();
        float lhsArea=output->getHorzArea();
        
        features.push_back(rhs1Area+rhs2Area-lhsArea);
        features.push_back(lhsArea/max(rhs1Area,rhs2Area));
        
        output->appendFeatures(features);
    }
    
    bool setCost(LHS_Type* output, RHS_Type1* RHS1, RHS_Type2* RHS2, vector<Terminal*> & terminals) {
        // Initialize features.
        output->setAdditionalCost(-log(getProbability(features)));
        return true;
    }
    
    LHS_Type* applyRuleLearning(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        assert(featureFile.is_open()); // you need to construct this rule with false for learning
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        appendAllFeatures(LHS,RHS1,RHS2,terminals);
        writeFeaturesToFile();
        return LHS;
    }
    
    LHS_Type* applyRuleInference(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        
        // below to be replaced by generic learned rules
//        appendAllFeatures(LHS, RHS1, RHS2, terminals);
        
        if(setCost(LHS,RHS1,RHS2, terminals)) {
            return LHS;
        }
        else
        {
            delete LHS;
            return NULL;
        }
    }
    
    LHS_Type* applyRuleGeneric(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS1);
        LHS->addChild(RHS2);
        LHS->computeSpannedTerminals();
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
        NonTerminal * newNT=applyRuleinference(RHS_extracted,terminals);
        
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
    SingleRule(bool learning=false)
    {
        string filename=string("rule_")+string(typeid(LHS_Type).name())+"__"+string(typeid(RHS_Type).name());
        if(learning)
        {
                featureFile.open(filename.data(),ios::app); // append to file
        }else {
            readDistribution(filename);
        }
    }
    
    void computeFeatures(RHS_Type* input)
    {
        features.clear();
        input->appendFeatures(features);
    }
    
     /**
    
     * @param output
     * @param input
     */
//    bool setCost(LHS_Type* output, RHS_Type* input, vector<Terminal*> & terminals)
//    {
//        assert(3 == 2);
//    }
    
    bool setCost(LHS_Type* output, RHS_Type* input, vector<Terminal*> & terminals) {
        output->setAdditionalCost(-log(getProbability(features)));
        return true;
    }
    
    LHS_Type* applyRuleLearning(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        assert(featureFile.is_open()); // you need to construct this rule with false for learning
        LHS_Type * LHS = applyRuleGeneric(RHS, terminals);
        
        computeFeatures(RHS);
        writeFeaturesToFile();
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        
        return LHS;
        
    }
    
    LHS_Type* applyRuleinference(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS, terminals);
        
        // to be replace by generic features
        computeFeatures(RHS);
        if(setCost(LHS, RHS,terminals))
             return LHS;
        else
        {
            delete LHS;
            return NULL;
        }
    }

    LHS_Type* applyRuleGeneric(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS);
        LHS->computeSpannedTerminals();
        return LHS;
    }
    
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        combineAndPushGeneric(extractedSym, pqueue, terminals, iterationNo);
    }
}; 

typedef boost::shared_ptr<Rule> RulePtr;

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


#endif