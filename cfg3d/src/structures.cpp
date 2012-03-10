#ifndef STRUCTURES_CPP
#define	STRUCTURES_CPP

//options
#define MAX_SEG_INDEX 100000
#define DIVIDE_BY_SIGMA
//#define COST_THRESHOLD 2000
//#define OCCLUSION_SHOW_HEATMAP
//#define PROPABILITY_RANGE_CHECK
#define DISABLE_HALLUCINATION

#include <boost/type_traits.hpp>
#include <boost/utility.hpp>

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
#include <stack> 
#include "utils.h"
#include "point_struct.h"
#include "color.cpp"
#include "pcl/features/feature.h"
#include <math.h>
//sac_model_plane.h

using namespace Eigen;
using namespace std;
typedef pcl::PointXYZRGBCamSL PointT;

#include "OccupancyMap.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/random/normal_distribution.hpp>
using boost::math::normal;
string fileName;
class NonTerminal;
class Terminal;
class HallucinatedTerminal;
class NTSetComparison {
public:
    bool operator() (NonTerminal * const & lhs, NonTerminal * const & rhs);
};
string rulePath;

class Params
{
public:
    const static double missPenalty=                900000000000000000000000000.0;
    const static double onTopPairDivide=30;
    const static double onTopPairDefaultOnModelMissing=500000000000.0;
    const static int timeLimit=500;
    const static double doubleRuleDivide=90;
    const static double objectCost=4;
    const static double maxFloorHeight=0.05;
    const static double floorOcclusionPenalty=200000.0;
    const static double costPruningThresh=          30000000000000000000.0;
    const static double costPruningThreshNonComplex=3000000000000000000.0;
    const static double additionalCostThreshold=300;
    const static double featScale=1000;
    
//    const static double missPenalty=9000;
//    const static double onTopPairDivide=5;
//    const static double onTopPairDefaultOnModelMissing=50;
//    const static int timeLimit=2000;
//    const static double doubleRuleDivide=10;
//    const static double objectCost=60;
//    const static double maxFloorHeight=0.05;
//    const static double floorOcclusionPenalty=20;
//    const static double costPruningThresh=150;
//    const static double costPruningThreshNonComplex=45;
//    const static double featScale=1000;        
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
double infinity() {
    return numeric_limits<double>::infinity();
}

class MultiVariateProbabilityDistribution {
protected:
    int numFeats;
public:
    virtual double minusLogProb(VectorXd & x)=0;

    double minusLogProb(vector<float> & x) {
        VectorXd feats(x.size());
        for(int i=0;i<(int)x.size();i++)
        {
            feats(i)=x.at(i);
        }
        return minusLogProb(feats);
    }

    int getNumFeats() const
    {
        return numFeats;
    }
//    virtual VectorXd getMean()=0;
//    virtual VectorXd getVar()=0;
//    virtual double getMaxCutoff()=0;
//    virtual double getMinCutoff()=0;
    
//    bool isInRange(double x)
//    {
//        return (getMinCutoff()<=x && x<=getMaxCutoff());
//    }
};

class MultiVarGaussian : public MultiVariateProbabilityDistribution
{
    VectorXd mean;
    VectorXd minv;
    VectorXd maxv;
    MatrixXd sigmaInv;
    double additiveConstant;
public:
    double minusLogProb(VectorXd & x)
    {
        assert(x.rows()==numFeats);
//        MatrixXd value= ((x-mean).transpose()*sigmaInv*(x-mean)/2.0 );
        MatrixXd value= ((x-mean).transpose()*sigmaInv*(x-mean)/2.0 ) ;
        assert(value.rows()==1);
        assert(value.cols()==1);
        double lp=value(0,0)+ additiveConstant ;
 //       double lp= sqr(x-mean)/(2*sqr(sigma)) ;//+ log(variance) + (log(2*boost::math::constants::pi<double>()))/2;
        if(lp<0)
        {
            cerr<<"lpn:"<<lp<<endl;
        }
        return lp;
    }

    MultiVarGaussian(std::ifstream & file)
    {
        std::string line;
        string separator=",";
        boost::char_separator<char> sep(separator.data());
        numFeats=0;
        {        
            getline(file, line);
            boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
            BOOST_FOREACH(std::string t, tokens1)
            {
                numFeats++;
            }
            mean.resize(numFeats);
         //   cerr<<filename<<","<<numFeats<<endl;
            int c=0;
            BOOST_FOREACH(std::string t, tokens1)
            {
                mean(c)=boost::lexical_cast<double > (t);
                c++;
            }
            assert(c==numFeats);
        }
        
        {        
            getline(file, line);
            boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
            minv.resize(numFeats);
            int c=0;
            BOOST_FOREACH(std::string t, tokens1)
            {
                minv(c)=boost::lexical_cast<double > (t);
                c++;
            }
            assert(c==numFeats);
        }
        
        {        
            getline(file, line);
            boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
            maxv.resize(numFeats);
            int c=0;
            BOOST_FOREACH(std::string t, tokens1)
            {
                maxv(c)=boost::lexical_cast<double > (t);
                c++;
            }
            assert(c==numFeats);
        }   
        sigmaInv.resize(numFeats,numFeats);
        for (int r = 0; r < numFeats; r++)
        {
            getline(file, line);
            boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
            int c = 0;

            BOOST_FOREACH(std::string t, tokens1)
            {
                sigmaInv(r, c) = boost::lexical_cast<double > (t);
                c++;
            }
            assert(c == numFeats);
        }

            getline(file, line);
            boost::tokenizer<boost::char_separator<char> > tokens1(line, sep);
            double SigInvDetLogMin;
            int c = 0;
            BOOST_FOREACH(std::string t, tokens1)
            {
                SigInvDetLogMin = boost::lexical_cast<double > (t);
                c++;
            }
            assert(c == numFeats);

            cerr<<"logd:"<<fileName<<SigInvDetLogMin<<endl;
//        additiveConstant=(log(2*boost::math::constants::pi<double>()))*numFeats/2.0 - log(sigmaInv.determinant())/2.0;
        additiveConstant=0;
#ifdef DIVIDE_BY_SIGMA        
        additiveConstant= SigInvDetLogMin/2.0;
//        additiveConstant=(log(2*boost::math::constants::pi<double>()))*numFeats/2.0 +SigInvDetLogMin/2.0;

#endif
//        cerr<<"-logdet:"<<sigmaInv.determinant()<<","<<- log(sigmaInv.determinant())<<endl;
    }
    
    
};

class ProbabilityDistribution {
public:
    virtual double minusLogProb(double x)=0;
    virtual double getMean()=0;
    virtual double getVar()=0;
    virtual double getMaxCutoff()=0;
    virtual double getMinCutoff()=0;
    
    bool isInRange(double x)
    {
        return (getMinCutoff()<=x && x<=getMaxCutoff());
    }
};

class Gaussian : public ProbabilityDistribution {
    double mean;
    double sigma;
    double min;
    double max;
    
public: 
    normal nd;
    Gaussian() {
        mean = 1;
        sigma = 0;
        normal ndTemp(mean, sigma);
        nd = ndTemp;
    }
    
    virtual double getMean()
    {
        return mean;
    }
    virtual double getVar()
    {
        return sigma;
    }
    
    Gaussian(double mean, double variance, double min, double max) {
        this->mean = mean;
        this->sigma = variance;
        this->min = min;
        this->max = max;
        normal ndTemp(mean, variance);
        nd = ndTemp;
    }
    
    /**
     * it is actually log(pdf(x)/pdf(mean))
     *  because pdf(x) need not be between 0 and 1
     * @param x
     * @return log(pdf(x)/pdf(mean))
     */
    double minusLogProb(double x)
    {
#ifdef PROPABILITY_RANGE_CHECK
        if(!isInRange(x))
            return infinity();
#endif
        double lp= sqr(x-mean)/(2*sqr(sigma)) ;//+ log(variance) + (log(2*boost::math::constants::pi<double>()))/2;
 //       double lp= sqr(x-mean)/(2*sqr(sigma)) ;//+ log(variance) + (log(2*boost::math::constants::pi<double>()))/2;
        assert(lp>=0);
        return lp;
    }
    
    virtual double getRange()
    {
        double range= max-min;
        return range;
        assert(range>0);
    }
    
    virtual double getMaxCutoff()
    {
        assert(sigma>=0);
        return std::max(max+getRange()/4,mean+6*sigma);
    }

    virtual double getMinCutoff()
    {
        assert(sigma>=0);
        return std::min(min-getRange()/4,mean-6*sigma);
    }
    
};

Eigen::Matrix<float ,Eigen::Dynamic,  Eigen::Dynamic> segMinDistances;

double pointPointDistance(pcl::PointXYZ& point1, pcl::PointXYZ& point2) {
    return sqrt(sqr(point1.x - point2.x) + sqr(point1.y - point2.y) + sqr(point1.z - point2.z));
}

Vector3d pointPointVector(pcl::PointXYZ& point1, pcl::PointXYZ& point2) {
    return Vector3d(point1.x - point2.x, point1.y - point2.y, point1.z - point2.z);
}

typedef set<NonTerminal*, NTSetComparison> NTSet;

pcl::PointCloud<PointT> scene;

PointT getPointFromScene(pcl::PointCloud<PointT> fromScene, int pointIndex) {
    return fromScene.points[pointIndex];
}

int NUMPointsToBeParsed;
int NUMTerminalsToBeParsed;
OccupancyMap<PointT> * occlusionChecker;
//pcl::PointCloud<pcl::PointXY> scene2D;
//pcl::PCLBase<pcl::PointXY>::PointCloudConstPtr scene2DPtr;

void printPoint(pcl::PointXYZ point) {
    cout<<"("<<point.x<<","<<point.y<<","<<point.z<<") "<<endl;
}

Vector3d getSceneOrigin()
{
    Vector4f orig=scene.sensor_origin_;
    return Vector3d(orig(0),orig(1),orig(2));
};

class Symbol {
protected:
    /** total weight to derive all these leaves
     * =max(cost  deriving any child) + cost of applying the rule that gave this
     *  required for being a superior CFG
     */
//    bool featuresComputed;
    bool isDeclaredOptimal;
    double cost;
    double zSquaredSum;
    AdvancedDynamicBitset neighbors;
    vector<NonTerminal*> optimalParents;
    pcl::PointXYZ centroid;
    pcl::PointXYZ minxyz;
    pcl::PointXYZ maxxyz;
    double distanceToBoundary;
    
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

    virtual void reFormatSubTree(NonTerminal * parent /* = 0 */){} // do nothing

   virtual int getNumObjectsSpanned()=0;
    
    virtual Eigen::Vector3d getPlaneNormal() const
    {
        assert(1==2);
    }
    
    template<typename SuperClass>
    bool isOfSubClass()
    {
        SuperClass* dummyTypeCheck=dynamic_cast<SuperClass*>(this);
        return (dummyTypeCheck!=NULL);
    }
    
    virtual ~Symbol(){};
    static float _getPolygonArea(const vector<cv::Point2f>& cv2D)
    {
        vector<cv::Point2f> contour;
        cv::approxPolyDP(cv::Mat(cv2D), contour, 0.01, true);
        return fabs(cv::contourArea(cv::Mat(contour)));
    }
    
    bool isPlanarPrimitive();    
    virtual Symbol * grandChildIfHallucinated()=0;
    virtual  HallucinatedTerminal * getHalChild()=0;

    float getMinDistance(Symbol * other)
    {
        resetSpannedTerminalIterator();
        other->resetSpannedTerminalIterator();
        
        assert(other!=this); // it is a parse tree and not a graph
        
        int index1,index2;
        float minDistance=numeric_limits<float>::infinity();
        while(nextSpannedTerminal(index1))
        {
            while(other->nextSpannedTerminal(index2))
            {
                float distance=segMinDistances(index1,index2);
                
                assert(!isnan(distance)); // one of the segments does not exist
                
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

    Eigen::Vector3d getCentroidVector() const
    {
        return Eigen::Vector3d(centroid.x,centroid.y,centroid.z);
    }

    pcl::PointXY getHorizontalCentroid()
    {
        pcl::PointXY temp;
        temp.x=centroid.x;
        temp.y=centroid.y;
        return temp;
    }

    Eigen::Vector2d getHorizontalCentroidVector()
    {
        return Eigen::Vector2d(centroid.x,centroid.y);
    }
    
    double getCentroidZ()
    {
        return centroid.z;
    }
    
    virtual void appendFeatures(vector<float> & features)
    {
        assert(featuresComputed);
        ColorRGB avgColorO(avgColor);
        
        // the color feats go first
        features.push_back(avgColorO.H*Params::featScale/256.0);
        features.push_back(avgColorO.S*Params::featScale);
        features.push_back(avgColorO.V*Params::featScale);

        assert(features.size()==3); //for safetly. if this fails, change the script which edits color values
        
        features.push_back(distanceToBoundary*Params::featScale);
        // remaining non-color feats
        features.push_back(Params::featScale*(zSquaredSum/(float)numPoints-sqr(centroid.z))); // variance along z
        features.push_back(Params::featScale*(maxxyz.z-minxyz.z));
        features.push_back(centroid.z*Params::featScale);
        features.push_back(maxxyz.z*Params::featScale);
        features.push_back(minxyz.z*Params::featScale);
        
        
        // horizontal convex hull area
        features.push_back(horzArea*Params::featScale);
        
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
    
    void computeColorDiffFeatures(float * colorDiff, Symbol * other)
    {
        ColorRGB avgColorThis(avgColor);
        ColorRGB avgColorOther(other->avgColor);
        colorDiff[0]=(avgColorThis.H - avgColorOther.H)*Params::featScale;
        colorDiff[1]=(avgColorThis.S - avgColorOther.S)*Params::featScale;
        colorDiff[2]=(avgColorThis.V - avgColorOther.V)*Params::featScale;
        
    }
    
    /**
     * 
     * @param other
     * @return +ve if this is infront of other
     */
    float inFrontNessof(Symbol * other)
    {
        Eigen::Vector3d r1=getCentroidVector()-getSceneOrigin();
        Eigen::Vector3d r2=other->getCentroidVector()-getSceneOrigin();
        return (r2.norm()-r1.norm());
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
        isDeclaredOptimal = false;
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
        // works for Teminals and NonTerminals, overridden for NTIntermediate 
        nonIntermediateChildren.push_back(this); 
        // children at lowest level of an NTI will be pushed latest
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

    void setDistanceToBoundary(double distanceToBoundary)
    {
        this->distanceToBoundary = distanceToBoundary;
    }

    double getDistanceToBoundary() const
    {
        return distanceToBoundary;
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
    
    virtual int getNumObjectsSpanned()
    {
        return 0;
    }
    
    virtual ~Terminal(){}
    virtual  HallucinatedTerminal * getHalChild()
    {
        return NULL;
    }
    
    PointT & getPoint(pcl::PointCloud<PointT> & cloud,int i)
    {
        return cloud.points.at(pointIndices.at(i));
    }
    string label;
    virtual Symbol * grandChildIfHallucinated()
    {
        return this;
    }

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
        return "Terminal__"+boost::lexical_cast<std::string>(index+1);
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
    
    void setNeighbors(int numTerminals)
    {
        neighbors.resize(numTerminals,false);
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
        isDeclaredOptimal = true;
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
    Eigen::Vector3d normal;
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
    
    bool isPlaneAlmostInvisible()
    {
        Eigen::Vector4f or4d= scene.sensor_origin_;
        Eigen::Vector3d origin;
        for(int i=0;i<3;i++)
        {
            origin(i)=or4d(i);
        }
        Eigen::Vector3d centv(centroid.x,centroid.y,centroid.z);
        Eigen::Vector3d ray=centv-origin;
        ray.normalize();
        assert(fabs(ray.norm()-1)<0.001);
        assert(fabs(normal.norm()-1)<0.001);
        return(fabs(ray.dot(normal))<0.20);
    }
    
    virtual Eigen::Vector3d getPlaneNormal() const
    {
        return normal;
    }
    
    void setPoint(PointT & p, float x, float y, float z)
    {
        static ColorRGB green(0.0,1.0,0.0);
        p.x=x;
        p.y=y;
        p.z=z;
        p.rgb=green.getFloatRep();
        p.segment=index;
    }
    
    void setAbsoluteCost(double cost)
    {
        assert(!isDeclaredOptimal);
        this->cost=cost;
    }
    
    HallucinatedTerminal(Eigen::Vector3d centroid) : Terminal(totalNumTerminals+numHallucinatedTerminals++)
    {
        neighbors.resize(totalNumTerminals,false);
        int start=scene.size();
        int numPoints=7;
        pointIndices.resize(numPoints);
        scene.points.resize(start+numPoints);
//        this->centroid=centroid;
        for(int i=0;i<numPoints;i++)
        {
            pointIndices.at(i)=start+i;
        }
        
        setPoint(scene.points.at(start+0),centroid(0),centroid(1),centroid(2));
        setPoint(scene.points.at(start+1),centroid(0)+0.1,centroid(1),centroid(2));
        setPoint(scene.points.at(start+2),centroid(0)-0.1,centroid(1),centroid(2));
        setPoint(scene.points.at(start+3),centroid(0),centroid(1)+0.1,centroid(2));
        setPoint(scene.points.at(start+4),centroid(0),centroid(1)-0.1,centroid(2));
        setPoint(scene.points.at(start+5),centroid(0),centroid(1),centroid(2)+0.1);
        setPoint(scene.points.at(start+6),centroid(0),centroid(1),centroid(2)-0.1);
    
        featuresComputed=false;
        computeFeatures(); 
        
        //delete all the points
        // we could avoid adding points totally if we could write code for all featueres to compute directly
        scene.points.resize(start);
    }
    
    void unionMembership(boost::dynamic_bitset<> & set_membership) {
        
        //do nothing
    }

    virtual void colorScene()
    {
        Eigen::Vector3d centroid=getCentroidVector();
        int start=scene.size();
        int numPointsh=20;
        scene.points.resize(start+numPointsh);

        for(int i=0;i<numPointsh;i++)
        {
            double rad=i*0.005;
            Eigen::Vector3d point=centroid+rad*normal;
            setPoint(scene.points.at(start+i),point(0),point(1),point(2));
        }
        scene.points.at(start).rgb=ColorRGB(1.0,0,0).getFloatRep();
//        setPoint(scene.points.at(start+0),centroid(0),centroid(1),centroid(2));
//        setPoint(scene.points.at(start+1),centroid(0)+0.01,centroid(1),centroid(2));
//        setPoint(scene.points.at(start+2),centroid(0)-0.01,centroid(1),centroid(2));
//        setPoint(scene.points.at(start+3),centroid(0),centroid(1)+0.01,centroid(2));
//        setPoint(scene.points.at(start+4),centroid(0),centroid(1)-0.01,centroid(2));
//        setPoint(scene.points.at(start+5),centroid(0),centroid(1),centroid(2)+0.01);
//        setPoint(scene.points.at(start+6),centroid(0),centroid(1),centroid(2)-0.01);
        scene.width=1;
        scene.height=scene.size();
        cerr<<"Added points hal\n";
    }    

    virtual void colorCentroid(double minCost, double maxCost)
    {
        Eigen::Vector3d centroid=getCentroidVector();
        int start=scene.size();
        int numPoints=1;
        scene.points.resize(start+numPoints);
        double scaledCost=(log(cost)-log(minCost))/(log(maxCost)-log(minCost));
        ColorRGB color(1.0-scaledCost,0,scaledCost);
        setPoint(scene.points.at(start+0),centroid(0),centroid(1),centroid(2));
        scene.points.at(start+0).rgb=color.getFloatRep();
        
        scene.width=1;
        scene.height=scene.size();
//        cerr<<"added points hal\n";
    }

    void setNormal(Eigen::Vector3d normal)
    {
        this->normal = normal;
    }    
};

    bool Symbol :: isATerminal()
    {
        Terminal * term= dynamic_cast<Terminal *>(this);
        return (term!=NULL);
    }

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
    int numObjectsSpanned;
    
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
        distanceToBoundary=0;
        
        for (size_t i = 0; i < children.size(); i++) {
            children.at(i)->getCentroid(childCent);
            long numPointsInChild=children.at(i)->getNumPoints();
            numPoints+=numPointsInChild;
            centroid.x += numPointsInChild*childCent.x;
            centroid.y += numPointsInChild*childCent.y;
            centroid.z += numPointsInChild*childCent.z;
            avg+=(children.at(i)->getAvgColor()*numPointsInChild);
            distanceToBoundary += numPointsInChild * (children.at(i)->getDistanceToBoundary());
        }
        centroid.x /= numPoints;
        centroid.y /= numPoints;
        centroid.z /= numPoints;
        avg/=numPoints;
        avgColor=avg.getFloatRep();
        distanceToBoundary/=numPoints;
    }
    
    void computeCovarianceMatrixWoMean()
    {
        covarianceMatrixWoMean=Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < children.size(); i++)
        {
            covarianceMatrixWoMean+=children.at(i)->getCovarianceMatrixWoMean();
        }
        
    }

    /**
     * compute leaves by concatenating
     * leaves of children */
    bool costSet;
public:
    
    virtual bool isPrunable()
    {
        return getCost()>Params::costPruningThreshNonComplex;
    }
    
    virtual ~NonTerminal(){}
    
    
    void reFormatSubTree(NonTerminal * parent /* = 0 */);
    
    virtual int getNumObjectsSpanned()
    {
        assert(isDeclaredOptimal);
        return numObjectsSpanned;
    }

    // Returns true if both Symbols do not overlap
    bool isSpanExclusive(NonTerminal * nt) {
        return !(spanned_terminals.intersects(nt->spanned_terminals));
    }
    /**
     * either non-interse
     * @param nt
     * @return 
     */
    bool isSpanContainedIn(NonTerminal * nt) {
        return (spanned_terminals.is_proper_subset_of(nt->spanned_terminals));
    }
    
    virtual Symbol * grandChildIfHallucinated()
    {
        if(children.size()>1)
            return this;
        
        HallucinatedTerminal * hl=getChild(0)->getHalChild(); // if hallucinated, it's child would be a plane and that plane's child could be Halterm
        if(hl==NULL)
        {
            return this;
        }
        else
            return hl;        
    }

    virtual void resetSpannedTerminalIterator()
    {
        spanned_terminals.iteratorReset();
    }
    
    virtual bool nextSpannedTerminal(int & tindex)
    {
        return spanned_terminals.nextOnBit(tindex);
    }
    
    /**
     * can be used to check if this part was made from a hallucinated terminals ..
     * 
     * @return pointer to halTerm child, NULL if it was not directly made up of halTerm 
     */
    HallucinatedTerminal * getHalChild()
    {
        return dynamic_cast<HallucinatedTerminal* >(getChild(0));
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
        return getCleanedTypeName()+"__"+boost::lexical_cast<std::string>(id)+"__"+boost::lexical_cast<string>((int)getCost());
    }
    
      string getCleanedTypeName() const
      {
        const char * name=typeid(*this).name();
        int count=0;
        while(isdigit(*name)&&count<5)
        {
            name++;
            count++;
        }
        return string(name);
          
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
        assert(!isDeclaredOptimal);
        //assert(additionalCost >= 0);
        cost = 0;
        for (size_t i = 0; i < children.size(); i++)
            cost += children[i]->getCost();
        cost += additionalCost;
        
        for (size_t i = 0; i < children.size(); i++)
        {
            if(cost < children[i]->getCost())
            {
                cerr<<"WARN:nonMon:"<<cost <<","<< children[i]->getCost()<<endl;
            }
        }
        costSet = true;
        cout << "ac:" << additionalCost << ",tc:" << cost << endl; // don't unshorten it unless u want the log files to swell into GBs
    }

    /**
     * caution: use this only when you can prove that this cost
     * larger than the cost of any children
     * @param absoluteCost
     */
    void setAbsoluteCost(double absoluteCost) {
        assert(!isDeclaredOptimal);
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
        cout << "absc " << cost << endl;
    }

    virtual void addChild(Symbol * child) {
        assert(!costSet);
        children.push_back(child);
    }

    virtual void addChildNoCheck(Symbol * child) {
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

    virtual void computeNumSpannedObjects()
    {
        numObjectsSpanned=0;
        for (vector<Symbol*>::iterator it = children.begin(); it != children.end(); it++) 
        {
            numObjectsSpanned=numObjectsSpanned+(*it)->getNumObjectsSpanned() ;
        }
        numObjectsSpanned++;
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
        if(isDeclaredOptimal)
            return true;
        isDeclaredOptimal = true;
        
        for (vector<Symbol*>::iterator it = children.begin(); it != children.end(); it++) {
            (*it)->appendOptimalParents(this);
        }

        computeNeighborTerminalSet();
        assert(costSet); // cost must be set before adding it to pq
        computeFeatures();
        additionalFinalize();
        computeNumSpannedObjects();
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

class NonTerminalIntermediate : virtual public NonTerminal
{
public:
//    virtual void appendFeatures(vector<float> & features)
//    {
//         intermediate types dont contribute to features
////        assert(false);
//    }
    
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
    
    virtual void computeNumSpannedObjects()
    {
        numObjectsSpanned=0;
    }

};

class VisualObject : virtual public NonTerminal
{
public:
    bool doesNotOverlapWithScenes(vector<VisualObject*> & identifiedScenes)
    {
        vector<VisualObject*>::iterator it;
        if(isOfSubClass<NonTerminalIntermediate>()&&cost>80)
            return false;
        for (it = identifiedScenes.begin(); it != identifiedScenes.end(); it++)
        {
            if (!isSpanExclusive(*it))
            {
                if((*it)->isSpanContainedIn(this))
                    ((*it))=this;
                return false;
            }
        }
        return true;
    }
    
};

class VisualObjects : public NonTerminal{};

class Scene : virtual public NonTerminal
{
    static std::ofstream graphvizFile;
    static std::ofstream NTmembershipFile;
    static std::ofstream labelmapFile;

    
    // the printData below should be used only for the Goal NT type
public:
    
    virtual bool isPrunable()
    {
        return false;
    }

    static double COST_THERSHOLD;
    
    static void initFiles()
    {
        string treeFileName = fileName + ".dot";
        string membersipFileName = fileName + "_membership.txt";
        string labelmapFileName = fileName + "_labelmap.txt";

        NTmembershipFile.open(membersipFileName.data(), ios::out);
        labelmapFile.open(labelmapFileName.data(), ios::out);
        graphvizFile.open(treeFileName.data(), ios::out);
        graphvizFile << "digraph g{\n"; // Move to postparse printer
        
    }
    
    static void closeFiles()
    {
        graphvizFile << "}\n"; // Move to postparse printer
        graphvizFile.close(); // Move to postparse printer
        NTmembershipFile.close();
        labelmapFile.close();
      //  string halPCDFileName = fileName + "_hallucinated.pcd";
      //  pcl::io::savePCDFile<PointT > (halPCDFileName, scene, true);
        
    }
    
    static void printAllScenes(vector<VisualObject*> & identifiedScenes)
    {
        initFiles();
        vector<VisualObject*>::iterator it;
        for (it = identifiedScenes.begin(); it != identifiedScenes.end(); it++)
        {
            printData(*it);
        }
        closeFiles();
    }


    void printData()
    {
        initFiles();
        printData(this);
        graphvizFile << "}\n"; // Move to postparse printer        
        graphvizFile.close();
        
        string prunedTreeFileName = fileName + "_pruned.dot";        
        graphvizFile.open(prunedTreeFileName.data(), ios::out);
        graphvizFile << "digraph g{\n"; // Move to postparse printer
        reFormatSubTree(NULL);
        printData(this,true); // pruned parse tree        
        
        closeFiles();
    }
    
    static void printData(NonTerminal * root, bool onlyGraphVis=false)
    {
        pcl::PointCloud<pcl::PointXYZRGBCamSL> sceneOut;
        sceneOut = scene;
        
        graphvizFile << root->getName() << " ;\n";


        stack<NonTerminal*> parseTreeNodes;
        parseTreeNodes.push(root);

        scene.width = 1;
        scene.height = scene.size();
        while (!parseTreeNodes.empty())
        {
            NonTerminal *curNode = parseTreeNodes.top();
            string curName = curNode->getName();
            parseTreeNodes.pop();
            
            if(!onlyGraphVis)
            {
                printNodeData(NTmembershipFile, labelmapFile, curNode);
            }
            
            for (size_t i = 0; i < curNode->getNumChildren(); i++)
            {
                Symbol * childs = curNode->getChild(i);

                graphvizFile << curName << " -> " << childs->getName() << " ;\n";
                Terminal *childT = dynamic_cast<Terminal *> (childs);
                if (childT == NULL) // not of type Terminal
                {
                    //   assert(childs!=NULL);
                    NonTerminal * child = dynamic_cast<NonTerminal*> (childs);
                    assert(child != NULL);
                    parseTreeNodes.push(child);
                }
                else
                {
                    childT->colorScene();
                }
            }

        }

    }

    static void printNodeData(std::ofstream & membershipFile, std::ofstream & labelmapFile, NonTerminal *node)
    {
     //   if (node == this) // will always cover full scene
       //     return;

        membershipFile << node->getId();

        bool isPlanarPrimitive = node->isPlanarPrimitive();
//        if(typeid(*node)==typeid(monitor))
  //          isPlanarPrimitive=true;
        if (isPlanarPrimitive)
            labelmapFile << node->getCleanedTypeName();

        node->resetTerminalIterator();
        int index;
        while (node->nextTerminalIndex(index))
        {
            membershipFile << "," << index + 1;

            if (isPlanarPrimitive)
                labelmapFile << "," << index + 1;
        }
        membershipFile << endl;

        if (isPlanarPrimitive)
            labelmapFile << endl;
    }

};

std::ofstream Scene::graphvizFile;
std::ofstream Scene::NTmembershipFile;
std::ofstream Scene::labelmapFile;
double Scene::COST_THERSHOLD;

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
    Eigen::Matrix3d eigenVecs;    
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
    
    virtual void computeNumSpannedObjects()
    {
        numObjectsSpanned=0;
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

    double getNorm() const {
        return (planeParams[0] * planeParams[0] + planeParams[1] * planeParams[1] + planeParams[2] * planeParams[2]);
    }
    
    double getZNormal() const {
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
    
    static const int NORMAL_Z_INDEX=12;
    virtual void appendAdditionalFeatures(vector<float> & features)
    {
        //assert(featuresComputed);
        features.push_back(getLength()*Params::featScale);
        features.push_back(getWidth()*Params::featScale);
#ifndef DISABLE_HALLUCINATION
        assert(NORMAL_Z_INDEX==(int)features.size()); 
#endif
        features.push_back(getZNormal()*Params::featScale);
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
    
    void computePlaneParamsAndEigens() {
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
        eigenVecs=eigen_vectors;
    }
    
    // TODO: Check if we have to call computeFeatures in this function.
    void computePlaneParamsAndSetCost()
    {
//        if (planeParamsComputed)
//            return;
//
//        computeFeatures();
//        Eigen::Vector4f xyz_centroid_;
//
//        for (int i = 0; i < 3; i++)
//            xyz_centroid_(i) = centroid.data[i];
//        xyz_centroid_(3) = 1;
//
//        Eigen::Matrix3d covMat; //
//        computeCovarianceMat(covMat);
//
//        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm(covMat);
//        EIGEN_ALIGN16 Eigen::Vector3d eigen_values = ei_symm.eigenvalues();
//        EIGEN_ALIGN16 Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors();
//
//        planeParams[0] = eigen_vectors(0, 0);
//        planeParams[1] = eigen_vectors(1, 0);
//        planeParams[2] = eigen_vectors(2, 0);
//        planeParams[3] = 0;
//
//        // Hessian form (D = nc . p_plane (centroid here) + p)
//        planeParams[3] = -1 * planeParams.dot(xyz_centroid_);
//        planeParamsComputed = true;
//
//        eigenValsAscending=eigen_values;
//        eigenVecs=eigen_vectors;
//        double sumSquaredDistances = eigen_values(0);
//        // Serious bug. This is called after setting the real cost.
        computePlaneParamsAndEigens();
        double sumSquaredDistances = eigenValsAscending(0);
        setAbsoluteCost(sumSquaredDistances);
    }
    
    /**
     * return the eigenvector with i'th smallest eigenvalue
     * @param i
     * @return 
     */
    Eigen::Vector3d getEigenVector(int i) const
    {
        return eigenVecs.col(i);
    }
    
    Eigen::Vector3d getLengthDirection() const
    {
        return getEigenVector(2);
    }
    
    Eigen::Vector3d getWidthDirection() const
    {
        return getEigenVector(1);
    }
    
    Eigen::Vector3d getPlaneNormal() const {
        return Vector3d(planeParams[0], planeParams[1], planeParams[2]);
    }
    
    Eigen::Vector4f getPlaneParams() const {
        return planeParams;
    }
    
    float getLength() const
    {
        assert(planeParamsComputed);
        return 2*sqrt(eigenValsAscending(2)/(float)getNumPoints()); // rms
    }
    float getWidth() const
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
        if (costOfAddingPoint(p) > .1) {
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

class PlanarPrimitive : virtual public NonTerminal {
public:
    const Plane * getPlaneChild() const
    {
        assert(children.size()==1);
        Plane * ret=dynamic_cast<Plane *>(children.at(0));
        assert(ret!=NULL);
        return ret;
    }
//    
//    virtual void addChild(Symbol * child) {
//        assert(typeid(*child)==typeid(Plane));
//        NonTerminal::addChild(child);
//    }
    virtual Eigen::Vector3d getPlaneNormal() const
    {
        return getPlaneChild()->getPlaneNormal();
    }
};

    bool Symbol::isPlanarPrimitive()
    {
        PlanarPrimitive *temp=dynamic_cast<PlanarPrimitive *>(this);
        return (temp!=NULL);
    }
    
    
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
        if (sym->isPrunable() ) {
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
        cout<<"sizeq"<<costSortedQueue.size()<<endl;
        duplicate=false;
        if(costSortedQueue.empty())
            return NULL;
        Symbol * top = costSortedQueue.top();
        assert(top!=NULL);
        costSortedQueue.pop();
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
    bool modelFileMissing;
public:
    string filename;
    /**
     * @param extractedSym : the extracted Symbol, guaranteed not to be a duplicate of anything was already extracted ... 
     * @param pqueue : the priority queue
     * @param terminals : set of all terminals
     */
    virtual void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */) = 0;
    
    vector<ProbabilityDistribution*> g; // not used
    
    MultiVariateProbabilityDistribution * pdist;
    
    Rule()
    {
        pdist=NULL;
        modelFileMissing=false;
    }
    
    ProbabilityDistribution * getNormalZDist()
    {
        assert(false); // not in use
        return g.at(Plane::NORMAL_Z_INDEX);
    }
    
    void readDistribution(string filename) {
    //    ifstream inputStream;
        this->filename=filename;
        filename.append(".model");
        modelFileMissing=false;
        ifstream file(filename.data(), std::ios::in);
        if(!file.is_open())
        {
           // cerr<<"no mainmodel found for rule:"<<typeid(*this).name()<<endl;
            modelFileMissing=true;
            return;
        }
        pdist=new MultiVarGaussian(file);
        file.close();
    }
    
    void readDistributionInd(string filename) {
        ifstream inputStream;
        filename.append(".out");
        inputStream.open(filename.data());
        assert(inputStream.is_open());
            string line;

            while (inputStream.good()) {
                getline(inputStream, line);
                
                if(line.size()==0)
                    break;
                
                vector<float> nbrs;
                getTokens(line, nbrs);
                Gaussian *tempGaussian=new Gaussian(nbrs.at(0), nbrs.at(1), nbrs.at(2), nbrs.at(3));
                g.push_back(tempGaussian);
            }
    }
    
//    /**
//     * NUMERICAL ISSUES: DO NOT USE IT
//     * Calculate probability of feature vector x in the observed mixture of Gaussians.
//     * @param x
//     * @return 
//     */
//    float getProbability(vector<float> x) {
//        double product = 1;
//        int counter = 0;
//        BOOST_FOREACH(Gaussian gaussian, g) {
//            double term=pdf(gaussian.nd, (double) x.at(counter));
//            product = product * term;
//            assert(term>=0);
//            assert(term<=1);
//            counter = counter + 1;
//        }
//        return product;
//    }
    
    /**
     * Calculate -log prob probability of feature vector x in the observed mixture of Gaussians.
     * @param x
     * @return 
     */
    double getMinusLogProbability(vector<float> & x) {
        double ret= pdist->minusLogProb(x);
        
        if(ret>Params::additionalCostThreshold)
        {
            return infinity();
        }
        else
        {
            return ret;
        }
    }
    
    const vector<float> & getFeatures() const
    {
        return features;
    }
    
//    virtual void computeFeatures(){};
    
    void writeFeaturesToFile()
    {
        assert(featureFile.is_open()); // you need to construct this rule with false for learning
        vector<float>::iterator it;
        for(it=features.begin();it!=features.end();it++)
        {
            featureFile<<*it<<",";
        }
        featureFile<<endl;
    }
    

    virtual bool addToPqueueIfNotDuplicate(NonTerminal * newNT, SymbolPriorityQueue & pqueue) {
      //  newNT->computeSetMembership(); // required for duplicate check
        if(newNT==NULL)
            return false;
            
        if(isinf(newNT->getCost()) || !pqueue.pushIfNoBetterDuplicateExistsUpdateIfCostHigher(newNT))
        {
            delete newNT;
            return false;
        }
        else
        {
            return true;
        }
    }
    
    ~Rule()
    {
        if(pdist!=NULL)
                delete pdist;
        for(int i=0;i<(int)g.size();i++)
            delete g.at(i);
    }

    bool isModelFileMissing() const
    {
        return modelFileMissing;
    }
};
/**
 * using templates, same class can be used to hold features(T=double) and
 *  models for corresponding features(T=ProbabilityDistribution *)
 * also, using union, lets us access info by name and also as array
 */
template<typename T>
class PairInfo
{
    const static int NUM_OCCLUSION_FEATS = 9;
    const static int NUM_OCCLUSION_FEATS_ASYMMETRIC = 3;
    const static int NUM_FEATS = 27;
    const static double UNKNOWN_FEATURE_LOG_PROB=5.0;
public:

    union
    {
        // add new features here and update the static constants. make use to set them later
        T all[NUM_FEATS];

        struct
        {
            T centDist;
            T centDistHorz;
            T centZDiff12;
            T distOfC2AlongEV1[3];
            T distOfC1AlongEV2[3];
            T EVdots12[9];
            T z1Min_2Max;
            T z1Max_2Min;
            T z1Min_2Min;
            T z1Max_2Max;
            T minDist;
            T frontness; // change NUM_FEATS above
            T colorDiffHSV[3];
        };
    };

    bool type1Hal;
    bool type2Hal;
    
    PairInfo()
    {
        //cerr<<sizeof(PairInfo<T>)<<","<<sizeof(bool)<<","<<sizeof(T)<<endl;
        assert(sizeof(PairInfo<T>)-2*sizeof(bool)>=NUM_FEATS*sizeof(T)); // assuming all fields in struct are useful, we dont want to loose any
        type1Hal=true;
        type2Hal=true;
    }
    
    T getPlaneDPInfo()
    {
        return EVdots12[0];
    }
    
    void pushToVector(vector<T> & infos)
    {
        for(int i=0;i<NUM_FEATS;i++)
            infos.push_back(all[i]);
    }
    
    void readInfo(typename vector<T>::const_iterator & it)
    {
        

        for(int i=0;i<NUM_FEATS;i++)
        {
            all[i]=*it;
            it++;
        }
        // all.insert(all.begin(),models.begin()+start,models.begin()+start+6);

        //       centDist=models.at(start);
        //       centDistHorz=models.at(start+1);
        //       centZDiff=models.at(start+2);
        //       for(int i=0;i<3;i++)
        //       {
        //           distAlongEV[i]=models.at(start+3+i);
        //       }
    }
    
    void computeInfo(Symbol * rhs1, Symbol * rhs2);
    
//    void computeInfoOcclusion(Symbol * extracted, HallucinatedTerminal * hal, bool type2Hal)
//    {
//        if(type2Hal)
//            computeInfo(extracted->childIfHallucinated(),hal,true );
//        else
//            computeInfo(extracted->childIfHallucinated(),extracted,true ); // type 1 hallucinated
//            
//    }
    

//    static double computeMinusLogProbHal(PairInfo<float> & feats, PairInfo<ProbabilityDistribution*> & models, bool type2Hal)
//    {
//        double sum=0;
////        for(int i=0;i<3;i++)
//        for(int i=0;i<(NUM_OCCLUSION_FEATS-2*NUM_OCCLUSION_FEATS_ASYMMETRIC);i++)
//        {
//            sum+=models.all[i]->minusLogProb(feats.all[i]);
//        }
//        
//        int asymStart,asymEnd;
//
//        if (type2Hal)
//        {
//            asymStart=NUM_OCCLUSION_FEATS-2*NUM_OCCLUSION_FEATS_ASYMMETRIC;
//            asymEnd=NUM_OCCLUSION_FEATS-NUM_OCCLUSION_FEATS_ASYMMETRIC;
//        }
//        else
//        {
//            asymStart=NUM_OCCLUSION_FEATS-NUM_OCCLUSION_FEATS_ASYMMETRIC;
//            asymEnd=NUM_OCCLUSION_FEATS;
//        }
//            
//        for(int i=asymStart;i<asymEnd;i++)
//        {
//            sum+=models.all[i]->minusLogProb(feats.all[i]);
//        }
//        
//     //   sum+=(NUM_FEATS-NUM_OCCLUSION_FEATS+NUM_OCCLUSION_FEATS_ASYMMETRIC); // for unaccounted features
//        return sum;
//    }
    
    static double computeMinusLogProb(PairInfo<float> & feats, PairInfo<ProbabilityDistribution*> & models)
    {
        double sum=0;
        if (!(feats.type1Hal || feats.type2Hal)) // none of them hallucinated
        {
            for (int i = 0; i < NUM_FEATS; i++)
            {
                sum += models.all[i]->minusLogProb(feats.all[i]);
            }
        }
        else
        {
           // assert(false);
            for (int i = 0; i < (NUM_OCCLUSION_FEATS - 2 * NUM_OCCLUSION_FEATS_ASYMMETRIC); i++)
            {
                sum += models.all[i]->minusLogProb(feats.all[i]);
            }

            if (feats.type1Hal && feats.type2Hal) // both hallucinated
                return sum+UNKNOWN_FEATURE_LOG_PROB*(NUM_FEATS-NUM_OCCLUSION_FEATS+2*NUM_OCCLUSION_FEATS_ASYMMETRIC);

            int asymStart, asymEnd;

            if (feats.type2Hal)
            {
                asymStart = NUM_OCCLUSION_FEATS - 2 * NUM_OCCLUSION_FEATS_ASYMMETRIC;
                asymEnd = NUM_OCCLUSION_FEATS - NUM_OCCLUSION_FEATS_ASYMMETRIC;
            }
            else
            {
                asymStart = NUM_OCCLUSION_FEATS - NUM_OCCLUSION_FEATS_ASYMMETRIC;
                asymEnd = NUM_OCCLUSION_FEATS;
            }

            for (int i = asymStart; i < asymEnd; i++)
            {
                sum += models.all[i]->minusLogProb(feats.all[i]);
            }
            
            int normalDPIndex=NUM_OCCLUSION_FEATS;
            sum += models.all[normalDPIndex]->minusLogProb(feats.all[normalDPIndex]);
            
            sum+=UNKNOWN_FEATURE_LOG_PROB*(NUM_FEATS-NUM_OCCLUSION_FEATS+NUM_OCCLUSION_FEATS_ASYMMETRIC);

        }
        return sum;
    }

    static double computeMinusLogProbHal(vector<Symbol*> & extractedSymExpanded, HallucinatedTerminal & halTerm, vector<PairInfo<ProbabilityDistribution*> > & allpairmodels, bool type2Hal /* = true */)
    {
        double sum=0;
        assert(allpairmodels.size()==extractedSymExpanded.size());
        for(unsigned int i=0;i<allpairmodels.size();i++)
        {
  //          PairInfo<float> feats;
//            feats.computeInfo(extractedSymExpanded.at(i), &halTerm); // ignore some models
            if(type2Hal)
                sum+=computeMinusLogProb(extractedSymExpanded.at(i)->grandChildIfHallucinated(), & halTerm, allpairmodels.at(i)); // ignore some models based on HAL
            else
                sum+=computeMinusLogProb( & halTerm,extractedSymExpanded.at(i)->grandChildIfHallucinated(), allpairmodels.at(i)); // ignore some models based on HAL
                
        }        
        return sum;
    }
    
//    static double computeMinusLogProbHal(Symbol* sym, HallucinatedTerminal * halTerm, PairInfo<ProbabilityDistribution*> & pairmodel, bool type2Hal /* = true */)
//    {
//            PairInfo<float> feats;
//            feats.computeInfoOcclusion(sym, halTerm,type2Hal); // ignore some models
//            return computeMinusLogProbHal(feats,pairmodel,type2Hal); // ignore some models based on HAL
//    }
    
    static double computeMinusLogProb(Symbol* rhs1, Symbol* rhs2, PairInfo<ProbabilityDistribution*> & pairmodel)
    {
            PairInfo<float> feats;
            feats.computeInfo(rhs1, rhs2); 
            return computeMinusLogProb(feats,pairmodel); // ignore some models based on HAL
    }
};

template<>
void PairInfo<float>::computeInfo(Symbol * rhs1, Symbol * rhs2)
{
    //centroid related features
    centDist=(rhs1->centroidDistance(rhs2)*Params::featScale);
    centDistHorz=(rhs1->centroidHorizontalDistance(rhs2)*Params::featScale);
    centZDiff12=(rhs1->getCentroid().z - rhs2->getCentroid().z)*Params::featScale;

    Eigen::Vector3d c1 = rhs1->getCentroidVector();
    Eigen::Vector3d c2 = rhs2->getCentroidVector();
    Eigen::Vector3d c12 = c1 - c2;


    PlanarPrimitive * plane1 = dynamic_cast<PlanarPrimitive *> (rhs1);
    PlanarPrimitive * plane2 = dynamic_cast<PlanarPrimitive *> (rhs2);
        //fabs because the eigen vector directions can be flipped 
        // and the choice is arbitrary

    if(plane1!=NULL) 
    {
        for (int i = 0; i < 3; i++)
            distOfC2AlongEV1[i]=(fabs(c12.dot(plane1->getPlaneChild()->getEigenVector(i))))*Params::featScale;
        type1Hal=false;
        
    }   

    
    if(plane2!=NULL) 
    {
        for (int i = 0; i < 3; i++)
            distOfC1AlongEV2[i]=(fabs(c12.dot(plane2->getPlaneChild()->getEigenVector(i))))*Params::featScale;
        type2Hal=false;
    }

        if(type1Hal||type2Hal)
        {
            EVdots12[0]=(rhs1->getPlaneNormal().dot(rhs2->getPlaneNormal()))*Params::featScale;            
            return;
        }

        assert(plane1 != NULL && plane2 != NULL);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                EVdots12[i*3+j]=(fabs(plane1->getPlaneChild()->getEigenVector(i).dot(plane2->getPlaneChild()->getEigenVector(j))))*Params::featScale;


    z1Min_2Max=(rhs1->getMinZ() - rhs2->getMaxZ())*Params::featScale;
    z1Max_2Min=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;
    z1Min_2Min=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;
    z1Max_2Max=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;





    minDist=(rhs1->getMinDistance(rhs2))*Params::featScale;
    
    frontness=rhs1->inFrontNessof(rhs2)*Params::featScale;
    rhs1->computeColorDiffFeatures(colorDiffHSV, rhs2);
    
    //assert((int) features.size() == beginSize + NUM_FEATS_PER_PAIR);
    //get horizontal area ratio
    //area ratio on plane defined by normal
}

class HalLocation
{
public:
    double rad;
    double z;
    double angle;
    
    Eigen::Vector2d getUnitVector()
    {
        return Eigen::Vector2d(cos(angle),sin(angle));
    }
    
    Eigen::Vector3d getCentroid(Eigen::Vector2d centxy)
    {
        assert(rad>=0);
        Eigen::Vector2d hal2d=centxy+rad*getUnitVector();
        return Eigen::Vector3d(hal2d(0),hal2d(1),z);
    }
};

template<typename LHS_Type, typename RHS_Type>
class SingleRule : public Rule
{
    virtual void combineAndPushForParam(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        RHS_Type* RHS_extracted = dynamic_cast<RHS_Type *>(extractedSym);
        NonTerminal * newNT=applyRuleinference(RHS_extracted,terminals);
        
        if(newNT!=NULL)
        {
                addToPqueueIfNotDuplicate(newNT, pqueue);
        }
    }

    virtual void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) == typeid (RHS_Type))
        {
            combineAndPushForParam(extractedSym, pqueue, terminals, iterationNo);
        }
    }

    bool learning;
public:
    virtual double getCostScaleFactor()
    {
        return 1.0;
    }
    
    SingleRule(string dummyConstructor){}
    
    SingleRule(bool learning=false)
    {
        this->learning=learning;
        string filename=string("rule_")+string(typeid(LHS_Type).name())+"__"+string(typeid(RHS_Type).name());
        if(learning)
        {
                featureFile.open(filename.data(),ios::app); // append to file
        }else {
            readDistribution(rulePath+"/"+filename);
        }
    }
    
    virtual void computeFeatures(RHS_Type* input)
    {
        features.clear();
        computeFeaturesSpecializable(input);
    }
    
    /**
     * to be specialied by rules ishing to add all feats on their own
     * @param input
     */
    virtual void computeFeaturesSpecializable(RHS_Type* input)
    {
        input->appendFeatures(features);
        computeAdditionalFeats(input);
    }
    
    /** to be specialized for rules requiring additional features
     */
    virtual void computeAdditionalFeats(RHS_Type* input)
    {
    }
    
     /**
    
     * @param output
     * @param input
     */
//    bool setCost(LHS_Type* output, RHS_Type* input, vector<Terminal*> & terminals)
//    {
//        assert(3 == 2);
//    }
    
    virtual bool setCost(LHS_Type* output, RHS_Type* input, vector<Terminal*> & terminals) {
        double cost=getMinusLogProbability(features);
        output->setAdditionalCost(cost*getCostScaleFactor());
        
        if(isinf(cost))
            return false;
        else
            return true;
    }
    
    virtual LHS_Type* applyRuleLearning(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS, terminals);
        
        computeFeatures(RHS);
        writeFeaturesToFile();
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        
        return LHS;
        
    }
    
    virtual LHS_Type* applyRuleinference(RHS_Type* RHS, vector<Terminal*> & terminals)
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

    virtual LHS_Type* applyRuleGeneric(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS);
        LHS->computeSpannedTerminals();
        return LHS;
    }
    
    virtual void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        combineAndPushGeneric(extractedSym, pqueue, terminals, iterationNo);
    }
}; 

template<typename LHS_Type, typename RHS_Type1, typename RHS_Type2 >
class DoubleRule : public Rule
{
protected:
    int hello;
    //    template<typename RHS_Type1, typename RHS_Type2>

//    template<typename NT_PlaneType>
//    NT_PlaneType * generateHalNT(HalLocation & loc, Eigen::Vector2d centroidxy)
//    {
//        HallucinatedTerminal finalHal(loc.getCentroid(centroidxy));
//        finalHal.setNeighbors(Terminal::totalNumTerminals);
//        finalHal.declareOptimal();
//
//        Plane* pl = new Plane();
//        pl->addChild(&finalHal);
//        pl->computeSpannedTerminals();
//        pl->computeFeatures();
//        pl->setAbsoluteCost(0);
//        pl->declareOptimal();
////        plane->returnPlaneParams();
//            vector<Terminal*> dummy;
//             
//       SingleRule<NT_PlaneType, Plane> ruleCPUFront(false);
//       return ruleCPUFront.applyRuleLearning(pl, dummy);
//        
//    }
    
    vector<PairInfo<ProbabilityDistribution *> > modelsForLHS; // LHS might me an intermediate for multiple NTs
    
   virtual void readPairModels(int numSymsInRHS1)
    {
        modelsForLHS.resize(numSymsInRHS1);
        
        vector<ProbabilityDistribution*>::const_iterator itr=g.begin();
        for(int i=0;i<numSymsInRHS1;i++)
        {
            modelsForLHS.at(i).readInfo(itr);
        }
    }
    
    virtual void combineAndPushForParam1(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {

        RHS_Type1 * RHS_extracted = dynamic_cast<RHS_Type1 *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL)
        {
            if ( nt->isOfSubClass<RHS_Type2>())
            {
                RHS_Type2 * RHS_combinee = dynamic_cast<RHS_Type2 *> (nt);
                addToPqueueIfNotDuplicate(applyRuleInference (RHS_extracted, RHS_combinee,terminals), pqueue);
            }
            nt = finder.nextEligibleNT();
        }
        
#ifndef DISABLE_HALLUCINATION        
          tryToHallucinate<RHS_Type1,RHS_Type2>(RHS_extracted,pqueue,terminals,iterationNo,true);
#endif

    }


    virtual void combineAndPushForParam2(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {

        RHS_Type2 * RHS_extracted = dynamic_cast<RHS_Type2 *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL)
        {
            if ( nt->isOfSubClass<RHS_Type1>())
            {
                RHS_Type1 * RHS_combinee = dynamic_cast<RHS_Type1 *> (nt);
                addToPqueueIfNotDuplicate(applyRuleInference(RHS_combinee, RHS_extracted,terminals), pqueue);
            }
            nt = finder.nextEligibleNT();
        }
        
#ifndef DISABLE_HALLUCINATION        
        tryToHallucinate<RHS_Type2,RHS_Type1>(RHS_extracted,pqueue,terminals,iterationNo,false);
#endif
    }


    virtual void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
        if (extractedSym->isOfSubClass<RHS_Type1>())
        {
            combineAndPushForParam1(extractedSym, pqueue, terminals, iterationNo);
        }
        else if ( extractedSym->isOfSubClass<RHS_Type2>())
        {
            combineAndPushForParam2(extractedSym, pqueue, terminals, iterationNo);
        }
    }

    template <typename ExtractedType, typename HalType>
    typename boost::enable_if<boost::is_base_of<PlanarPrimitive, HalType>, void>::type
    tryToHallucinate(ExtractedType * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */, bool type2Hallucinated)
    {
        vector<Symbol*> extractedSymExpanded;
        extractedSym->expandIntermediates(extractedSymExpanded);
        
        //if(typeid(HalType)!=typeid(CPULSide))
        
   //     if(extractedSymExpanded.size()<MAX_SEG_INDEX)
     //       return;
//        for(unsigned int i=0;i<extractedSymExpanded.size();i++)
//            cerr<<"type NTIC:"<<i<<typeid(*extractedSymExpanded.at(i)).name()<<endl;
        
        int numNodes = extractedSymExpanded.size();
        if (modelsForLHS.size() == 0)//only called the 1st time this rule is used
        {
            readPairModels(numNodes);
        }
        //find the XY centroids

        
//        for(int i=0;i<numNodes;i++)
//        {
//            cerr<<"centroids "<<i<<":";
//            cerr<<extractedSymExpanded.at(i)->getCentroidVector();
//            cerr<<"\n";
//        }
//        
//        for(int i=0;i<numNodes;i++)
//        {
//            cerr<<modelsForLHS.at(i).centDist->getMean()<<endl;
//        }
//        
//        for(int i=0;i<numNodes;i++)
//        {
//            cerr<<modelsForLHS.at(i).centDist->getVar()<<endl;
//        }
        

        Eigen::Vector2d sum(0, 0);

        Eigen::Vector2d centxy[numNodes];
        int count = 0;
        //compute the region to sample
        // get the centroid of centroids of the available parts
        // this will be the centre(x,y component) of the vertical cylinder to sample
        for (vector<Symbol*>::iterator it = extractedSymExpanded.begin(); it != extractedSymExpanded.end(); it++)
        {
            centxy[count] = (*it)->getHorizontalCentroidVector();
            sum += centxy[count];
            count++;
        }
        Eigen::Vector2d centroidxy = sum / numNodes;

        // estimate the radius and starting and ending heights of the cylinder
        double maxRange = 0; // radius of culinder
        double overallMaxCZ = -infinity(); // ending Z of cylinder
        double overallMinCZ = infinity(); // starting Z of cylinder
        double maxCZ,minCZ;
        Eigen::Vector2d disp;
        for (int i = 0; i < numNodes; i++)
        {
            disp = centroidxy - centxy[i];
            double nodeCZ = extractedSymExpanded.at(i)->getCentroidZ();
            double ccDist = disp.norm();
            double range = modelsForLHS.at(i).centDistHorz->getMaxCutoff();
            
            if(type2Hallucinated)
            {
                //diff12=c1-c2 => c2=c1-diff12
                maxCZ = nodeCZ - modelsForLHS.at(i).centZDiff12->getMinCutoff();
                minCZ = nodeCZ - modelsForLHS.at(i).centZDiff12->getMaxCutoff();
            }
            else
            {
                //diff12=c1-c2 => c1=c2+diff12
                maxCZ = nodeCZ + modelsForLHS.at(i).centZDiff12->getMaxCutoff();
                minCZ = nodeCZ + modelsForLHS.at(i).centZDiff12->getMinCutoff();                
            }
            
            assert(range >= 0);
            assert(range < 3);
            if (maxRange < range + ccDist)
                maxRange = range + ccDist;
            if (overallMaxCZ < maxCZ)
                overallMaxCZ = maxCZ;
            if (overallMinCZ > minCZ)
                overallMinCZ = minCZ;


        }
        assert(overallMaxCZ < 2);
        assert(overallMinCZ>-0.5);
        assert(overallMinCZ <= overallMinCZ);

        //        HallucinatedTerminal halTerm(centroid);
        double minCost = infinity();
        HalLocation halLoc, minHalLoc;

#ifdef OCCLUSION_SHOW_HEATMAP
        
        double maxCost = -infinity();
        vector<HallucinatedTerminal> halTermsForHeatmap;
#endif
//        PairInfo<float> feats;
       SingleRule<HalType, Plane> ruleCPUFront(false); // not for learning

    namespace bg = boost::geometry;
    typedef bg::point<double, 3, bg::cs::cartesian> cartesian;
    typedef bg::point<double, 2, bg::cs::spherical<bg::degree> > spherical;
    
    double minNormalCost=infinity();
    spherical minAngles;
       
       ProbabilityDistribution * dist=ruleCPUFront.getNormalZDist();
       //estimate normal
        for(double elev = 0.0; elev <= 90.0; elev += 22.5)
        {
            for (double azimuth = 0.0; azimuth <= 180.0; azimuth += 10.0)
            {
                double cost=0;
                Eigen::Vector3d halNormal=getDirection(azimuth,elev);
                int count=0;
                cost+=dist->minusLogProb(fabs(halNormal(2))); // z component of normal
                for (vector<Symbol*>::iterator it = extractedSymExpanded.begin(); it != extractedSymExpanded.end(); it++)
                {
                    Eigen::Vector3d otherNormal=(*it)->getPlaneNormal();
                    double dotP=fabs(otherNormal.dot(halNormal));
                    cost+=modelsForLHS.at(count).getPlaneDPInfo()->minusLogProb(dotP);
                    count++;
                }
                if(minNormalCost>cost)
                {
                    minNormalCost=cost;
                    minAngles=spherical(azimuth,elev);
                }
            }
        }
       
       Eigen::Vector3d optimalNormal=getDirection(minAngles.get<0>(),minAngles.get<1>());
       
       
        // sample each point in the cylinder. density: points separated by 2 cm
        cout<<overallMinCZ<<","<<overallMaxCZ<<","<<maxRange<<endl;
        double cost = 0;
        for (halLoc.z = overallMinCZ; halLoc.z <= overallMaxCZ; halLoc.z += 0.02)
        {
            for (halLoc.rad = 0.0; halLoc.rad <= maxRange; halLoc.rad += 0.02/*2 cm*/)
            {
                int numAngles = 1;
                if (halLoc.rad >= 0.02)
                {
                    numAngles=ceil(2 * boost::math::constants::pi<double>() * halLoc.rad / 0.02);
                }

                for (int i = 0; i < numAngles; i++)
                {
                    halLoc.angle = 2.0 * i * boost::math::constants::pi<double>() / numAngles;
                    HallucinatedTerminal halTerm(halLoc.getCentroid(centroidxy));
                    halTerm.setNormal(optimalNormal);
                //    cerr<<"centroid:\n"<<halTerm.getCentroidVector()<<endl;
               //     cerr<<halLoc.getCentroid(centroidxy)<<endl;
               //     cerr<<halTerm.getCentroidVector()<<endl;
                    assert((halLoc.getCentroid(centroidxy)-halTerm.getCentroidVector()).norm()<0.01);
  //                  feats.computeInfoOcclusion(extractedSym, &halTerm,true);
                    cost = PairInfo<float>::computeMinusLogProbHal(extractedSymExpanded, halTerm, modelsForLHS, type2Hallucinated);
                    
#ifdef OCCLUSION_SHOW_HEATMAP
                    halTerm.setAbsoluteCost(cost);
                    halTermsForHeatmap.push_back(halTerm);
                    if(maxCost< cost)
                    {
                        maxCost=cost;
                    }
#endif                    
                  //  cerr<<"cost:"<<cost<<endl;
                    if (minCost > cost)
                    {
                        minCost = cost;
                        minHalLoc = halLoc;
                    }

                }
            }
        }
        
      //  assert(!isinf(minCost)); // this should not happen. if it does for a good reason, just replace by the line below which just returns 
        if(isinf(minCost)) return;

#ifdef OCCLUSION_SHOW_HEATMAP
        
        for(vector<HallucinatedTerminal>::iterator ith=halTermsForHeatmap.begin();ith!=halTermsForHeatmap.end();ith++)
        {
            (*ith).colorCentroid(minCost,maxCost);
        }
#endif        
     //   cerr<<"optimal is:"<<minHalLoc.getCentroid(centroidxy)<<endl;
        HallucinatedTerminal *finalHal=new HallucinatedTerminal(minHalLoc.getCentroid(centroidxy));
        finalHal->setNormal(optimalNormal);
        finalHal->setNeighbors(Terminal::totalNumTerminals);
        finalHal->declareOptimal();

        Plane* pl = new Plane();
        pl->addChild(finalHal);
        pl->computeSpannedTerminals();
        pl->computeFeatures();
        pl->setAbsoluteCost(0);
        pl->declareOptimal();
//        pl->setAbsoluteCost(0);
//        plane->returnPlaneParams();
            vector<Terminal*> dummy;
            
       HalType *halPart=ruleCPUFront.applyRuleGeneric(pl, dummy);
       double additionalCost=800+std::max(3-numNodes,0)*400;
       halPart->setAdditionalCost(additionalCost);// replace with cost of forming a plane by estimating nof points
       halPart->declareOptimal();

       LHS_Type  *lhs;
       if(type2Hallucinated)
       {
           lhs=applyRuleGeneric(extractedSym,halPart,dummy);
       }
       else 
       {
           lhs=applyRuleGeneric(halPart,extractedSym,dummy);
       }
      

        lhs->setAdditionalCost(minCost); // ideally max of other feature values which were not considered
        bool occluded = occlusionChecker->isOccluded(minHalLoc.getCentroid(centroidxy));
        bool almostInvisible = finalHal->isPlaneAlmostInvisible();
        bool hallucinable= occluded || almostInvisible;
      //  cerr<<finalHal->getName()<<"(hallucinated):"<<occluded<<almostInvisible<<endl;
        if(hallucinable && addToPqueueIfNotDuplicate(lhs,pqueue))
        {
             //   cerr<<typeid(LHS_Type).name()<<"hallucinated with cost"<<minCost<<endl;
        }
        else
        {
          // cerr<<"rejected: "<<typeid(LHS_Type).name()<<" hallucinated with cost"<<minCost<<endl;
           delete finalHal;
           delete pl;
        }   
        
        
        // vary z according to the centz diff range 

    }
   
    template <typename ExtractedType, typename HalType>
    typename boost::disable_if<boost::is_base_of<PlanarPrimitive, HalType>,void>::type
    tryToHallucinate(ExtractedType * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */, bool type2Hallucinated)
    {
        //cerr<<"nohal"<<typeid(HalType).name()<<endl;
    }
    
public:
    /**
     * This must be overriden by the inheriting class as each Rule will have its own specific cost function.
     * @param output
     * @param input
     */
    
    const static int NUM_FEATS_PER_PAIR=26;
    
    virtual bool isLearned()
    {
        return true;
    }
    
    DoubleRule(string dummyConstructor){}
    DoubleRule(bool learning=false)
    {
        if (isLearned())
        {
            string filename = string(string("rule_") + typeid (LHS_Type).name()) + "__" + string(typeid (RHS_Type1).name()) + "_" + string(typeid (RHS_Type2).name());
            if (learning)
            {
                // LHS__RHS1_RHS2
                featureFile.open(filename.data(), ios::app); // append to file
            }
            else
            {
                readDistribution(rulePath+"/"+filename);
            }
        }
    }

//    bool setCost(LHS_Type* output, RHS_Type1* RHS1, RHS_Type2* RHS2, vector<Terminal*> & terminals)
//    {
//        assert(3 == 2); // needs specialization
//    }

    /**
     * 
     * @param rhs1 : a nonterminal(possibly obtained after expansion) from the 1st RHS of the rule
     * @param rhs2 : the second NT of the rule
     */
    virtual void appendPairFeatures(Symbol * rhs1, Symbol * rhs2)
    {
        PairInfo<float> pairFeats;
        pairFeats.computeInfo(rhs1,rhs2);
        pairFeats.pushToVector(features);
    }
    
    virtual void appendAllFeatures(LHS_Type* output, RHS_Type1 * rhs1, RHS_Type2 * rhs2, vector<Terminal*> & terminals)
    {
        features.clear();
        
        vector<Symbol*> RHS1Expanded;
        vector<Symbol*> RHS2Expanded;
        rhs1->expandIntermediates(RHS1Expanded);
        rhs2->expandIntermediates(RHS2Expanded);
        assert(RHS2Expanded.size()==1); // 2nd RHS should not be of intermediate type coz we cannot hallucinate a complicated type
        
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
        
       // output->computeFeatures();
     //   Symbol * rhs1;
      //  Symbol * rhs2;
        
        //disabled for occlusion handling
        
//        float rhs1Area=rhs1->getHorzArea();
//        float rhs2Area=rhs2->getHorzArea();
//        float lhsArea=output->getHorzArea();
//        features.push_back(rhs1Area+rhs2Area-lhsArea);
//        features.push_back(lhsArea/max(rhs1Area,rhs2Area));        
//        output->appendFeatures(features);
    }
    
    virtual bool setCost(LHS_Type* output, RHS_Type1* RHS1, RHS_Type2* RHS2, vector<Terminal*> & terminals) {
        // Initialize features.
        double cost=getMinusLogProbability(features);
        output->setAdditionalCost(cost/Params::doubleRuleDivide - - log(Params::objectCost));
        
        if(isinf(cost))
            return false;
        else
            return true;
    }
    
    virtual LHS_Type* applyRuleLearning(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        appendAllFeatures(LHS,RHS1,RHS2,terminals);
        writeFeaturesToFile();
        return LHS;
    }
    
    virtual LHS_Type* applyRuleInference(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        
        // below to be replaced by generic learned rules
        
#ifdef DISABLE_HALLUCINATION
        {
            appendAllFeatures(LHS, RHS1, RHS2, terminals);
            if (setCost(LHS, RHS1, RHS2, terminals))
            {
                return LHS;
            }
            else
            {
                 delete LHS;
               return NULL;
            }
        }
#else
        {
            features.clear();

            vector<Symbol*> RHS1Expanded;
            vector<Symbol*> RHS2Expanded;
            RHS1->expandIntermediates(RHS1Expanded);
            RHS2->expandIntermediates(RHS2Expanded);
            assert(RHS2Expanded.size() == 1); // from rule design pattern so far ... compex objects are in left

            int numNodes = RHS1Expanded.size();
            if (modelsForLHS.size() == 0)//only called the 1st time this rule is used
            {
                readPairModels(numNodes);
            }

            vector<Symbol*>::iterator it1;
            vector<Symbol*>::iterator it2;

            int count = 0;
            double cost = 0;
            for (it1 = RHS1Expanded.begin(); it1 != RHS1Expanded.end(); it1++)
            {
                for (it2 = RHS2Expanded.begin(); it2 != RHS2Expanded.end(); it2++)
                {
                    cost += PairInfo<float>::computeMinusLogProb((*it1)->grandChildIfHallucinated(), (*it2)->grandChildIfHallucinated(), modelsForLHS.at(count));
                    count++;
                }
            }
            
            if(isinf(cost))
            {
                delete LHS;
                return NULL;
            }
            cost=cost-log(Params::objectCost); 
            LHS->setAdditionalCost(cost);
            

        }
#endif
        return LHS;
        
    }
    
    virtual LHS_Type* applyRuleGeneric(Symbol * RHS1, Symbol * RHS2, vector<Terminal*> & terminals)
    {
        assert(typeid(*RHS1)==typeid(RHS_Type1));
        assert(typeid(*RHS2)==typeid(RHS_Type2));
        LHS_Type * LHS = new LHS_Type();
        LHS->addChild(RHS1);
        LHS->addChild(RHS2);
        LHS->computeSpannedTerminals();
        return LHS;
    }

    virtual void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {
//        if()
       // cout<<filename<<"combine called\n";
        combineAndPushGeneric (extractedSym, pqueue, terminals, iterationNo);
    }
}; 

typedef boost::shared_ptr<Rule> RulePtr;


class LabelSelector
{
public:
    set<int> targetLabels;
    LabelSelector(int label)
    {
        targetLabels.insert(label);
    }

    LabelSelector(){}

    void addAcceptedLabel(int label)
    {
        targetLabels.insert(label);        
    }
    
    LabelSelector(set<int> targetLabels)
    {
        this->targetLabels=targetLabels;
    }

    bool acceptLabel(int label)
    {
        return (targetLabels.find(label)!=targetLabels.end());
    }
};

void readLabelMap(char * filename , map<int,int> & label_mapping)
{
    std::ifstream labelFile;
    labelFile.open(filename);
    int segNo, label;
        if(labelFile.is_open())
    {
        while(labelFile.good())
        {
            label=-1;
            labelFile>>segNo>>label;
            if(label==-1)
                break;
            cout<<"adding seg "<<segNo<<" with label:"<<label<<endl;
            label_mapping[segNo]=label;
        }
    }
    else
    {
        cout<<"could not open label file...exiting\n";
        exit(-1);
    }
    
}
void getSegmentDistanceToBoundaryOptimized( pcl::PointCloud<PointT> &cloud , vector<Terminal *> & terminals, double *maxDist)
{
    //assuming the camera is in the centre of the room and Z is aligned vertical
    
    //bin points based on azimuthal angle 
//    vector<int> directionBins[360];
    getMaxRanges(maxDist,cloud);
    
    cerr<<"origin used for distance to boundary:\n"<< cloud.sensor_origin_;
    for(int i=0;i<(int)terminals.size();i++)
    {
        double distBoundary=0;
        double dist;
        Terminal* segment= terminals.at(i);
        for(int j=0;j<(int)segment->getNumPoints();j++)
        {
                pair<int,double>angDist= get2DAngleDegreesAndDistance(segment->getPoint(cloud,j),cloud.sensor_origin_);
                int angle=angDist.first;
                dist=maxDist[angle]-angDist.second;
                assert(dist>=0);
                distBoundary+=dist;
                segment->getPoint(cloud,j).distance=dist;
        }
        segment->setDistanceToBoundary(distBoundary/(segment->getNumPoints()-1));
    }
    
}
class RPlaneSeg : public Rule {
public:

    Plane * applyRule(Symbol * extractedSym)
    {
        Plane * LHS = new Plane();
        LHS->addChild(extractedSym);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        return LHS;
        
    }
    
    Plane * applyRuleLearning(Symbol * extractedSym)
    {
        Plane * LHS = new Plane();
        LHS->addChild(extractedSym);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndEigens(); // also computes feats
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        return LHS;
        
    }
    
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) != typeid (Terminal))
            return;
        
        addToPqueueIfNotDuplicate(applyRule(extractedSym),pqueue); //TODO: duplicate check is not required
    }
};

// Manual rules that we need.
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
        //TODO : FIRST COMPUTE PLANE PARAMS FROM JUST THIS AND THEN 
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        return LHS;
    }

    Plane * applyRuleLearning(Plane * RHS_plane, Terminal *RHS_seg) {
        
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndEigens(); // also computes feats
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        
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

class NTComplex : public NonTerminal
{
public:
    virtual NonTerminal * getBase()=0;
    
    virtual bool isPrunable()
    {
        return getCost()>Params::costPruningThresh;
    }
    
};

void NonTerminal::reFormatSubTree(NonTerminal * parent /* = 0 */)
{
        vector<Symbol*> childrenCopy=children; // children may change in this process so make a copy
    if ((isOfSubClass<NTComplex>()||isOfSubClass<Plane>()) && parent!=NULL && typeid(*this)==typeid(*parent))
    {
        for (vector<Symbol*>::iterator it=childrenCopy.begin();it!=childrenCopy.end();it++)
        {
            if(typeid(*this)==typeid(*(*it)))
            {
                (*it)->reFormatSubTree(parent);
            }
            else
            {
                (*it)->reFormatSubTree(this);
                parent->addChildNoCheck(*it);
            }
                
        }
        if(parent->getChild(0)==this)
        {
            parent->children.erase(parent->children.begin());
        }
    }
    else
    {
        for (vector<Symbol*>::iterator it=childrenCopy.begin();it!=childrenCopy.end();it++)
        {
            (*it)->reFormatSubTree(this);
        }
    }
}

template <typename Support>
class SupportComplex : public NTComplex
{
    bool baseHallucinated;
    Support * base;
public:
    SupportComplex()
    {
        baseHallucinated=false;
        base=NULL;
    }
    
    NonTerminal * getBase()
    {
        assert(!baseHallucinated);
        assert(base!=NULL);
        return base;
    }

    void setBaseHallucinated(bool baseHallucinated)
    {
        this->baseHallucinated = baseHallucinated;
        assert(base==NULL);
    }

    bool isBaseHallucinated() const
    {
        return baseHallucinated;
    }

    void setBase(Support* base)
    {
        this->base = base;
    }

    void copyBasePtrAndFlags(SupportComplex<Support>* other)
    {
        this->base = other->base;
        this->baseHallucinated=other->baseHallucinated;
    }
    
    vector<NonTerminal*> objectsOnTop;
    
};

/**
 * these rules have no feature based weights .. only ratios
 */
template<typename LHS_Type,typename RHS_Type>
class SingleRuleNoFeature : public SingleRule<LHS_Type,RHS_Type>
{
    float cost;
public:
    
    SingleRuleNoFeature(float ratio=1):SingleRule<LHS_Type,RHS_Type>("dummy")
    {
        this->cost=log(ratio);
        this->filename=string("SingleRuleChoice_")+typeid(LHS_Type).name()+"_"+typeid(RHS_Type).name(); // not used : only for tagging
    }
    
    virtual void additionalProcessing(LHS_Type * LHS, RHS_Type* RHS)
    {
        
    }
    
    virtual LHS_Type* applyRuleLearning(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS, terminals);
        
        computeFeatures(RHS);
        LHS->setAdditionalCost(this->cost);
        additionalProcessing(LHS,RHS);
        LHS->declareOptimal();
        
        return LHS;
    }
    
    virtual LHS_Type* applyRuleinference(RHS_Type* RHS, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS, terminals);
        
        // to be replace by generic features
        additionalProcessing(LHS,RHS);
        computeFeatures(RHS);
        LHS->setAdditionalCost(0);
        return LHS;
    }
    
};

template<typename SupportType>
class SingleRuleComplex : public SingleRuleNoFeature<SupportComplex<SupportType> ,SupportType>
{

    typedef SupportComplex<SupportType> LHS_Type;
    typedef SupportType RHS_Type;
    
public:
    
    
    SingleRuleComplex()
    {
        this->filename=string("SupportComplex_")+typeid(SupportType).name(); // not used : only for tagging
    }
    
    virtual void additionalProcessing(LHS_Type * LHS, RHS_Type* RHS)
    {
        LHS->setBase(RHS);        
    }
    
};

template<typename T>
class PairInfoSupportComplex
{
    const static int NUM_FEATS = 9;
public:

    union
    {
        // add new features here and update the static constants. make use to set them later
        T all[NUM_FEATS];

        struct
        {
            T centDist;
            T centDistHorz;
            T centZDiff12;
            T z1Min_2Max;
            T z1Max_2Min;
            T z1Min_2Min;
            T z1Max_2Max;
            T minDist;
            T frontness; // change NUM_FEATS above
        };
    };
    
    void computeInfo(Symbol * rhs1, Symbol * rhs2){};
    void writeToFile(ofstream & featureFile)
    {
        for(int i=0;i<NUM_FEATS;i++)
        {
            featureFile<<all[i]<<",";
        }
        featureFile<<endl;
        
    }
    
    void pushToVector(vector<T> & infos)
    {
        for(int i=0;i<NUM_FEATS;i++)
            infos.push_back(all[i]);
    }

};
float overallMinZ;

template<>
void PairInfoSupportComplex<float> :: computeInfo(Symbol * rhs1, Symbol * rhs2)
{
    centDist=(rhs1->centroidDistance(rhs2))*Params::featScale;
    centDistHorz=(rhs1->centroidHorizontalDistance(rhs2))*Params::featScale;
    centZDiff12=(rhs1->getCentroid().z - rhs2->getCentroid().z)*Params::featScale;

    z1Min_2Max=(rhs1->getMinZ() - rhs2->getMaxZ())*Params::featScale;
    z1Max_2Min=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;
    z1Min_2Min=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;
    z1Max_2Max=(rhs1->getMaxZ() - rhs2->getMinZ())*Params::featScale;

    minDist=(rhs1->getMinDistance(rhs2))*Params::featScale;
    frontness=rhs1->inFrontNessof(rhs2)*Params::featScale;
    
}

template<typename SupportType, typename RHS_Type2 >
class DoubleRuleComplex : public DoubleRule< SupportComplex<SupportType> , SupportComplex<SupportType> , RHS_Type2 >
{
    typedef pair<string,string> PairType;
    map<PairType, MultiVariateProbabilityDistribution * > pairWiseModels; // between a type inside RHS_Type1(key) and Rhs_Type2
    map<PairType, ofstream*  > pairWiseFeatFiles; // between a type inside RHS_Type1(key) and Rhs_Type2
    
    typedef SupportComplex<SupportType> LHS_Type;
    typedef SupportComplex<SupportType> RHS_Type1;
public:
    bool canBaseBeHallucinated();
    
    PairType makeSortedPair(string name1, string name2)
    {
        if(name1<name2)
            return PairType(name1,name2);
        else
            return PairType(name2,name1);
    }
    
   virtual string getFileName(PairType & types)
    {
        string out=string("rule_DSRP_") +string(typeid(LHS_Type).name())+"__";
        out.append(types.first+"_");
        out.append(types.second);
       // cerr<<"filename:"<<out<<endl;
        return out;
    }
    
    
    DoubleRuleComplex( vector<string> types, bool learning=false):DoubleRule< SupportComplex<SupportType> , SupportComplex<SupportType> , RHS_Type2 >(learning)
    {
       if (this->isLearned())
        {
            
            for (vector<string>::iterator it = types.begin(); it != types.end(); it++)
            {
                PairType typeSt=makeSortedPair(*it,string(typeid(RHS_Type2).name()));
                string fname = getFileName(typeSt);
                
                if (learning)
                {
                    ofstream * temp =new ofstream(fname.data(), ios::app); // append to file
                    pairWiseFeatFiles[typeSt]=temp;
                }
                else
                {
                    string fnamep=rulePath+"/"+fname+".model";
                    ifstream file(fnamep.data(), std::ios::in);
                    if(file.is_open())
                    {
                        pairWiseModels[typeSt]=new MultiVarGaussian(file);
                        file.close();
                    }

                }
            }
        }
    }
    
    virtual void appendMainFeats(RHS_Type1 * RHS1, RHS_Type2 * RHS2)
    {
        this->features.clear();
        double baseMaxZ;
        if(RHS1==NULL || RHS1->isBaseHallucinated())
        {
            baseMaxZ=0;
        }
        else
        {
            baseMaxZ=RHS1->getBase()->getMaxZ();
        }
        this->features.push_back((baseMaxZ - RHS2->getMinZ())*Params::featScale);
    }
    
    virtual LHS_Type* applyRuleGeneric(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = new LHS_Type();
        if(RHS1!=NULL)
        {
                LHS->addChild(RHS1);
                LHS->copyBasePtrAndFlags(RHS1);
                LHS->objectsOnTop=RHS1->objectsOnTop;
        }
        else
        {
            LHS->setBaseHallucinated(true);
        }
        
        LHS->addChild(RHS2);
        LHS->objectsOnTop.push_back(RHS2);
        appendMainFeats(RHS1,RHS2);
        LHS->computeSpannedTerminals();
        return LHS;
    }

        
    virtual LHS_Type* applyRuleLearning(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        assert(this->featureFile.is_open()); // you need to construct this rule with false for learning
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        this->writeFeaturesToFile();
        LHS->setAdditionalCost(0);
        LHS->declareOptimal();
        if (RHS1 != NULL)
        {
            for (vector<NonTerminal*>::iterator it = RHS1->objectsOnTop.begin(); it != RHS1->objectsOnTop.end(); it++)
            {
                PairType sortTypes;
                string name1 = string(typeid (*(*it)).name());
                string name2 = string(typeid (*RHS2).name());

                sortTypes = makeSortedPair(name1, name2);

                PairInfoSupportComplex<float> feats;

                if (name1 < name2)
                    feats.computeInfo((*it), RHS2);
                else
                    feats.computeInfo(RHS2, (*it));

                cout << name1 << "," << name2 << endl;
                ofstream * file = pairWiseFeatFiles[sortTypes];
                assert(file != NULL);
                feats.writeToFile(*file);


            }
        }
        return LHS;
    }
    
    virtual LHS_Type* applyRuleInference(RHS_Type1 * RHS1, RHS_Type2 * RHS2, vector<Terminal*> & terminals)
    {
        LHS_Type * LHS = applyRuleGeneric(RHS1,RHS2,terminals);
        
        
            appendMainFeats(RHS1, RHS2);
             if(!setCost(LHS, RHS1, RHS2, terminals)) // set main cost
             {
                delete LHS;
                return NULL;                 
             }

        if (RHS1 != NULL)
        {
            double additionalCost = 0;
            for (vector<NonTerminal*>::iterator it = RHS1->objectsOnTop.begin(); it != RHS1->objectsOnTop.end(); it++)
            {
                PairType sortTypes;
                string name1 = string(typeid (*(*it)).name());
                string name2 = string(typeid (*RHS2).name());

                sortTypes = makeSortedPair(name1, name2);

                MultiVariateProbabilityDistribution * model = pairWiseModels[sortTypes];
                if (model != NULL)
                {
                    PairInfoSupportComplex<float> feats;
                    if (name1 < name2)
                        feats.computeInfo((*it), RHS2);
                    else
                        feats.computeInfo(RHS2, (*it));


                    vector<float> featv;
                    feats.pushToVector(featv);
                    additionalCost += (model->minusLogProb(featv));
                }
                else
                {
                    additionalCost += Params::onTopPairDefaultOnModelMissing;
                }

            }

            LHS->setAdditionalCost(additionalCost / Params::onTopPairDivide - log(Params::objectCost));



            if (isinf(additionalCost))
            {
                delete LHS;
                return NULL;
            }
        }
         return LHS;
        
    }
    
    
    virtual void applyRuleLearningBaseOccluded(RHS_Type2 * RHS_extracted)
    {
        
    }
    
    virtual void combineAndPushForParam2(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */)
    {

        RHS_Type2 * RHS_extracted = dynamic_cast<RHS_Type2 *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL)
        {
            if ( nt->isOfSubClass<RHS_Type1>())
            {
                RHS_Type1 * RHS_combinee = dynamic_cast<RHS_Type1 *> (nt);
                addToPqueueIfNotDuplicate(applyRuleInference(RHS_combinee, RHS_extracted,terminals), pqueue);
            }
            nt = finder.nextEligibleNT();
        }
        
        //handle case when floor is occluded 
        if(canBaseBeHallucinated() && overallMinZ>Params::maxFloorHeight)
        {
            //SupportComplex<SupportType> * occFloor=new SupportComplex<SupportType>();
            //occFloor->setBaseHallucinated(true);
            LHS_Type *lhsOcc=applyRuleInference(NULL, RHS_extracted,terminals);
            lhsOcc->setAdditionalCost(Params::floorOcclusionPenalty);
            addToPqueueIfNotDuplicate(lhsOcc, pqueue);            
        }
        
    }
    
}; 


void appendRuleInstance(vector<RulePtr> & rules, RulePtr rule) {
    if(!rule->isModelFileMissing())
    {
        rules.push_back(rule);
        cout<<"added rule:"<<rule->filename<<endl;
    }
    else
    {
        cout<<"rejected rule:"<<rule->filename<<endl;        
    }
    
}

#endif
