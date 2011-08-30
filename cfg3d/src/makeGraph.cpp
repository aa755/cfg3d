#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
//#include "point_struct.h"
#include "point_types.h"
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include "utils.h"
#include <pcl/segmentation/extract_clusters.h>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/dynamic_bitset.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/algorithms/transform.hpp>

using namespace std;

typedef pcl::PointXYZRGBCamSL PointOutT;
typedef pcl::PointXYZRGB PointInT;

void getComplementPointSet(vector<int>& memberIndices, vector<int>& complementIndices, pcl::PointCloud<PointInT> & cloud_seg)
{
    int it1 = 0;
    int numPoints = cloud_seg.size();
    complementIndices.clear();
    sort(memberIndices.begin(), memberIndices.end());
    vector<int>::iterator it2 = memberIndices.begin();
    while ((it1 < numPoints) && (it2 != memberIndices.end()))
    {
        if (it1 < *it2)
        {
            complementIndices.push_back(it1);
            it1++;
        }
        else if (it1 == *it2)
        {
            ++it1; //skip this point
            ++it2;
        }
        else
        {
            assert(1 == 2);
        }


    }

    while (it1 < numPoints)
    {
        complementIndices.push_back(it1);
        it1++;
    }
}

void getComplementSearchTree(pcl::KdTreeFLANN<PointInT> &nnFinder, vector<int>& memberIndices, pcl::PointCloud<PointInT> & cloud_seg)
{

    vector<int> complementIndices;

    assert(cloud_seg.size() != memberIndices.size()); // else it will crash in setInputCloud

    complementIndices.reserve(cloud_seg.size() - memberIndices.size()); // +1 is not required

    getComplementPointSet(memberIndices, complementIndices, cloud_seg);

    nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointInT> >(&cloud_seg), createStaticShared<vector<int> >(&complementIndices));
}

class NeighborFinder
{
    /** $i^th$ bit is false if no neighbor was found in that direction
     */
    boost::dynamic_bitset<> directions; 
    int numAzimuthBins;
    int numElevationBins;
    double azimuthBinSize;
    double elevationBinSize;
    Eigen::Vector3d point;
    int iterator;
public:

    NeighborFinder(PointOutT p, int numAzimuthBins_ = 8, int numElevationBins_ = 8) : point(p.x, p.y, p.z)
    {
        numAzimuthBins = numAzimuthBins_;
        numElevationBins = numElevationBins_;
        azimuthBinSize = 360.0 / numAzimuthBins;
        elevationBinSize = 180.0 / numElevationBins;
        directions.resize(numAzimuthBins*numElevationBins, true);
        iterator=-1;
    }

    int getIndex(double azimuth, double elevation)
    {
        //check that values lie in principle domain
        assert(azimuth>-180.0);
        assert(azimuth <= 180.0);
        assert(elevation >= 0.0);
        assert(elevation < 180.0);

        if (azimuth < 0)
            azimuth += 360.0;


        int azimuthBin = (int) (azimuth / azimuthBinSize);
        int elevationBin = (int) (elevation / elevationBinSize);
        assert(elevationBin <= numElevationBins);
        assert(azimuthBin < numAzimuthBins);

        return azimuthBin * numElevationBins + elevationBin;
    }

    int getIndex(Eigen::Vector3d & direction)
    {
        direction.normalize();
        //check that values lie in principle domain
        namespace bg = boost::geometry;
        typedef bg::point<double, 3, bg::cs::cartesian> cartesian;
        typedef bg::point<double, 2, bg::cs::spherical<bg::degree> > spherical;
        //  ros::init(argc, argv,"segmenterAndGraphMaker");

        cartesian p3(direction(0), direction(1), direction(2));
        spherical p1;
     //   cout << "original cart:" << bg::dsv(p3) << endl;
        bg::transform<cartesian, spherical > (p3, p1);
       // cout << "converted sph:" << bg::dsv(p1) << endl;
        return getIndex(p1.get < 0 > (), p1.get < 1 > ());
    }

    static Eigen::Vector3d vectorFromPoint(const PointOutT & p)
    {
        return Eigen::Vector3d(p.x, p.y, p.z);
    }

    static PointOutT pointFromVector(const Eigen::Vector3d & d)
    {
        PointOutT p;
        for(int i=0;i<3;i++)
            p.data[i]=d(i);
        p.data[3]=1.0f;
        return p;
        
    }
    
    void processNeighbor(PointOutT n_)
    {
        Eigen::Vector3d n = vectorFromPoint(n_);
        Eigen::Vector3d direction = n - point;
        
        if(direction.norm()==0)
            return;
        
        directions.set(getIndex(direction), false);
    }

    Eigen::Vector3d directionFromIndex(int index)
    {
        int azimuthBin = index / numElevationBins;
        int elevationBin = index % numElevationBins;

        double azimuth = (azimuthBin * azimuthBinSize);
        double elevation = (elevationBin * elevationBinSize);

        namespace bg = boost::geometry;
        typedef bg::point<double, 3, bg::cs::cartesian> cartesian;
        typedef bg::point<double, 2, bg::cs::spherical<bg::degree> > spherical;
        //  ros::init(argc, argv,"segmenterAndGraphMaker");

        cartesian p3;
        spherical p1(azimuth, elevation);

        bg::transform<spherical, cartesian > (p1, p3);
        cout << bg::dsv(p3) << endl;
        //        assert(1 == 2); // check the direction2index(index2direction(x))=x
        return Eigen::Vector3d(p3.get < 0 > (), p3.get < 1 > (), p3.get < 2 > ());
    }
    
    bool iterator_get_next_direction(Eigen::Vector3d & direction)
    {
        
        if(iterator==-1)
        {
            iterator= directions.find_first();
        }
        else
            iterator= directions.find_next(iterator);
        
        //http://cplusplus.syntaxerrors.info/index.php?title=Warning:_comparison_between_signed_and_unsigned_integer_expressions
        if(iterator==(int)boost::dynamic_bitset<>::npos)
        {
            iterator=directions.size(); // at the end, so that next time also npos is returned
            return false;
        }
        else
        {
            direction=directionFromIndex(iterator);
            return true;
        }
    }
    
    void iterator_reset()
    {
        iterator=-1;
    }

};

class OccupancyMap
{
    float resolution;
    octomap::OcTreeROS tree;
    pcl::PointCloud<pcl::PointXYZ> xyzcloud;
    pcl::KdTreeFLANN<PointOutT> nnFinder;
    pcl::PointCloud<PointOutT> *cloudSeg;
    static const float nearThresh=0.10;
    static const float nearOccThresh=2.0;

public:

    pcl::PointXYZ convertFromVector(Eigen::Vector4f p)
    {
        return pcl::PointXYZ(p(0), p(1), p(2));
    }

    static void convertToXYZ(const pcl::PointCloud<PointOutT> &cloud, pcl::PointCloud<pcl::PointXYZ> & cloudxyz)
    {
        cloudxyz.points.resize(cloud.size());
        for (size_t i = 0; i < cloud.size(); i++)
        {
            cloudxyz.points[i].x = cloud.points[i].x;
            cloudxyz.points[i].y = cloud.points[i].y;
            cloudxyz.points[i].z = cloud.points[i].z;
        }
    }

    enum OccupancyState
    {
        OCCUPANCY_OCCUPIED = 1, OCCUPANCY_FREE = 2, OCCUPANCY_OUT_OF_RANGE = 3
    };

    OccupancyMap(pcl::PointCloud<PointOutT> & cloud, float resolution_ = 0.02) : tree(resolution_)
    {
        cloudSeg=& cloud;
        resolution = resolution_;
        convertToXYZ(cloud, xyzcloud);
        tree.insertScan(xyzcloud, convertFromVector(cloud.sensor_origin_), -1, true);
        //http://www.ros.org/doc/api/octomap_ros/html/classoctomap_1_1OctomapROS.html
        
        nnFinder.setInputCloud(createStaticShared<pcl::PointCloud<PointOutT> >(&cloud));

    }

    OccupancyState getOccupancyState(Eigen::Vector3d ptv)
    {
        pcl::PointXYZ pt;
        for(int i=0;i<3;i++)
            pt.data[i]=ptv(i);
        pt.data[3]=1.0f;
        return getOccupancyState(pt);
    }
    
    OccupancyState getOccupancyState(const pcl::PointXYZ pt)
    {
        octomap::OcTreeROS::NodeType * treeNode;
        treeNode = tree.search(pt);

        if (treeNode == NULL)
        {
           // cout << "null returned" << endl;
            return OCCUPANCY_OUT_OF_RANGE;
        }
        double occupancy = treeNode->getOccupancy();
        cout << "getOcc:" << occupancy << endl;
        if (occupancy >= 0.5)
            return OCCUPANCY_OCCUPIED;
        else
            return OCCUPANCY_FREE;
    }
    
    bool isFree(const pcl::PointXYZ pt)
    {
        octomap::OcTreeROS::NodeType * treeNode;
        treeNode = tree.search(pt);

        if (treeNode == NULL)
            return false;
        double occupancy = treeNode->getOccupancy();
        if (occupancy >= 0.5)
            return false;
        else
            return true;
        
    }

    OccupancyState getOccupancyState(size_t index)
    {
        return getOccupancyState(xyzcloud.points[index]);
    }

    void printPoint(PointOutT p,string msg="")
    {
        cout<<msg<<":("<<p.x<<","<<p.y<<","<<p.z<<","<<")"<<endl;
    }

    /**
     * @return : -1 if a free point is encountered ... the entire area is not occluded
     * else returns the index of the 1st point encountered in the direction "direction" from "origin"
     */
    int castRay(const Eigen::Vector3d & origin, const Eigen::Vector3d & direction)
    {
        const float step = 0.005;
        OccupancyState st;
        vector<float> nearest_distances;
        vector<int> nearest_indices;

        nearest_indices.resize(1, 0);
        nearest_distances.resize(1, 0.0);

        for (float dis = nearThresh*4.0/3.0; dis < nearOccThresh; dis += step)
        {
            Eigen::Vector3d tPv=origin + dis * direction;
            st = getOccupancyState(tPv);
            if (st == OCCUPANCY_FREE)
            {
                break;
            }
            else if (st == OCCUPANCY_OCCUPIED)
            {
                PointOutT tpt=NeighborFinder::pointFromVector(tPv);
                nnFinder.nearestKSearch(tpt, 1, nearest_indices, nearest_distances);
                if (nearest_distances.at(0) > 2 * resolution)
                {
                    cout<<"Warn: this point was expected to be occupied\n";
                    continue;
                }
                else if(cloudSeg->points.at(nearest_indices[0]).segment==0)
                {
                    cout<<"Info: occlusion search reached an unlabeled point\n";
                    continue;
                    
                }
                else
                {
                   return nearest_distances.at(0);
                }
            }
        }
        
        
        return -1;

    }
    
    void getNeighborSegs(size_t index, set<int> & segIndices )
    {
        PointOutT &origin =cloudSeg->points.at(index);
        uint32_t originSegment=origin.segment;
                
        NeighborFinder nf(origin);
        vector<int> indices;
        vector<float> distances;
        
        segIndices.clear();
        
        nnFinder.radiusSearch(index,nearThresh,indices,distances,std::numeric_limits<int>::max());
        vector<int>::iterator it;
        
        cout<<"\n\n\n\nnumNearNbrs:"<<indices.size()<<endl;
        for(it=indices.begin();it!=indices.end();it++)
        {
            PointOutT &nbr =cloudSeg->points.at(*it);
            nf.processNeighbor(nbr);
            if(nbr.segment!=originSegment && nbr.segment!=0)
            {
                segIndices.insert(nbr.segment);
            }            
        }
        
        Eigen::Vector3d originv=NeighborFinder::vectorFromPoint(origin);
        Eigen::Vector3d direction;
        nf.iterator_reset();
        vector<float> nearest_distances;
        vector<int> nearest_indices;
        
        nearest_indices.resize(1,0);
        nearest_distances.resize(1,0.0);
        
        int nbrIndex;
        uint32_t nbrSeg;
        int count=0;
        while(nf.iterator_get_next_direction(direction))
        {
            count++;
            nbrIndex=castRay(originv,direction);
            if(nbrIndex>=0)
            {
                nbrSeg=cloudSeg->points.at(nbrIndex).segment;
                
                if(nbrSeg!=originSegment)
                        segIndices.insert(nbrSeg);
            }

        }
        
        cout<<count<<"directions were vacant\n";
            cout<<"i"<<index<<":";
            set<int>::iterator sit;
            for(sit=segIndices.begin();sit!=segIndices.end();sit++)
                cout<<*sit<<",";
            cout<<endl;
        
    }
};



int main(int argc, char** argv)
{

        pcl::PointCloud<PointOutT> cloud_seg;
        pcl::PointCloud<PointInT> cloud_temp;
        pcl::io::loadPCDFile<PointInT > (argv[1], cloud_temp);
        pcl::PointCloud<PointInT> cloud;
        cloud.points.reserve(cloud_temp.size());
        
        //remove Nans
    for (size_t i = 0; i < cloud_temp.size(); i++)
    {
        if(isnan( cloud_temp.points[i].x))
            continue;
        
        cloud.points.push_back(cloud_temp.points.at(i));
    }
        
        pcl::PointCloud<PointInT>::Ptr cloud_ptr = createStaticShared<pcl::PointCloud<PointInT> >(&cloud);


    int number_neighbours = 100;
    float radius = 0.05; // 0.025
    float angle = 0.32;
    pcl::KdTree<PointInT>::Ptr normals_tree_, clusters_tree_;
    pcl::NormalEstimation<PointInT, pcl::Normal> n3d_;
    std::vector<pcl::PointIndices> clusters;


    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointInT> > ();
    initTree(0, clusters_tree_);
    clusters_tree_->setInputCloud(cloud_ptr);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointInT> > ();
    n3d_.setKSearch(number_neighbours);
    //    n3d_.setSearchMethod (normals_tree_);
    n3d_.setSearchMethod(clusters_tree_);

    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(cloud_normals);

    //pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = createStaticShared<const pcl::PointCloud<pcl::Normal> > (& cloud_normals);
    pcl::extractEuclideanClusters<PointInT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle);
    std::cout << clusters.size() << "clusters found in pcd of size " << cloud.size() << std::endl;

cloud_seg.points.resize(cloud.size());

    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloud_seg.points[i].clone(cloud.points[i]);

    }


    int total = 0;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
        {
            cloud_seg.points[clusters[i].indices[j]].segment = i + 1;
        }
        std::cout << "seg size " << clusters[i].indices.size() << std::endl;
        total += clusters[i].indices.size();
    }
    
   pcl::io::savePCDFile<PointOutT>("segmented_"+std::string(argv[1]), cloud_seg);

   std::cout << total << std::endl;

    OccupancyMap occupancy(cloud_seg);

        
    std::ofstream logFile;
    logFile.open((std::string(argv[1])+"nbrMap.txt").data(),ios::out);
    set<int>::iterator sit;

    int tIndex;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        set<int> segNbrs;
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
        {
            set<int> ptNbrs;
            tIndex=clusters[i].indices[j];
            occupancy.getNeighborSegs(tIndex, ptNbrs );
            segNbrs.insert(ptNbrs.begin(),ptNbrs.end());
        }
        
        std::cout << "seg size " << clusters[i].indices.size() << std::endl;
        
        total += clusters[i].indices.size();
        
            logFile<<i+1;
            for(sit=segNbrs.begin();sit!=segNbrs.end();sit++)
                logFile<<","<<*sit;
            logFile<<endl;

    }

    logFile.close();

}


