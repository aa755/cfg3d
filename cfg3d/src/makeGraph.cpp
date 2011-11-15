#include "OccupancyMap.h"
#include <iostream>
#include <fstream>
#include <vector>
#include<typeinfo>
//#include "point_struct.h"
#include "point_types.h"
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/io.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <boost/dynamic_bitset.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include "extract_planes.cpp"

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

    NeighborFinder(PointOutT p, int numAzimuthBins_ = 8, int numElevationBins_ = 4) : point(p.x, p.y, p.z)
    {
        numAzimuthBins = numAzimuthBins_;
        numElevationBins = numElevationBins_;
        azimuthBinSize = 360.0 / numAzimuthBins;
        elevationBinSize = 180.0 / numElevationBins;
        directions.resize(numAzimuthBins*numElevationBins, true);
        assert(azimuthBinSize==elevationBinSize);
        iterator=-1;
    }
    
    /**
     * 
     * @param direction
     * @return  f such that f*d is the rad that can fit in the conical beam at distance d
     */
    inline double getBallRadFactor()
    {
        assert(azimuthBinSize/2.0<=90.0);
        return sin(DEG2RAD(azimuthBinSize/2.0));
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

class OccupancyMapAdv : public OccupancyMap<PointOutT>
{
public:
        OccupancyMapAdv(pcl::PointCloud<PointOutT> & cloud, float resolution_ = 0.02) : OccupancyMap<PointOutT>(cloud, resolution_)
        {
            
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
    
    /**
     * @return : -1 if a free point is encountered ... the entire area is not occluded
     * else returns the index of the 1st point encountered in the beam of angular width binAngle with central direction as "direction" from "origin"
     */
    int castBeam(const Eigen::Vector3d & origin, const Eigen::Vector3d & direction, double SineOfBeamConeHalfANgle)
    {
        OccupancyState st;
                vector<int> indices;
                vector<float> distances;

                vector<int>::iterator it;
                
        for (float dis = nearThresh; dis < nearOccThresh; dis += step)
        {
            Eigen::Vector3d tPv=origin + dis * direction;
            st = getOccupancyState(tPv);
            if (st == OCCUPANCY_FREE)
            {
                /*
                 *should we check freeness of the entire beam ?
                 * probably not ... free space is not sparse ... points are 
                 */
                break;
            }
            else 
            {
        
        
                PointOutT pt=NeighborFinder::pointFromVector(tPv);
                if(nnFinder.radiusSearch(pt, dis*SineOfBeamConeHalfANgle ,indices , distances,std::numeric_limits<int>::max())==0)
                {
                    //cerr<<"no point found\n";
                    continue;
                }
                else
                {
                    for(it=indices.begin();it!=indices.end();it++)
                    {
                        if(cloudSeg->points.at(*it).segment!=0)
                            return *it;
                    }
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
        
        cout<<"\n\n\n\nseg"<<originSegment<<"numNearNbrs:"<<indices.size()<<endl;
        for(it=indices.begin();it!=indices.end();it++)
        {
            PointOutT &nbr =cloudSeg->points.at(*it);
            nf.processNeighbor(nbr);
            if(nbr.segment!=originSegment && nbr.segment!=0)
            {
              //  double dist=(pcl::euclideanDistance<PointOutT,PointOutT>(origin,nbr) );
                // assert( dist<= nearThresh );
                cout<<nbr.segment<<"added (within nearThresh)\n";
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
            //nbrIndex=castRay(originv,direction);
            nbrIndex=castBeam(originv,direction,nf.getBallRadFactor());
            if(nbrIndex>=0)
            {
                double distance=pcl::euclideanDistance<PointOutT,PointOutT>(origin,cloudSeg->points[nbrIndex]);
                Eigen::Vector3d dir2=NeighborFinder::vectorFromPoint(cloudSeg->points[nbrIndex])-originv;
                dir2.normalize();
                int freeCount=0;
                for(double dis=0;dis<distance;dis+=step)
                {
                    Eigen::Vector3d testPtV=originv+dis*dir2;
                    if(getOccupancyState(testPtV)==OCCUPANCY_FREE)
                    {
                        freeCount++;
                    }
                }                
                
                if(freeCount>2)
                    continue;
                    
                
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

template<typename PointET>
Eigen::Vector3f getPoint(PointET p)
{
    Eigen::Vector3f temp;
    for(int i=0;i<3;i++)
        temp(i)=p.data[i];
    
    return temp;
}

  template <typename PointT, typename Normal> 
  void extractEuclideanClustersM( const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals, \
      float tolerance, const boost::shared_ptr<pcl::KdTree<PointT> > &tree, \
      std::vector<pcl::PointIndices> &clusters, double eps_angle, \
      unsigned int min_pts_per_cluster = 1, \
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      ROS_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!", cloud.points.size (), normals.points.size ());
      return;
    }

    Eigen::Vector3f origin;
    for(int i=0;i<3;i++)
        origin(i)=cloud.sensor_origin_(i);
    
    cout<<"origin:"<<endl;
    cout<<origin<<endl;
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (processed[i])
        continue;

      std::vector<unsigned int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (i);

      processed[i] = true;

      while (sq_idx < (int)seed_queue.size ())
      {
          Eigen::Vector3f pc=getPoint(cloud.points.at(seed_queue[sq_idx]));
          Eigen::Vector3f dir=(pc-origin);
                  
          //cout<<"direction:"<<dir<<endl<<"dirnorm"<<dir.norm()<<endl;
          float rad=sqr(dir.norm())*0.0075+tolerance;
          //cout<<"rad:"<<rad<<endl;
        // Search for sq_idx
        if (!tree->radiusSearch (seed_queue[sq_idx],rad , nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                         // Has this point been processed before ?
            continue;


          
         // processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                         normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                         normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle /* TODO: check color*/)
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }

        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
      {
        pcl::PointIndices r;
        r.indices.resize (seed_queue.size ());
        for (size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);   // We could avoid a copy by working directly in the vector
      }
    }
  }
  
  void extractRansacClusters(  pcl::PointCloud<PointInT> &cloud, std::vector<pcl::PointIndices> &segments)
  {
    Extractor p;

    p.initializeFromCloud(createStaticShared<pcl::PointCloud<PointInT> >(& cloud));
    int i = 0;
    while(i < 100 && p.compute_object(i,segments)) {
        i++;
    }

  }

#define MIN_SEG_SIZE 50

bool compareSegsDecreasing(const pcl::PointIndices & seg1,const pcl::PointIndices & seg2) 
{
    return (seg1.indices.size()> seg2.indices.size());
}

int main(int argc, char** argv)
{

        pcl::PointCloud<PointOutT> cloud_seg;
        pcl::PointCloud<PointInT> cloud_temp;
        pcl::io::loadPCDFile<PointInT > (argv[1], cloud_temp);
        pcl::PointCloud<PointInT> cloud;
        cloud.points.reserve(cloud_temp.size());
        cloud.sensor_origin_=cloud_temp.sensor_origin_;
        //remove Nans
    for (size_t i = 0; i < cloud_temp.size(); i++)
    {
        if(isnan( cloud_temp.points[i].x))
            continue;
        
        cloud.points.push_back(cloud_temp.points.at(i));
    }
        
        pcl::PointCloud<PointInT>::Ptr cloud_ptr = createStaticShared<pcl::PointCloud<PointInT> >(&cloud);


    int number_neighbours = 200;
    float radius =  0.04;
    float angle = 0.52;
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
 //   pcl::extractEuclideanClusters<PointInT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,300);
    
    extractEuclideanClustersM<PointInT, pcl::Normal > (cloud, cloud_normals, radius, clusters_tree_, clusters, angle,500);
   // extractRansacClusters(cloud, clusters);
    
    sort(clusters.begin(),clusters.end(),compareSegsDecreasing);
    
    std::cout << clusters.size() << "clusters found in pcd of size " << cloud.size() << std::endl;

cloud_seg.points.resize(cloud.size());
cloud_seg.sensor_origin_=cloud_temp.sensor_origin_;

    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloud_seg.points[i].clone(cloud.points[i]);
        cloud_seg.points[i].segment=0;

    }


    int total = 0;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        if(clusters[i].indices.size()<MIN_SEG_SIZE)
            continue;
        
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
        {
            cloud_seg.points[clusters[i].indices[j]].segment = i + 1;
        }
        std::cout << "seg size " << clusters[i].indices.size() << std::endl;
        total += clusters[i].indices.size();
    }
    
   pcl::io::savePCDFile<PointOutT>("segmented_"+std::string(argv[1]), cloud_seg,true);

   std::cout << total << std::endl;
   exit(1);

    OccupancyMapAdv occupancy(cloud_seg);

        
    std::ofstream logFile;
    logFile.open((std::string(argv[1])+".nbrMap.txt").data(),ios::out);
    set<int>::iterator sit;

    int tIndex;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        set<int> segNbrs;
        for (size_t j = 0; j < clusters[i].indices.size(); j+=3)
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


