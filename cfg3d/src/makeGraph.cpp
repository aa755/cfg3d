#include "segmentUtils.h"
#define COMPUTE_NEIGHBORS
#define DISTANCE_BASED_PLANE_MERGE_THRESHOLD
int** intArrPointer;

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

    void getNeighborSegs(size_t index, set<int> & segIndices , bool consideOcclusion=true)
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
            
        if(consideOcclusion)
        {
            nf.processNeighbor(nbr);
        }
            
            if(nbr.segment!=originSegment && nbr.segment!=0)
            {
              //  double dist=(pcl::euclideanDistance<PointOutT,PointOutT>(origin,nbr) );
                // assert( dist<= nearThresh );
                cout<<nbr.segment<<"added (within nearThresh)\n";
                segIndices.insert(nbr.segment);
            }            
        }
        
        if(!consideOcclusion)
            return;
            
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


//  void extractRansacClusters(  pcl::PointCloud<PointInT> &cloud, std::vector<pcl::PointIndices> &segments)
//  {
//    Extractor p;
//
//    p.initializeFromCloud(createStaticShared<pcl::PointCloud<PointInT> >(& cloud));
//    int i = 0;
//    while(i < 100 && p.compute_object(i,segments)) {
//        i++;
//    }
//
//  }

#define MIN_SEG_SIZE 10


Terminal mergeTerminals(Terminal& terminal1, Terminal& terminal2) {
    vector<int> terminal2PointIndices = terminal2.getPointIndices();
    vector<int>::iterator it;
    for (it = terminal2PointIndices.begin(); it != terminal2PointIndices.end(); it++) {
        terminal1.addPointIndex(*it);
    }
    return terminal1;
}

bool goodEnough(Terminal & terminal1, Terminal& terminal2, pcl::PointXYZ cameraOrigin, pcl::PointCloud<PointOutT> & scene) {
    
    terminal1.computeFeatures();
    terminal2.computeFeatures();
    
    Plane terminal1Plane;
    terminal1Plane.addChild(createStaticShared(&terminal1));
    terminal1Plane.addPointIndices(terminal1.getPointIndices());
    terminal1Plane.computeFeatures();
    terminal1Plane.computePlaneParams();
    
    Plane terminal2Plane;
    terminal2Plane.addChild(createStaticShared(&terminal2));
    terminal2Plane.addPointIndices(terminal2.getPointIndices());
    terminal2Plane.computeFeatures();
    terminal2Plane.computePlaneParams();
    
    double distanceThreshold;
    double parallelThreshold = .8;
    
    Plane basePlane;
    Terminal otherTerminal;
    if (terminal1.getNumPoints() > terminal2.getNumPoints()) {
        basePlane = terminal1Plane;
        otherTerminal = terminal2;
        
        pcl::PointXYZ p1Centroid;
        basePlane.getCentroid(p1Centroid);
        
        distanceThreshold = .0025;
#ifdef DISTANCE_BASED_PLANE_MERGE_THRESHOLD
        distanceThreshold = pointPointDistance(cameraOrigin, p1Centroid) * .0075 * fabs(basePlane.getPlaneNormal().dot(pointPointVector(cameraOrigin, p1Centroid)));
#endif
    } else {
        otherTerminal = terminal1;
        basePlane = terminal2Plane;
        
        pcl::PointXYZ p2Centroid;
        basePlane.getCentroid(p2Centroid);
        
        distanceThreshold = .0025;
#ifdef DISTANCE_BASED_PLANE_MERGE_THRESHOLD
        distanceThreshold = pointPointDistance(cameraOrigin, p2Centroid) * .0075 * fabs(basePlane.getPlaneNormal().dot(pointPointVector(cameraOrigin, p2Centroid)));
#endif
    }
    
     boost::shared_ptr <std::vector<int> > terminal1BoostPtr = terminal1.getPointIndicesBoostPtr();
     boost::shared_ptr <std::vector<int> > terminal2BoostPtr = terminal2.getPointIndicesBoostPtr();
    
    return getSmallestDistance(scene, terminal1BoostPtr, terminal2BoostPtr) < distanceThreshold && 
           basePlane.getCentroidProximity(otherTerminal) < distanceThreshold && 
            terminal1Plane.isParallelEnough(terminal2Plane, parallelThreshold);
    
//    return basePlane.getCentroidProximity(otherTerminal) < distanceThreshold && 
//            terminal1Plane.isParallelEnough(terminal2Plane, parallelThreshold);
}
/**
 * 
 * @param baseTerminal
 * @param terminalsToMerge
 * @return true if we merged baseTerminal with one of terminalsToMerge
 *      if true, baseTerminal grown, terminalsToMerge size - 1
 */
bool runThroughTerminalsToMerge(Terminal& baseTerminal, vector<Terminal>& terminalsToMerge, pcl::PointXYZ cameraOrigin,pcl::PointCloud<PointOutT> & scene) {
    vector<Terminal>::iterator it;
    for (it = terminalsToMerge.begin(); it != terminalsToMerge.end(); it++) {
        if(goodEnough(baseTerminal, *it, cameraOrigin,scene)) {
            mergeTerminals(baseTerminal, *it);
            terminalsToMerge.erase(it);
            return true;
        }
    }
    return false;
}

vector<Terminal> mergeTerminalsProcess(vector<Terminal> terminalsToMerge, pcl::PointXYZ cameraOrigin,pcl::PointCloud<PointOutT> & scene) {
    vector<Terminal> mergedTerminals;
    while(!terminalsToMerge.empty()) {
        Terminal baseTerminal = terminalsToMerge.at(0);
        terminalsToMerge.erase(terminalsToMerge.begin());
        while (runThroughTerminalsToMerge(baseTerminal, terminalsToMerge, cameraOrigin,scene)) {
        }
        mergedTerminals.push_back(baseTerminal);
    }
    return mergedTerminals;
}

vector<Terminal> getTerminalsFromClusters(std::vector<pcl::PointIndices> clusters) {
    vector<Terminal> unmergedTerminals;
    for (unsigned int i = 0; i < clusters.size(); i++)
    {
        Terminal terminal(i + 1);
//        cout<<"i = "<<i<<endl;
        for (unsigned int j = 0; j < clusters.at(i).indices.size(); j++) {
//            cout<<"\tj = "<<j<<endl;
            terminal.addPointIndex(clusters.at(i).indices.at(j));
        }
        unmergedTerminals.push_back(terminal);
    }
    return unmergedTerminals;
}

std::vector<pcl::PointIndices> clusterFromTerminals(vector<Terminal> terminals) {
    vector<Terminal>::iterator terminalIterator;
    std::vector<pcl::PointIndices> newClusters;
    for (terminalIterator = terminals.begin(); terminalIterator != terminals.end(); terminalIterator++) {
        Terminal terminal = *terminalIterator;
        vector<int> pointIndices = terminal.getPointIndices();
        
        pcl::PointIndices content;
        content.indices = pointIndices;
        
        newClusters.push_back(content);
    }
    return newClusters;
}

/**
 * reads an already segmented PCD .. useful for times when only neighborgraph has to be computed
 * @param filename
 * @param clusters
 * @param scene
 */
void segmentDummy(string filename, std::vector<pcl::PointIndices> & clusters, pcl::PointCloud<PointOutT> & scene)
{
    pcl::io::loadPCDFile<PointOutT > (filename, scene);
    int segId=1;
    int done=false;
    while(!done)
    {
        pcl::PointIndices segIndices;
        for(int i=0;i<(int)scene.size();i++)
        {
            if((int)scene.points[i].segment==segId)
                segIndices.indices.push_back(i);
        }
        if(segIndices.indices.size()>0)
        {
            clusters.push_back(segIndices);
            segId++;
        }
        else
            done=true;

    }
}

void segment(string filename, std::vector<pcl::PointIndices> & clusters, pcl::PointCloud<PointOutT> & scene)
{
        pcl::PointCloud<PointInT> cloud_temp;
        pcl::io::loadPCDFile<PointInT > (filename, cloud_temp);
        pcl::PointCloud<PointInT> cloud;
        cloud.points.reserve(cloud_temp.size());
        cloud.sensor_origin_=cloud_temp.sensor_origin_;
        //remove Nans
#ifdef FILTER_SEGMENTS_SEGMENTATION
      LabelSelector lselect(1);  
#endif
    for (size_t i = 0; i < cloud_temp.size(); i++)
    {
        if(isnan( cloud_temp.points[i].x))
            continue;
#ifdef FILTER_SEGMENTS_SEGMENTATION
        if(!lselect.acceptLabel(cloud_temp.points[i].label))
            continue;
#endif
        cloud.points.push_back(cloud_temp.points.at(i));
    }
        
        pcl::PointCloud<PointInT>::Ptr cloud_ptr = createStaticShared<pcl::PointCloud<PointInT> >(&cloud);


    int number_neighbours = 30;
    
        SegmentPCDGraph<PointInT> segmenter(0.05,0.04,number_neighbours,0.1,500);
        segmenter.segment(cloud,clusters);

    
    sort(clusters.begin(),clusters.end(),compareSegsDecreasing);
    
    std::cout << clusters.size() << "clusters found in pcd of size " << cloud.size() << std::endl;

    scene.points.resize(cloud.size());
    scene.sensor_origin_=cloud_temp.sensor_origin_;

    for (size_t i = 0; i < cloud.size(); i++)
    {
        scene.points[i].clone(cloud.points[i]);
        scene.points[i].segment=0;
    }

    // Scene initialized
    vector<Terminal> unmergedTerminals = getTerminalsFromClusters(clusters);
    cout<<"Unmerged Terminal Size = "<<unmergedTerminals.size()<<endl;
    pcl::PointXYZ cameraOrigin = pcl::PointXYZ(cloud.sensor_origin_[0], cloud.sensor_origin_[1], cloud.sensor_origin_[2]);
    //assert(cameraOrigin.z!=0);
    vector<Terminal> mergedTerminals = mergeTerminalsProcess(unmergedTerminals, cameraOrigin,scene);
    cout<<"Merged Terminal Size = "<<mergedTerminals.size()<<endl;
    clusters = clusterFromTerminals(mergedTerminals);
    cout<<"Cluster Size = "<<clusters.size()<<endl;
    int total = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        if(clusters[i].indices.size()<MIN_SEG_SIZE)
            continue;
        
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
        {
            scene.points[clusters[i].indices[j]].segment = i + 1;            
        }
        std::cout << "seg size " << clusters[i].indices.size() << std::endl;
        total += clusters[i].indices.size();
    }
   string toAppendToString = filename;//filename.substr(0, filename.length()-4);
   string segmentedPCD = toAppendToString.append("_segmented.pcd");
   
   pcl::io::savePCDFile<PointOutT>(segmentedPCD, scene,true);
   std::cout << total << std::endl;
    
}

int main(int argc, char** argv)
{
    cout<<"Starting..."<<endl;
   string filename=string(argv[1]);
    std::vector<pcl::PointIndices> clusters;
     pcl::PointCloud<PointOutT> scene;
     
//     segment(filename,clusters,scene);
       segmentDummy(filename,clusters,scene);
#ifdef COMPUTE_NEIGHBORS
    OccupancyMapAdv occupancy(scene);
#endif        
    std::ofstream logFile;
    
    string neighborMap = filename.append("_nbr.txt");
    logFile.open(neighborMap.data(),ios::out);
    set<int>::iterator sit;
    
    int tIndex;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        set<int> segNbrs;
        for (size_t j = 0; j < clusters[i].indices.size(); j+=3)
        {
            set<int> ptNbrs;
            tIndex=clusters[i].indices[j];
#ifdef COMPUTE_NEIGHBORS
            occupancy.getNeighborSegs(tIndex, ptNbrs, true );// consider occlusion
//            occupancy.getNeighborSegs(tIndex, ptNbrs , false ); // wont consider occlusion
            segNbrs.insert(ptNbrs.begin(),ptNbrs.end());
#endif
        }
        
        std::cout << "seg size " << clusters[i].indices.size() << std::endl;
        
        
            logFile<<i+1;
            for(sit=segNbrs.begin();sit!=segNbrs.end();sit++)
                logFile<<","<<*sit;
            logFile<<endl;
    }

    logFile.close();

}


