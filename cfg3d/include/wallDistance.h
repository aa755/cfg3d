/* 
 * File:   wallDistance.h
 * Author: aa755
 *
 * Created on June 20, 2011, 6:18 PM
 */

#ifndef WALLDISTANCE_H
#define	WALLDISTANCE_H

class Point2DGeo
{
public:
  double x;
  double y;
  Point2DGeo(double x_,double y_)
  {
    x=x_;
    y=y_;
  }
  Point2DGeo()
  {
  }
  
  Point2DGeo subtract(Point2DGeo rhs)
  {
      return Point2DGeo(x-rhs.x,y-rhs.y);
  }
 
  double toDegress(double rad)
  {
  //    cerr<<"PI is"<<PI<<endl;
      return rad*180.0/boost::math::constants::pi<double>();
  }
  
  double angle()
  {
      double principalAngle=toDegress(atan(y/x));
  //    cout<<"y:"<<y<<"x:"<<x<<"angle:"<<principalAngle<<endl;
      double fullAngle;
      if(x>=0)
          fullAngle= principalAngle;
      else
          fullAngle= principalAngle+180.0;
      
      if(fullAngle<0.0)
          fullAngle+=360.0;
    //  cout<<"fa:"<<fullAngle<<endl;
      assert(fullAngle>=0.0);
      assert(fullAngle<360.0);
      return fullAngle;
  }
  
  const Point2DGeo  operator-(const Point2DGeo & other)
  {
      return Point2DGeo(x-other.x,y-other.y);
  }
  
  double norm()
  {
      return sqrt(x*x+y*y);
  }
  
  int principleRange(int angle)
  {
      return (angle % 360);
  }
  
  vector<int>  getAdjacentAngles(int angle,int ticks)
  {
      vector<int> ret;
      
      for(int i=0;i<ticks;i++)
          ret.push_back(principleRange(angle+i));
      
      for(int i=1;i<ticks;i++) // 0 is already covered => so starting from 1
          ret.push_back(principleRange(angle-i));
      
      return ret;
  }
};

template <typename PointType>
pair<int,double> get2DAngleDegreesAndDistance(const PointType & point,Eigen::Vector4f origin)
{
    Point2DGeo point2D(point.x,point.y);
//    VectorG origin=camera.getOrigin();
    Point2DGeo origin2D(origin(0),origin(1));
    Point2DGeo ray2D=point2D-origin2D;
    return pair<int,double>((int)ray2D.angle(),ray2D.norm());
}

void getMaxRanges(double * maxDist, const pcl::PointCloud<PointT> &cloud)
{
    for(int angle=0;angle<360;angle++)
    {
        maxDist[angle]=0;            
    }
    
    //get the max distance to boundary along each direction
    for(int i=1;i<(int)cloud.size();i++)
    {
        if(isnan( cloud.points[i].x))
            continue;
        pair<int,double>angDist= get2DAngleDegreesAndDistance(cloud.points[i],cloud.sensor_origin_/*,camera*/);
        int angle=angDist.first;
        double distance=angDist.second;
//        directionBins[angle].push_back(i);
        if(maxDist[angle]<distance)
            maxDist[angle]=distance;
    }
    
}

//double getWallDistanceCent(double * maxDist,PointT  p)
//{
//                pair<int,double>angDist= get2DAngleDegreesAndDistance(p/*,camera*/);
//                int angle=angDist.first;
//                double dist=maxDist[angle]-angDist.second;
//                //assert(dist>=0);
//                return dist;
//    
//}


//void add_distance_features(pcl::PointCloud<PointT> &cloud, map< int,vector<float> >&features,std::vector<pcl::PointCloud<PointT> > & segment_clouds){
//    bool outputDistancePCDSegmentWise=false;
//    bool outputDistancePCDPointWise=false;
//    map<int,float> segment_boundary_distance;
//    getSegmentDistanceToBoundaryOptimized(cloud,segment_boundary_distance,segment_clouds);
//    for(map<int,float>::iterator it = segment_boundary_distance.begin(); it != segment_boundary_distance.end(); it++ )
//    {
//        int segid = (*it).first;
//        features[segid].push_back((*it).second);
//    }
//        pcl::PCDWriter writer;
//    if (outputDistancePCDSegmentWise)
//    {
//        pcl::PointCloud<PointT> temp;
//            temp.header = cloud.header;
//            for (int i = 0; i < (int)cloud.size(); i++)
//            {
//                if (segment_boundary_distance.find(cloud.points[i].segment) != segment_boundary_distance.end())
//                {
//                    PointT tp = cloud.points[i];
//
//                    tp.distance = segment_boundary_distance[cloud.points[i].segment];
//                    temp.points.push_back(tp);
//                }
//            }
//            writer.write<pcl::PointXYZRGBCamSL > ("data_scene_distance_seg.pcd", temp, true);
//    }
//    if (outputDistancePCDPointWise)
//    {
//        pcl::PointCloud<PointT> temp;
//            temp.header = cloud.header;
//            
//            for (int i = 0; i < (int)segment_clouds.size(); i++)
//            {
//                temp+=segment_clouds[i];
//            }
//            
//        
//            writer.write<pcl::PointXYZRGBCamSL > ("data_scene_distance_point.pcd", temp, true);
//    }
//}



#endif	/* WALLDISTANCE_H */

