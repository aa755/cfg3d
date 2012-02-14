/******************************************************************************
 * RSS 2011 - PCL Tutorial                                                    *
 * Session 04 - Segmentation                                                  *
 * Nico Blodow                                                                *
 *****************************************************************************/

// some basic includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// specific includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// let's make it easier on ourselves..
using namespace std;
using namespace pcl;
using namespace pcl::io;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

// compute a planar model
void sample1 (const PointCloud<PointXYZRGB>::Ptr & input)
{
  // Create a shared plane model pointer directly
  SampleConsensusModelPlane<PointXYZRGB>::Ptr model (new SampleConsensusModelPlane<PointXYZRGB> (input));
  
  // Create the RANSAC object
  RandomSampleConsensus<PointXYZRGB> sac (model, 0.03);

  // perform the segmenation step
  bool result = sac.computeModel ();

  // check for success
  if (!result)
  {
    cerr<<"Couldn't compute model \n";
  }
  else
  {
    // get the initial sample points - just for fun
    std::vector<int> sample;
    sac.getModel (sample);

    // get inlier indices
    boost::shared_ptr<vector<int> > inliers (new vector<int>);
    sac.getInliers (*inliers);
    cout << "Found model with " << inliers->size () << " inliers";

    // get model coefficients
    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    cout << ", plane normal is: " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << "." << endl;
    
    RGBValue color;
    color.Red=0;
    color.Green=255;
    color.Blue=0;
    color.Alpha=255;
    
    for(std::vector<int>::iterator it=inliers->begin(); it!=inliers->end();it++)
    {
        input->points[*it].rgb=color.float_value;
    }
    
        pcl::io::savePCDFile("plane.pcd",*input,true);
        exit(-1);

    // perform a refitting step
//    Eigen::VectorXf coeff_refined;
//    model->optimizeModelCoefficients (*inliers, coeff, coeff_refined);
//    model->selectWithinDistance (coeff_refined, 0.03, *inliers);
//    cout << "After refitting, model contains " << inliers->size () << " inliers";
//    cout << ", plane normal is: " << coeff_refined[0] << ", " << coeff_refined[1] << ", " << coeff_refined[2] << "." << endl;
//
//    // Projection tests
//    PointCloud<PointXYZRGB> proj_points;
//    model->projectPoints (*inliers, coeff_refined, proj_points);
  }
}

