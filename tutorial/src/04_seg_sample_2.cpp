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

// includes for visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include "color_handler_uniform.h"

// let's make it easier on ourselves..
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

// compute a planar model
void sample1 (const PointCloud<PointXYZ>::ConstPtr & input)
{
  // Create a visualizer
  PCLVisualizer vis ("RSS 2011 PCL Tutorial - 04 Segmentation");

  // Create a shared plane model pointer directly
  SampleConsensusModelPlane<PointXYZ>::Ptr model (new SampleConsensusModelPlane<PointXYZ> (input));
  
  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

  // perform the segmenation step
  bool result = sac.computeModel ();

  // check for success
  if (!result)
  {
    PCL_ERROR ("Couldn't compute model \n");
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

    // Create the filtering object
    ExtractIndices<PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud (input);
    extract.setIndices (inliers);
    extract.setNegative (false);
    PointCloud<PointXYZ>::Ptr subcloud (new PointCloud<PointXYZ>);
    extract.filter (*subcloud);

    vis.addPointCloud<PointXYZ> (input, WhiteCloudHandler (input), "cloud");
    vis.addPointCloud<PointXYZ> (subcloud, RedCloudHandler (input), "inliers");
    
    vis.spin ();

    // perform a refitting step
    Eigen::VectorXf coeff_refined;
    model->optimizeModelCoefficients (*inliers, coeff, coeff_refined);
    model->selectWithinDistance (coeff_refined, 0.03, *inliers);
    cout << "After refitting, model contains " << inliers->size () << " inliers";
    cout << ", plane normal is: " << coeff_refined[0] << ", " << coeff_refined[1] << ", " << coeff_refined[2] << "." << endl;

    vis.removePointCloud ("inliers");
    extract.setIndices (inliers);
    extract.filter (*subcloud);
    vis.addPointCloud<PointXYZ> (subcloud, BlueCloudHandler (input), "inliers");
    vis.spin ();

    // Projection tests
    PointCloud<PointXYZ> proj_points;
    model->projectPoints (*inliers, coeff_refined, proj_points);
  }
}
