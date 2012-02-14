/******************************************************************************
 * RSS 2011 - PCL Tutorial                                                    *
 * Session 04 - Segmentation                                                  *
 * Nico Blodow                                                                *
 *****************************************************************************/

// some basic includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

// specific includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// segmentation includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include <pcl/surface/convex_hull.h>

// includes for visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <color_handler_uniform.h>

// let's make it easier on ourselves..
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

// compute a planar model
void sample3 (const PointCloud<PointXYZ>::ConstPtr & input)
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
    // get inlier indices
    boost::shared_ptr<vector<int> > inliers (new vector<int>);
    sac.getInliers (*inliers);
    cout << "Found model with " << inliers->size () << " inliers";

    // get model coefficients
    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    cout << ", plane normal is: " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << "." << endl;

    // Project the planar points
    PointCloud<PointXYZ>::Ptr proj_points (new PointCloud<PointXYZ>);
    model->projectPoints (*inliers, coeff, *proj_points);
    
    // Create the filtering object
    ExtractIndices<PointXYZ> extract;

    // Extract the outliers
    extract.setInputCloud (proj_points);
    extract.setIndices (inliers);
    extract.setNegative (true);
    PointCloud<PointXYZ>::Ptr outliers (new PointCloud<PointXYZ>);
    extract.filter (*outliers);

    // and extract the inliers
    extract.setNegative (false);
    PointCloud<PointXYZ>::Ptr inliers_cloud (new PointCloud<PointXYZ>);
    extract.filter (*inliers_cloud);
    
    vis.addPointCloud<PointXYZ> (outliers, WhiteCloudHandler (outliers), "outliers");
    vis.addPointCloud<PointXYZ> (inliers_cloud, BlueCloudHandler (inliers_cloud), "inliers");

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (inliers_cloud);
    chull.reconstruct (*cloud_hull);
    vis.addPointCloud<PointXYZ> (cloud_hull, RedCloudHandler (cloud_hull), "hull");

    // segment those points that are in the polygonal prism
    ExtractPolygonalPrismData<PointXYZ> ex;
    ex.setInputCloud (outliers);
    ex.setInputPlanarHull (cloud_hull);

    // do it
    PointIndices::Ptr output (new PointIndices);
    ex.segment (*output);
    
    // and create a point cloud object containing only the handles
    PointCloud<PointXYZ>::Ptr subcloud (new PointCloud<PointXYZ>);
    extract.setInputCloud (outliers);
    extract.setIndices (output);
    extract.filter (*subcloud);

    // Creating the KdTree object for the search method of the extraction
    KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
    tree->setInputCloud (subcloud);
    
    // split the handles point cloud into separate clusters
    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud( subcloud);
    ec.extract (cluster_indices);

    int j = 0;

    // and finally, add all found clusters to the visualizer using random colors
    for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (subcloud->points[*pit]); 

        stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        j++;

        vis.addPointCloud<PointXYZ> (cloud_cluster, PointCloudColorHandlerRandom<PointXYZ>(cloud_cluster), ss.str());
    }

    vis.spin ();
  }
}

/* ---[ */
int
  main (int argc, char** argv)
{
  // create PointCloud object
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  // load file from disk
  if (io::loadPCDFile<PointXYZ> ("rss11_04_seg.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file rss11_04_seg.pcd \n");
    return EXIT_FAILURE;
  }

  // output some basic information about the point cloud
  std::cerr << "Loaded "
            << cloud->width * cloud->height
            << " data points from rss11_04_seg.pcd with the following fields: "
            << getFieldsList (*cloud)
            << std::endl;

  // call the different examples

  sample3 (cloud);

  return EXIT_SUCCESS;
}
/* ]--- */
