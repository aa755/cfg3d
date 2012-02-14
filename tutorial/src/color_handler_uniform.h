/******************************************************************************
 * RSS 2011 - PCL Tutorial                                                    *
 * Session 04 - Segmentation                                                  *
 * Nico Blodow                                                                *
 *****************************************************************************/

#include <pcl/visualization/point_cloud_handlers.h>

using namespace pcl;
using namespace pcl::visualization;

template <typename PointT>
class PointCloudHandlerUniform : public PointCloudColorHandler<PointT>
{
  public:
    /** \brief Constructor */
    PointCloudHandlerUniform (const typename PointCloud<PointT>::ConstPtr &cloud,
                              unsigned char r, unsigned char g, unsigned char b)
      : PointCloudColorHandler<PointT> (cloud)
      , r_(r), g_(g), b_(b)
    {
      capable_ = true;
    }
    
    /** \brief Abstract getName method. */
    virtual inline std::string 
    getName () const { return ("PointCloudHandlerUniform"); }

    /** \brief Get the name of the field used. */
    virtual std::string 
    getFieldName () const { return ("[uniform]"); }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
      * \param scalars the resultant scalars containing the color for the input dataset
      */
    virtual void 
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
      if (!capable_)
        return;

      if (!scalars)
        scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
      scalars->SetNumberOfComponents (3);

      vtkIdType nr_points = cloud_->width * cloud_->height;
      reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

      // Allocate space for colors
      unsigned char* colors = new unsigned char[nr_points * 3];

      // Color every point
      for (vtkIdType cp = 0; cp < nr_points; ++cp)
      {
        colors[cp * 3 + 0] = r_;
        colors[cp * 3 + 1] = g_;
        colors[cp * 3 + 2] = b_;
      }
      reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
    }

  protected:
    // Members derived from the base class
    using PointCloudColorHandler<PointT>::cloud_;
    using PointCloudColorHandler<PointT>::capable_;
    unsigned char r_, g_, b_;
};

class WhiteCloudHandler : public PointCloudHandlerUniform<PointXYZ>
{
  public:
    WhiteCloudHandler (const PointCloud::ConstPtr &cloud)
      : PointCloudHandlerUniform<PointXYZ> (cloud, 255, 255, 255)
    {}
};

class RedCloudHandler : public PointCloudHandlerUniform<PointXYZ>
{
  public:
    RedCloudHandler (const PointCloud::ConstPtr &cloud)
      : PointCloudHandlerUniform<PointXYZ> (cloud, 255, 0, 0)
    {}
};

class BlueCloudHandler : public PointCloudHandlerUniform<PointXYZ>
{
  public:
    BlueCloudHandler (const PointCloud::ConstPtr &cloud)
      : PointCloudHandlerUniform<PointXYZ> (cloud, 0, 0, 255)
    {}
};

