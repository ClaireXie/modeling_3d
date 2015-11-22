#include "align.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

/* calculate the Euclidean distance */
double distance (double x1, double y1, double z1, double x2, double y2,
    double z2)
{
  return sqrt (
      (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

/* save point cloud to *.ply format */
void save2ply (PointCloud cloud, const char* name)
{
  int num = cloud.points.size ();

  ofstream myfile;
  myfile.open (name);

  myfile << "ply" << endl;
  myfile << "format ascii 1.0" << endl;
  myfile << "comment authoer Jun" << endl;
  myfile << "element vertex " << num << endl;
  myfile << "property float x" << endl;
  myfile << "property float y" << endl;
  myfile << "property float z" << endl;
  myfile << "property uchar red" << endl;
  myfile << "property uchar green" << endl;
  myfile << "property uchar blue" << endl;
  myfile << "end_header" << endl;

  for (int i = 0; i < num; i++)
  {
    myfile << float (cloud.points[i].x) << " " << float (cloud.points[i].y)
        << " " << float (cloud.points[i].z) << " ";
    myfile << float (cloud.points[i].r) << " " << float (cloud.points[i].g)
        << " " << float (cloud.points[i].b) << " " << endl;
  }

  myfile.close ();
}


/* perform filtering */
void filter (PointCloud &pointcloud, float downsample, PointCloud &output)
{
  PointCloudPtr output0 (new PointCloud ());

  // downsampling
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (downsample, downsample, downsample);
  grid.setFilterFieldName ("z");
  grid.setFilterLimits (-1, 1);
  grid.setInputCloud (pointcloud.makeShared ());
  grid.filter (*output0);

  // outlier points removal
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (output0);
  sor.setMeanK (20);
  sor.setStddevMulThresh (1.3);
  sor.filter (output);
}

/* compare two point cloulds (same point cloud in two poses) */
Eigen::Affine3f compare (PointCloud p1, PointCloud p2)
{
  if (p1.size () != p2.size ())
    cout << "cannot compare two point clouds" << endl;

  else
  {
    pcl::TransformationFromCorrespondences trans;
    double diff = 0;
    for (int i = 0; i < p1.size (); i++)
    {
      Eigen::Vector3f tgt, src;
      tgt (0) = p1[i].x;
      tgt (1) = p1[i].y;
      tgt (2) = p1[i].z;

      src (0) = p2[i].x;
      src (1) = p2[i].y;
      src (2) = p2[i].z;

      trans.add (tgt, src, 1.0);
    }

    return trans.getTransformation ();
  }
}


/* computing the norm and surface variation for a given point cloud
 * Then write the information into a file
 */
void curvature (ofstream &file, PointCloud src_points)
{

  PointCloudPtr src (new PointCloud (src_points));
  pcl::NormalEstimation<PointT, PointNormalT> ne;
  ne.setInputCloud (src);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod (tree);

  // Output datasets
  PointCloudWithNormals::Ptr cloud_normals (new pcl::PointCloud<PointNormalT>);

  // Use all neighbors in a sphere of radius 1cm
  ne.setRadiusSearch (0.008);  //0.01

  // Compute the features
  ne.compute (*cloud_normals);

  for (int i = 0; i < src_points.points.size (); ++i)
  {
    file << float (src_points.points[i].x) << " "
        << float (src_points.points[i].y) << " "
        << float (src_points.points[i].z) << " ";
    file << float (cloud_normals->points[i].curvature) << " ";

    //uncomment the following if the normal vector is required
    file << float (cloud_normals->points[i].normal_x) << " "
        << float (cloud_normals->points[i].normal_y) << " "
        << float (cloud_normals->points[i].normal_z) << endl;

    // uncomment the follwing if (r,g,b) coordinate is requaired
    //file<<float(src_points.points[i].r/255.0)<<" "<<float(src_points.points[i].g/255.0)<<" "<<float(src_points.points[i].b/255.0)<<" "<<endl;
  }
}

/* helper function */
void printUsage (const char* progName)
{
  cout << "\n\nUsage: " << progName
      << " -<dir of txt file for the input imagenames>  [options]\n\n"
      << "Options:\n" << "-------------------------------------------\n"
      << "-r   <threhold_r>       adjust registration threshold (DEFAULT=0.0035f)\n"
      << "-h                      this help\n"
      << "-g                      global Alignment\n"
      << "-lcoff                  turn off loop detection switch (DEFAULT=1)\n"
      << "-f   <threshold_f>      surface Fitting\n"
      << "-m                      moving least square surface smoothing\n"
      << "-dn                     de-noising (statistic outlier removal)\n"
      << "-vd  <threshold_vd>     voxel down sampling (DEFAULT=0.002f)\n"
      << "-mesh                   surface mesh reconstruction\n"
      << "-text                   texture mapping\n"
      << "-iter <# of iterations> global alignment iterations (DEFAULT=3)\n"
      << "-hessian <threshold_h>  SIFT matching threshold (DEFAULT=200f), lower-> more matches\n"<< "\n\n";
}
