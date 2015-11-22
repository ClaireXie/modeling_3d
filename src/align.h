#ifndef ALIGN_H_
#define ALIGN_H_

#include <stdio.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_representation.h>

// opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <stdlib.h>
#include <math.h>
#include <pcl/PolygonMesh.h>

// visualization
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef PointCloud::Ptr PointCloudPtr;
typedef pair<string, PointCloudPtr> CloudPair;
typedef vector<CloudPair> CloudVector;

//------------------------------------------------------------------------------------------------
#define PRINT_OVERLAPPING 0
#define OUTPUT_COR 0
//------------------------------------------------------------------------------------------------


// save point cloud to ply
void save2ply (PointCloud cloud, const char* name);

//calculate the Euclidean distance
double distance (double x1, double y1, double z1, double x2, double y2, double z2);

// read the RGB-D images and convert them into point cloud
void read (Mat rgbimg, Mat depimg, Mat mask, PointCloud &pointcloud, PointCloud &keypoints, vector<KeyPoint> &key,
    int x1, int y1, double minHessian);

// perform 2D SIFT feature matching
void match (Mat img_1, Mat img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2,
    vector<DMatch> &good_matches, pcl::CorrespondencesPtr &correspondences);

// perform filtering
void filter (PointCloud &pointcloud, float downsample, PointCloud &output);

void erosion (Mat &src, int erosion_size);

// computing the norm and surface variation for a given point cloud
void curvature (ofstream &file, PointCloud src_points);

// compare two point cloulds (same point cloud in two poses)
Eigen::Affine3f compare (PointCloud p1, PointCloud p2);

//
class MyPointRepresentation : public pcl::PointRepresentation<PointT>
{
  using pcl::PointRepresentation<PointT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    nr_dimensions_ = 6;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void
  copyToFloatArray (const PointT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    // RGB -> YUV
    out[3] = float (p.r) * 0.299 + float (p.g) * 0.587 + float (p.b) * 0.114;
    out[4] = float (p.r) * 0.595716 - float (p.g) * 0.274453 - float (p.b) * 0.321263;
    out[5] = float (p.r) * 0.211456 - float (p.g) * 0.522591 + float (p.b) * 0.311135;
  }
};

// read the images with prefix and suffix
void readImg (string prefix, string filename, string suffix, Mat &img);

void readImg (string filename, string suffix, Mat &img);

// perform plane fitting
//int fitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_raw,
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &final, float threshold);
int planeFitting (PointCloud::Ptr cloud_raw, PointCloud::Ptr &final, float threshold);

void meshFitting_RGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PolygonMesh &triangles);
void meshFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &triangles);
// TODO delaunaey triangulation
//int dt(PointCloud cloud0);

// loop closure detection
bool loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last);

// coarse alignment according to the SIFT matching + RANSAC outlier rejection
Eigen::Matrix4f computeInitialAlignment (const PointCloud & ksrc_points, const PointCloud & ktgt_points,
    pcl::CorrespondencesPtr correspondences);

// fine registration with initial transform guess from the coarse alignment
Eigen::Matrix4f fineAlign (const PointCloudPtr & src_points, const PointCloudPtr & tgt_points, PointCloud &output,
    double threshold, bool initial);

// global alignment
void globalAlign (std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > data, PointCloudPtr &output,
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &matrix_buffer,
    std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > &out, int &first, int &last);

void printUsage (const char* progName);

#endif
