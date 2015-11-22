#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/icp.h>
#include "align.h"

#define PRINT_OVERLAPPING 0
#define OUTPUT_COR 0

extern double MIN_DISTANCE;
bool KEY_FRAME = 0;


/* loop closure detection */
bool loopDetection (int end, const CloudVector &clouds, double dist, int &first,
    int &last)
{
  static double min_dist = -1;
  int state = 0;

  for (int i = end - 1; i >= 0; i--)
  {
    Eigen::Vector4f cstart, cend;
    //Use percentage of overlapping area
    pcl::registration::CorrespondenceEstimation<PointT, PointT> ce;
    ce.setInputTarget (clouds[i].second);  //match
    ce.setInputSource (clouds[end].second);  //query
    pcl::CorrespondencesPtr corr (new pcl::Correspondences);
    ce.determineCorrespondences (*corr, MIN_DISTANCE);
    double norm = 1.0
        - double (corr->size ()) / double (clouds[end].second->size ());

    // for debugging
    if (PRINT_OVERLAPPING)
    {
      std::cout << "overlapping percentage between " << i << " and " << end
          << " is " << norm << " state is " << state << std::endl;
    }

    if (state == 0 && norm > dist)
    {
      state = 1;
    }

    if (state > 0 && norm < dist)
    {
      state = 2;
      //cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << endl;
      if (min_dist < 0 || norm < min_dist)
      {
        min_dist = norm + 0.001;
        first = i;
        last = end;
      }
    }
  }

  if (min_dist > 0 && (state < 2 || end == int (clouds.size ()) - 1))
  {
    min_dist = -1;
    return true;
  }
  return false;
}


/* coarse alignment according to the SIFT matching + RANSAC outlier rejection */
Eigen::Matrix4f computeInitialAlignment (const PointCloud & ksrc_points,
    const PointCloud & ktgt_points, pcl::CorrespondencesPtr correspondences)
{

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
    pcl::Correspondences inliers;

    PointCloudPtr ksrc (new PointCloud (ksrc_points));
    PointCloudPtr ktgt (new PointCloud (ktgt_points));

    rejector.setInputSource (ksrc);
    rejector.setInputTarget (ktgt);
    rejector.setInputCorrespondences (correspondences);
    rejector.setMaximumIterations (200);
    rejector.setInlierThreshold (MIN_DISTANCE);  //0.0035 for case 1
    rejector.getCorrespondences (inliers);

    if (OUTPUT_COR)
    {
        ofstream cor;
        cor.open ("correspondence.txt");

        for (int i = 0; i < inliers.size (); i++)
        {
          int a = inliers[i].index_query;
          int b = inliers[i].index_match;

          cor << (ksrc_points).points[a].x << " " << (ksrc_points).points[a].y
              << " " << (ksrc_points).points[a].z << " "
              << (ktgt_points).points[b].x << " " << (ktgt_points).points[b].y
              << " " << (ktgt_points).points[b].z << endl;
        }
        cor.close();
    }

    cout << "RANSAC_rejection--correspondences: " << correspondences->size ()
      << " --> " << inliers.size () << endl;
    return rejector.getBestTransformation ();
}

/* fine registration with initial transform guess from the coarse alignment */
Eigen::Matrix4f fineAlign (const PointCloudPtr & src_points,
    const PointCloudPtr & tgt_points, PointCloud &output, double threshold, bool initial)
{

  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;

  // TODO:
  // 1. color based ICP
  // 2. non-linear ICP
  // 3. weight the 'curvature' dimension so that it is balanced against x, y, and z

  float alpha[6] = { 1.0, 1.0, 1.0, 0.1 / 255, 0.1 / 255, 0.1 / 255 };
  point_representation.setRescaleValues (alpha);

  pcl::IterativeClosestPoint<PointT, PointT> reg;
  reg.setTransformationEpsilon (1e-6);
  reg.setMaxCorrespondenceDistance (threshold);
  // Set the point representation
  reg.setPointRepresentation (
      boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (src_points);
  reg.setInputTarget (tgt_points);
  reg.align (output);

  if (initial)
    printf ("ICP Score: %f \n", reg.getFitnessScore ());

  pcl::transformPointCloud (*src_points, output, reg.getFinalTransformation ());

  return reg.getFinalTransformation ();
}


/* global optimization using elch */
void globalAlign (
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > data,
    PointCloudPtr &output,
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &matrix_buffer,
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > &out,
    int &first, int &last)
{

  pcl::registration::ELCH<PointT> elch;
  pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp (
      new pcl::IterativeClosestPoint<PointT, PointT>);
  icp->setMaximumIterations (100);
  icp->setMaxCorrespondenceDistance (0.0025);
  icp->setRANSACOutlierRejectionThreshold (0.2);
  elch.setReg (icp);

  CloudVector clouds;
  for (int i = 0; i < data.size (); i++)
  {
    PointCloudPtr pc (new PointCloud (data[i]));

    stringstream strStream;
    strStream << i;
    clouds.push_back (CloudPair (strStream.str (), pc));

    elch.addPointCloud (clouds[i].second);
  }

  int keyframe = 0;

  if (first == 0 && last == 0)
  {
    size_t i = clouds.size () - 1;
    {
      if (loopDetection (int (i), clouds, 0.8, first, last))  //adaptive in the future
      {
        cout << "Loop between #" << first + 1 << " and #" << last + 1 << endl;
        elch.setLoopStart (first);
        elch.setLoopEnd (last);
        elch.compute ();
      }

    }
  }

  else
  {
    cout << "Loop between #" << first + 1 << " and #" << last + 1 << endl;
    elch.setLoopStart (first);
    elch.setLoopEnd (last);
    elch.compute ();
  }

  for (size_t i = 0; i < clouds.size (); i++)
  {
    Eigen::Affine3f adjust = compare (data[i], * (clouds[i].second));
    matrix_buffer[i] = adjust * matrix_buffer[i];

    out.push_back (* (clouds[i].second));

    // only fuse the key frames with less overlaps
    if (KEY_FRAME)
    {
      if (i == 0)
        *output = * (clouds[0].second);

      else if (i > 0)
      {
        pcl::registration::CorrespondenceEstimation<PointT, PointT> ce;
        ce.setInputTarget (clouds[i].second);  //match
        ce.setInputSource (clouds[keyframe].second);  //query
        pcl::CorrespondencesPtr corr (new pcl::Correspondences);
        ce.determineCorrespondences (*corr, 0.003f);
        double nonoverlap = 1- double (corr->size ())/ double (clouds[keyframe].second->size ());

        if (nonoverlap > 0.3)
        {
          cout << "Keyframe is " << i << endl;
          keyframe = i;
          *output = *output + * (clouds[i].second);
        }
      }
    }

    else
      *output = *output + * (clouds[i].second);
  }
}
