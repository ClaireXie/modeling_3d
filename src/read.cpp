#include "align.h"

extern double FOCAL;
extern double MM_PER_M;
extern int WIDTH;
extern int HEIGHT;
extern double DEPTH_THRESHOLD;

#define DEP_FILTER 0

// for debugging
bool SHOW_MATCHING = 0;

void read (Mat rgbimg, Mat depimg, Mat mask, PointCloud &pointcloud,
    PointCloud &keypoints, vector<KeyPoint> &key, int x1, int y1, double minHessian)
{
  vector<KeyPoint> keytemp;
  SurfFeatureDetector detector (minHessian);
  detector.detect (rgbimg, keytemp);

  // convert the feature points in the smaller mask into point cloud
  for (int k = 0; k < keytemp.size (); k++)
  {
    int i = (int) keytemp[k].pt.y;
    int j = (int) keytemp[k].pt.x;

    unsigned short depth = depimg.at<unsigned short> (i, j);

    if (mask.at<bool> (i, j) != 0 && depth != 0 && depth / MM_PER_M < DEPTH_THRESHOLD)
    {
      PointT point;
      double x = (WIDTH-(j + x1)-WIDTH/2) * depth / FOCAL / MM_PER_M;
      double y = (HEIGHT-(i + y1)-HEIGHT/2) * depth / FOCAL / MM_PER_M;
      double z = depth / MM_PER_M;

      point.x = x;
      point.y = y;
      point.z = z;

      Vec3b rgb = rgbimg.at<Vec3b> (i, j);
      point.b = (uint8_t) rgb[0];
      point.g = (uint8_t) rgb[1];
      point.r = (uint8_t) rgb[2];

      keypoints.points.push_back (point);
      //update the keypoints deleting some points outside the mask;
      key.push_back (keytemp[k]);
    }
  }

  // convert all the points in the mask into point cloud
  for (int i = 0; i < rgbimg.rows; i++)
  {
    for (int j = 0; j < rgbimg.cols; j++)
    {
      unsigned short depth = depimg.at<unsigned short> (i, j);
      if (mask.at<bool> (i, j) != 0 && depth != 0 && depth / MM_PER_M < DEPTH_THRESHOLD)
      {
        PointT point;
        double x = (WIDTH-(j + x1)-WIDTH/2) * depth / FOCAL / MM_PER_M;
        double y = (HEIGHT-(i + y1)-HEIGHT/2) * depth / FOCAL / MM_PER_M;
        double z = depth / MM_PER_M;

        point.x = x;
        point.y = y;
        point.z = z;

        Vec3b rgb = rgbimg.at<Vec3b> (i, j);
        point.b = (uint8_t) rgb[0];
        point.g = (uint8_t) rgb[1];
        point.r = (uint8_t) rgb[2];

        pointcloud.points.push_back (point);
      }
    }
  }

  pointcloud.width = (uint32_t) pointcloud.points.size ();
  pointcloud.height = 1;
  keypoints.width = (uint32_t) keypoints.points.size ();
  keypoints.height = 1;

}

/* perform 2D SURF feature matching */
void match (Mat img_1, Mat img_2, vector<KeyPoint> keypoints_1,
    vector<KeyPoint> keypoints_2, vector<DMatch> &good_matches,
    pcl::CorrespondencesPtr &correspondences)
{
  SurfDescriptorExtractor extractor;
  Mat descriptors_1, descriptors_2;

  extractor.compute (img_1, keypoints_1, descriptors_1);
  extractor.compute (img_2, keypoints_2, descriptors_2);

  //FlannBasedMatcher matcher;
  BFMatcher matcher (NORM_L2);
  std::vector<DMatch> matches;

  matcher.match (descriptors_1, descriptors_2, matches);

  double max_dist = 0;
  double min_dist = 100;

  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = matches[i].distance;

    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  for (int i = 0; i < descriptors_1.rows; i++)
  {
    // need to change the factor "2" to adapt to different cases
    if (matches[i].distance < 3 * min_dist)  //may adapt for changes
    {
      good_matches.push_back (matches[i]);
    }
  }

  correspondences->resize (good_matches.size ());

  for (unsigned cIdx = 0; cIdx < good_matches.size (); cIdx++)
  {
    (*correspondences)[cIdx].index_query = good_matches[cIdx].queryIdx;
    (*correspondences)[cIdx].index_match = good_matches[cIdx].trainIdx;

    if (0)  // for debugging
    {
      cout << good_matches[cIdx].queryIdx << " " << good_matches[cIdx].trainIdx
          << " " << good_matches[cIdx].distance << endl;
      cout << good_matches.size () << endl;
    }
  }

  // change the constant value of SHOW_MATCHING to 1 if you want to visulize the matching result
  if (SHOW_MATCHING)
  {
    Mat img_matches;
    drawMatches (img_1, keypoints_1, img_2, keypoints_2, good_matches,
        img_matches, Scalar::all (-1), Scalar::all (-1), vector<char> (),
        DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //-- Show detected matches
    imshow ("Good Matches", img_matches);
    waitKey (0);
  }
}

/* read the images with prefix and suffix */
void readImg (string prefix, string filename, string suffix, Mat &img)
{
  string imagefile;
  imagefile = prefix;
  imagefile.append (filename);
  imagefile.append (suffix);
  img = imread (imagefile, -1);
}

/* read the images with suffix */
void readImg (string filename, string suffix, Mat &img)
{
  string imagefile;
  imagefile = filename;
  imagefile.append (suffix);
  img = imread (imagefile, -1);
}

/* errosion operation */
void erosion (Mat &src, int erosion_size)
{
  int erosion_type = MORPH_RECT;

  Mat element = getStructuringElement (erosion_type,
      Size (2 * erosion_size + 1, 2 * erosion_size + 1),
      Point (erosion_size, erosion_size));

  /// Apply the erosion operation
  erode (src, src, element);
}

