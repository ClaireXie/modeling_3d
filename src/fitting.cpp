#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "align.h"

//using namespace std;

int planeFitting (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &final, float threshold)

{
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (
      new pcl::PointCloud<pcl::PointXYZRGB> (*cloud_raw));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp1 (
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 (
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj (
      new pcl::PointCloud<pcl::PointXYZRGB>);
  vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > model_buffer;

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::ExtractIndices<pcl::Normal> extract_n;
  std::vector<int> inliers;

  //filtered=cloud_raw;    /////////////////////////////////////////////////////////
  int points_num = cloud_raw->size ();

  // normal estimation
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud_raw);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (
      new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (
      new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.01);
  ne.compute (*cloud_normals);

  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_q (
      new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud_raw));

  int count = 0;
  while (filtered->size () > 0.2 * points_num)
  {
    std::vector<int> inliers1;
    // ransac_plane model
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (
        new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (filtered));

    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (threshold);
    ransac.computeModel ();
    ransac.getInliers (inliers);
    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients (model_coefficients);

    // second selection of inliers
    //model_p->selectWithinDistance (model_coefficients, atof(argv[2]), inliers);

    //plane coefficients already normalized
    if (0)
    {
      cout << model_coefficients[0] << " " << model_coefficients[1] << " "
          << model_coefficients[2] << " " << model_coefficients[3] << endl;
    }

    //std::vector<int> inliers_true;

    // find the true inliers by using the norm check

    count++;

    // double check- norm check
    // TODO
    std::vector<int> inliers_true;
    for (int k = 0; k < inliers.size (); k++)
    {
      int i = inliers[k];

      Eigen::Vector3f norm;
      norm[0] = cloud_normals->points[i].normal[0];
      norm[1] = cloud_normals->points[i].normal[1];
      norm[2] = cloud_normals->points[i].normal[2];

      double test_cosine = norm.head<3> ().dot (model_coefficients.head<3> ());
      double angular_tolerance = 0.15;
      double upper_limit = 1 + angular_tolerance;
      double lower_limit = 1 - angular_tolerance;

      // norm check
      if ( ( (test_cosine < upper_limit) && (test_cosine > lower_limit))
          || ( (test_cosine > -upper_limit) && (test_cosine < -lower_limit)))
        inliers_true.push_back (i);
    }

    cout << inliers.size () << " " << inliers_true.size () << endl;
    //model_p->computeModelCoefficients (inliers_true, model_coefficients);

    model_p->optimizeModelCoefficients (inliers_true, model_coefficients,
        model_coefficients);
    model_buffer.push_back (model_coefficients);

    // projection
    //TODO
    /*for (int i=0;i<points_num;i++)
     {
     double distance=abs(model_coefficients[0]*(cloud_raw->points[i].x)+model_coefficients[1]*(cloud_raw->points[i].y)
     +model_coefficients[2]*(cloud_raw->points[i].z)+model_coefficients[3])/sqrt(model_coefficients[0]*model_coefficients[0]+
     model_coefficients[1]*model_coefficients[1]+model_coefficients[2]*model_coefficients[2]);

     Eigen::Vector3f norm;
     norm[0]=cloud_normals->points[i].normal[0];
     norm[1]=cloud_normals->points[i].normal[1];
     norm[2]=cloud_normals->points[i].normal[2];

     double test_cosine = norm.head<3>().dot(model_coefficients.head<3>());
     double angular_tolerance=0.2;
     double upper_limit = 1 + angular_tolerance;
     double lower_limit = 1 - angular_tolerance;

     // norm check
     /*if (distance <0.005 || (distance<0.02 && (((test_cosine < upper_limit) && (test_cosine > lower_limit) ) ||
     ((test_cosine > -upper_limit) && (test_cosine < -lower_limit)))) )
     /*if ( distance<0.01 && (((test_cosine < upper_limit) && (test_cosine > lower_limit) ) ||
     ((test_cosine > -upper_limit) && (test_cosine < -lower_limit))) )
     inliers1.push_back(i);

     if (distance<0.008)
     inliers1.push_back(i);
     }


     //model_q->projectPoints (inliers1, model_coefficients, *proj, 0);*/

    if (1)
    {
      cout << filtered->size () << " " << cloud_normals->size () << " "
          << cloud_raw->size () << endl;
      cout << "# of planes: " << count << endl;
    }

    // Extract the inliers
    boost::shared_ptr<vector<int> > indicesptr (new vector<int> (inliers));
    extract.setInputCloud (filtered);
    extract.setIndices (indicesptr);
    extract.setNegative (false);
    extract.filter (*tmp1);
    extract.setNegative (true);
    extract.filter (*tmp2);
    filtered.swap (tmp2);

    // test the extracted inliers and write them into .cpd format
    /*pcl::PCDWriter writer;
     std::stringstream ss;
     ss << "plane" << count << ".pcd" ;
     writer.write<pcl::PointXYZRGB> (ss.str (), *tmp1, false);*/

    // *final=*final+*proj;     // plane projection
    //*final=*final+*tmp1;    // no pojection
    // extract the inliers for the norm
    extract_n.setInputCloud (cloud_normals);
    extract_n.setIndices (indicesptr);
    extract_n.setNegative (true);
    extract_n.filter (*cloud_normals);

  }

  // projection 2
  // TODO
  vector<vector<int> > inliers_proj;
  int s = model_buffer.size ();

  for (int j = 0; j < s; j++)
  {
    inliers_proj.push_back (vector<int> ());
  }

  for (int i = 0; i < points_num; i++)
  {
    Eigen::VectorXf model_coefficients;
    double distance_min = 0.006;
    double distance[s];
    int index_min = -1;
    for (int j = 0; j < s; j++)
    {
      distance[j] = abs (
          model_buffer[j][0] * (cloud_raw->points[i].x)
              + model_buffer[j][1] * (cloud_raw->points[i].y)
              + model_buffer[j][2] * (cloud_raw->points[i].z)
              + model_buffer[j][3])
          / sqrt (
              model_buffer[j][0] * model_buffer[j][0]
                  + model_buffer[j][1] * model_buffer[j][1]
                  + model_buffer[j][2] * model_buffer[j][2]);

      if (distance[j] < distance_min)
      {
        distance_min = distance[j];
        index_min = j;
      }
    }

    if (index_min >= 0)
      inliers_proj[index_min].push_back (i);

    /*if (index_min>=0)
     {
     for (int j=0;j<s;j++)
     {
     if (j!=index_min && (distance[j]-distance_min<0.0015))
     inliers_proj[j].push_back(i);
     }
     }*/
  }

  for (int j = 0; j < s; j++)
  {
    model_q->projectPoints (inliers_proj[j], model_buffer[j], *proj, 0);
    *final = *final + *proj;
  }

  return (0);
}


void meshFitting_RGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PolygonMesh &triangles)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    ne.setKSearch (20);
    ne.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
    (new pcl::PointCloud<pcl::PointXYZRGBNormal>);    //pcl::PointXYZRGBNormal
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

    // Create search tree
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2
    (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    // Set typical values for the parameters
    gp3.setSearchRadius (0.1);  //0.025
    gp3.setMu (10);
    gp3.setMaximumNearestNeighbors (300);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

}


void meshFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &triangles)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setKSearch (50);
    ne.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals
    (new pcl::PointCloud<pcl::PointNormal>);    //pcl::PointXYZRGBNormal
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2
    (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    // Set typical values for the parameters
    gp3.setSearchRadius (0.2);  //0.025
    gp3.setMu (5);
    gp3.setMaximumNearestNeighbors (200);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

}

