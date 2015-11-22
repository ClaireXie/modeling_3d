#include "align.h"
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>

bool FINE_FLAG = 1;             /*Fine registration*/
bool GLOBAL_FLAG = 0;           /*Global alignment*/
bool DENOISING = 0;             /*Denoise - outlier rejection*/
bool DEPTH_AVG = 0;             /*Down-sampling (recommand apply it before mls)*/
bool MLS = 0;                   /*Denoise - moving least square*/
double MIN_DISTANCE = 0.0035f;  /*Registration threshold*/
double DOWN_SAMPLE = 0.002f;    /*Downsampling voxel size*/
bool LOOP_CLOSURE = 1;          /*Loop closure*/
int GITER=3;                    /*Global alginment iterations*/
double HESSIAN = 200;           /*Hessian threshold for sift*/

// Since the images are cropped, the cropped location is also provided
// If no crop, set the parameters below as zeros
int TOP_LEFT1 = -1;             /*Crop location - x*/
int TOP_LEFT2 = -1;             /*Crop location - y*/


// Global var for kinect_v1
/*-----------------------------------------------------------------*/
double FOCAL = 570.3;
double MM_PER_M = 1000;
int WIDTH = 640;
int HEIGHT = 480;
double DEPTH_THRESHOLD = 1.2;   /*The largest depth we capture*/
/*-----------------------------------------------------------------*/

pcl::visualization::PCLVisualizer *p;

// print sturctions
#define PRINT_TRANSFORM_PAIR 0
#define PRINT_TRANSFORM 1

// saving structions
#define SAVE_EACH_PCD 0

void
showScene (pcl::visualization::PCLVisualizer& viewer, Eigen::Matrix4f m, PointCloud output);


/* Registration: combine the initial and fine registration together
 * @params
        src_points: source point cloud
        tgt_points: target point cloud
        src_key: the 3D SURF point in the source pcd
        tgt_key: the 3D SURF point in the target pcd
        correspondences: pointer of correspondences of the 2D surf key points
        initial_matrix:initial transform
 * @output
        output point cloud
        initial: flag to indicate whether to apply initial reg.
        threshold: threshold for ICP
 */

Eigen::Matrix4f twostepAlign (const PointCloud &src_points,
    const PointCloud &tgt_points, const PointCloud &src_key,
    const PointCloud &tgt_key, pcl::CorrespondencesPtr correspondences,
    const Eigen::Matrix4f initial_matrix, PointCloud &output, bool initial,
    double threshold)
{
  Eigen::Matrix4f matrix1;  // initial
  Eigen::Matrix4f matrix2;  //fine*initial
  Eigen::Matrix4f matrix_final;

  // smart pointer no need to delete it at the end
  PointCloudPtr src (new PointCloud (src_points));
  PointCloudPtr tgt (new PointCloud (tgt_points));

  bool flag = 1;

  // initial alignment
  if (initial && flag == 1)
  {
	  if (correspondences->size () < 3)
	  {
	    printf (
	        "The correspondence points are less than 3. The RANSAC based initial alignment will be swithched off.  \n");
	    printf ("Please relax the threshold for SURF feature mtaching.  \n");
	    flag = 0;
	  }
	  else
		  matrix1 = computeInitialAlignment (src_key, tgt_key, correspondences);
  }

  // Switch off the initial alignment
  if (!initial || flag == 0)
  {
    matrix1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  }
  PointCloudPtr points_transformed (new PointCloud);
  pcl::transformPointCloud (*src, *points_transformed, matrix1);

  // fine registration
  if (FINE_FLAG)
  {
    matrix2 = fineAlign (points_transformed, tgt, output, threshold,initial) * matrix1;  //pairwise alignment matrix
  }

  else  // Switch off the fine alignment
  {
    matrix2 = matrix1;
    output = *points_transformed;
  }

  // initial_matrix: the initial alignment refered to the first frame
  // output is the transformed point cloud refered to the firs frame
  matrix_final = initial_matrix * matrix2;
  pcl::transformPointCloud (output, output, initial_matrix);

  // print the transform for debugging
  if (PRINT_TRANSFORM_PAIR)
  {
    cout << "Final Tranform: " << endl;
    for (int k = 0; k < 3; k++)
    {
      for (int j = 0; j < 4; j++)
      {
        cout << matrix_final (k, j) << " ";
      }
      cout << endl;
    }
  }

  // return the final ransfrom relative to the first frame
  return matrix_final;
}


/* perform the pairwise alignment for a bunch of data (global alignment)
 * @params
        data: a bunch of point clouds for pairwise registration
        correspondencesdata: the correpondence information for pairwise frames that is pre-processed
        key3data: the 3D feature points in each frame
        output: the output pcd combining all the registered pcds
        out_data: a bunch of output point clouds for each frame
        matrix_buffer: a bunch a matrix storing the transform
 */

void pairwiseAlign (
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > data,
    vector<pcl::CorrespondencesPtr> correspondencesdata,
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > key3data,
    PointCloud &output,
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > &out_data,
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & matrix_buffer,
    bool initial, double threshold)
{
  cout << "pariwise registration ..." << endl;

  Eigen::Matrix4f matrix_initial;
  Eigen::Matrix4f matrix;
  matrix_initial << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  output = data[0];
  if (initial == 1)
  {
    p->addPointCloud (data[0].makeShared (),"3");
    p->spinOnce ();
  }

  matrix_buffer.push_back (matrix_initial);

  if (SAVE_EACH_PCD)
  {
    pcl::io::savePCDFile ("pcd/p1.pcd", data[0], true);
  }

  out_data.push_back (data[0]);

  for (unsigned int i = 1; i < data.size (); i++)
  {

    PointCloudPtr temp (new PointCloud ());
    if (initial)
      cout << "# " << i << "->" << i + 1 << ":" << endl;
    matrix = twostepAlign (data[i], data[i - 1], key3data[i], key3data[i - 1],
        correspondencesdata[i - 1], matrix_initial, *temp, initial, threshold);

    if (SAVE_EACH_PCD)
    {
      string savename;
      stringstream strStream;
      strStream << i + 1;
      savename = "pcd/p" + strStream.str () + ".pcd";
      pcl::io::savePCDFile (savename.c_str (), *temp, true);
    }

    out_data.push_back (*temp);

    // for global align
    if (matrix_buffer.size () < i + 1)
      matrix_buffer.push_back (matrix);
    else
      matrix_buffer[i] = matrix * matrix_buffer[i];

    matrix_initial = matrix;

    output = output + *temp;
    if (initial==1)
      showScene (*p, matrix_buffer[i], output);
  }

}

int main (int argc, char** argv)
{
  double f;
  double vd;
  double r;
  int iter;
  double h;

  // argiment parsing
  if (pcl::console::parse_argument (argc, argv, "-r", r) >= 0)
  {
    MIN_DISTANCE = r;
    cout << "adjust the registration threshold to " << r << endl;
  }

  if (pcl::console::parse_argument (argc, argv, "-giter", iter) >= 0)
  {
    GITER = iter;
    cout << "adjust global alignment iterations to " << iter <<" iterations"<< endl;
  }

  if (pcl::console::parse_argument (argc, argv, "-hessian", h) >= 0)
  {
    HESSIAN=h;
    cout << "adjust SIFT matching threshold to " << h << endl;
  }

  if (pcl::console::find_argument (argc, argv, "-h") >= 0 || argc == 1)
  {
    printUsage (argv[0]);
    return 0;
  }

  if (pcl::console::find_argument  (argc, argv, "-g") >= 0)
  {
    GLOBAL_FLAG = 1;
    cout << "Global Reistration is ON" << endl;
  }

  if (pcl::console::find_argument (argc, argv, "-lcoff") >= 0)
   {
    LOOP_CLOSURE = 0;
    cout << "Loop CLosure Detection is OFF" << endl;
   }

  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    MLS = 1;
    cout << "Moving least square smoothing is ON" << endl;
  }

  if (pcl::console::find_argument (argc, argv, "-dn") >= 0)
  {
    DENOISING= 1;
    cout << "De-noising (statistic) is ON" << endl;
  }


  if (pcl::console::parse_argument (argc, argv, "-vd", vd) >= 0)
  {
    DEPTH_AVG = 1;
    cout << "Down-sampling is ON" << endl;
    if (vd>0.00001)
    {
      DOWN_SAMPLE = vd;
      cout<<"Down sample threshold is changed to "<<DOWN_SAMPLE<<endl;
    }

  }

  vector<PointCloud, Eigen::aligned_allocator<PointCloud> > data;
  vector<PointCloud, Eigen::aligned_allocator<PointCloud> > key3data;
  vector<vector<KeyPoint> > key2data;
  vector<pcl::CorrespondencesPtr> correspondencesdata;

  Mat previousrgb;
  vector<KeyPoint> previouskey2d;

  // read the rgb-d data one by one
  ifstream index;
  string root_dir = argv[1];
  index.open (string(root_dir + "/input.txt").c_str());

  int i = 0;
  while (!index.eof ())
  {
    string file;
    index >> file;
    if (file.empty ())
      break;

    file = root_dir + "/" + file;

    Mat rgbimg;
    Mat depimg;
    Mat mask;

    // read data
    readImg (file, "_rgb.png", rgbimg);
    readImg (file, "_depth.png", depimg);
    readImg (file, "_mask.png", mask);

    if (rgbimg.empty () || depimg.empty () || mask.empty ())
    {
      printf (
          "Error: The RGB-D file does not exit. Please note that the right format should be: \n");
      printf (
          "X_rgb.png, X_depth.png, X_mask.png with the same size. ('X' is your input) \n");
      return (0);
    }

    if (rgbimg.size () != depimg.size () || rgbimg.size () != mask.size ())
    {
      printf (
          "Error: The RGB-D files are not consistent. The rgb, depth and mask images should have the same size. \n");
      return (0);
    }

    cout << "loading:  " << file << endl;

    // check the image size and the offest coordinate
    int x1, y1;
    if (rgbimg.rows == 480 && rgbimg.cols == 640)
    {
      // if the image is full size (480*640), no offset coordinate
      x1 = 0;
      y1 = 0;
    }
    else
    {
      // if not, read the offset file
      ifstream loc;
      string location1 = file + "_loc.txt";
      loc.open (location1.c_str ());

      if (!loc.is_open ())
      {
        if (TOP_LEFT1!=0 && TOP_LEFT2!=0)
        {
        	x1=TOP_LEFT2;
        	y1=TOP_LEFT1;
        }
        else
        {
      	  cout << "Error! There is no associated off-set location file." << endl;
          return 0;
        }
      }
      else
      {
        char comma;
        loc >> x1 >> comma >> y1;
      }
      loc.close();
    }

    // perform erosion to remove the noises around the boundary
    erosion (mask, 4);

    vector<KeyPoint> key2d;
    PointCloud pointcloud;
    PointCloud keypoints;

    // read data and perform surf feature matching
    read (rgbimg, depimg, mask, pointcloud, keypoints, key2d, x1, y1, HESSIAN);
    filter (pointcloud, 0.001, pointcloud);
    data.push_back (pointcloud);
    key3data.push_back (keypoints);
    key2data.push_back (key2d);

    // find the pairwise correspondecnes based on surf feature
    if (i >= 1)
    {
      pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());  //may have bugs??
      vector<DMatch> good_matches;
      match (rgbimg, previousrgb, key2d, previouskey2d, good_matches,
          correspondences);
      correspondencesdata.push_back (correspondences);
      correspondences.reset (new pcl::Correspondences);
    }
    previousrgb = rgbimg;
    previouskey2d = key2d;

    i++;
  }

  index.close();

  printf ("Loaded %d datasets.\n", (int) data.size ());

  PointCloudPtr final (new PointCloud);
  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > matrix_buffer;

  // registration: initial+fine+global

  //initialize the PCL viewer
  p = new pcl::visualization::PCLVisualizer (argc, argv, "3D Moeling example");
  p->setSize (480, 640);
  p->setPosition (480, 200);

  // registration
  int first = 0;
  int last = 0;
  if (!LOOP_CLOSURE)
  {
	  last=(int) data.size ()-1;
  }

  // need to change back
  double threshold = MIN_DISTANCE;
  for (int z = 0; z < GITER; z++)
  {
    vector<PointCloud, Eigen::aligned_allocator<PointCloud> > out;

    PointCloudPtr output (new PointCloud);
    PointCloudPtr final_temp (new PointCloud);

    if (GLOBAL_FLAG)
    {

      vector<PointCloud, Eigen::aligned_allocator<PointCloud> > out_global;

      if (z == 0)
        pairwiseAlign (data, correspondencesdata, key3data, *output, out,
            matrix_buffer, 1, threshold);
      else
      {
        pairwiseAlign (data, correspondencesdata, key3data, *output, out,
            matrix_buffer, 0, threshold);
      }

      // global optimization
      cout << endl << "Global optimization begins ... at iteration " <<z+1<< endl;
      globalAlign (out, final_temp, matrix_buffer, out_global, first, last);
      data = out_global;
    }
    else
    {
      pairwiseAlign (data, correspondencesdata, key3data, *output, out,
          matrix_buffer, 1, threshold);
      final = output;
      break;
    }
    threshold = threshold * 0.9;

    final = final_temp;
    showScene (*p, matrix_buffer[(int) data.size ()-1], *final);
  }


  if (PRINT_TRANSFORM)
  {
    for (int i = 0; i < data.size (); i++)
    {
      cout << "# " << i + 1 << ": " << endl;
      for (int k = 0; k < 3; k++)
      {
        for (int j = 0; j < 4; j++)
        {
          cout << matrix_buffer[i] (k, j) << " ";
        }
        cout << endl;
      }
      cout << "-------------------------------------------" << endl;
    }
  }


  // de-noising
  PointCloudPtr denoised (new PointCloud);
  if (DENOISING)
  {
    cout<<"statistical outlier removal begins ..."<<endl;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (final);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*denoised);
    showScene (*p, matrix_buffer[(int) data.size ()-1], *denoised);
  }
  else
    denoised = final;

  // if voxel down-sampling is needed
  PointCloudPtr downsampled (new PointCloud);
  if (DEPTH_AVG)
  {
    cout<<"Down sampling the point cloud..."<<endl;
    pcl::VoxelGrid<PointT> vf;
    vf.setInputCloud (denoised);
    vf.setLeafSize (DOWN_SAMPLE, DOWN_SAMPLE, DOWN_SAMPLE);
    vf.filter (*downsampled);
    showScene (*p, matrix_buffer[(int) data.size ()-1], *downsampled);
  }
  else
    downsampled=denoised;

  // moving least square smoothing
  PointCloudPtr mls_points (new PointCloud);
  if (MLS && DEPTH_AVG)
  // if the pcd is not downsampled, the mls process will be much slower
  {
    cout<<"moving least square begins ..."<<endl;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (downsampled);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.01);
    mls.process (*mls_points);

    showScene (*p, matrix_buffer[(int) data.size ()-1], *mls_points);
  }
  else
    mls_points = downsampled;

  //=============EOD of The Process========================

  cout<<"Number of points: "<<mls_points->size()<<endl;

  // save the final pcd
  string ply_name = root_dir + "/" + "result.ply";
  cout << "saving data to " << ply_name << endl;
  save2ply(*mls_points, ply_name.c_str());

  cout << "done!" << endl;


  // show the result
  while (!p->wasStopped ())
  {
    p->spin ();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);

}

void
showScene (pcl::visualization::PCLVisualizer& viewer, Eigen::Matrix4f m, PointCloud output)
{

    viewer.removeAllPointClouds ();
    viewer.addPointCloud (output.makeShared ());

     Eigen::Matrix3f r;
     r<< m(0, 0), m(0, 1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2);
     Eigen::Vector3f t(m(0,3),m(1,3),m(2,3));

     Eigen::Affine3f pose = viewer.getViewerPose();
     pose.linear()=r;
     pose.translation()=t;

     Eigen::Vector3f pos_vector = pose * Eigen::Vector3f (0, 0, 0);
     Eigen::Vector3f look_at_vector = pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
     Eigen::Vector3f up_vector = pose.rotation () * Eigen::Vector3f (0, 1, 0);
     viewer.setCameraPosition(pos_vector[0],pos_vector[1],pos_vector[2],look_at_vector[0],look_at_vector[1],look_at_vector[2],
                                                   up_vector[0],up_vector[1],up_vector[2]);
     viewer.spinOnce ();
}

