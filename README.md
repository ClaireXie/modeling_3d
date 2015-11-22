This is a simple demo for 3D Object Modeling based on an RGB-D camera 

General Notes
-------------------------
This package is designed to use RGB-D images for 3D object reconstruction. 

If you use this code for your reasearch, please cite the following papers:
```
[1] J. Xie, Y.F. Hsu, R. Feris, M.T. Sun, 
"Fine registration of 3D point clouds fusing structural and photometric information using an RGB-D camera," 
Journal of Visual Communication and Image Representation (JVCI), vol. 32, pp. 194-204, Oct. 2015.

[2] M. Jaiswal, J. Xie and M.T. Sun, 
"3D Object Modeling with a Kinect Camera," 
Asia-Pacific Signal and Information Processing Association Annual Summit and Conference (APSIPA ASC) 2014.
```
[This](https://sites.google.com/site/junxie302/code-sample-for-object-modeling-with-an-rgbd-images) webpage show a few demo of the sample code. 

Building Environment
-------------------------
Building requires CMake, OpenCV, and [PCL](http://pointclouds.org/downloads/) cross platform toolkit. The recommended OpenCV version is 2.4.x (3.x may not work!) and the recommended PCL version is 1.7. The program has been tested to successfully build on Ubuntu 14.04 with OPENCV 2.4.9, PCL 1.7.1 installed. 

To build the code
```
mkdir build
cd build
cmake ..
make
```
If there are no erros, a binary called reconstruct will be built. 

How to Run
------------------------
```
$reconstruct -[dir of txt file for the input image names]  [options]
```

Options:
```
-r   <threhold_r>       adjust registration threshold (DEFAULT=0.0035f)
-h                      this help
-g                      global Alignment
-lcoff                  turn off loop detection switch (DEFAULT=1)
-m                      moving least square surface smoothing
-dn                     de-noising (statistic outlier removal)
-vd  <threshold_vd>     voxel down sampling (DEFAULT=0.002f)
-iter <# of iterations> global alignment iterations (DEFAULT=3)
-hessian <threshold_h>  SIFT matching threshold (DEFAULT=200f), lower-> more matches
```

The txt file will point to a directory and list the image names used for reconstruction
Format of the txt file for the input image names should be:

```
dir/frame1
dir/frame2
...
dir/frame N (without format extension)
```

For each image name, it should point to four files with the following names:
```
Depth image:     framex_depth.png 
RGB image:       framex_rgb.png 
Mask image:      framex_mask.png    #pre-Segmentated mask, segmentation is not included in the source code
Location file:   framex_loc.txt     #(optional) cropping location indication if the image is cropped
```


An exmaple command is   
```
$reconstruct ../resources/ -g -dn -vd 0.001 -m
```
A few of [RGB-D data](http://rgbd-dataset.cs.washington.edu/) has been provided in resources/ in order to run the demo.




----------
Please feel free to contact xjsjtu88@gmail.com for any questions or bug reports. 
