The dataset contains over 15K images of 20 people (6 females and 14 males - 4 people were recorded twice). 
The point cloud type is XYZLabel with labels 1 for face points, 0 otherwise.
Each frame comes with a ***_pose.txt file with ground truth provided in the form of the 3D location of the head and its rotation.
Please note that this is a database acquired with frame-by-frame estimation in mind, not tracking, for this reason, some frames are missing.

Original website containing the dataset:
http://www.vision.ee.ethz.ch/~gfanelli/head_pose/head_forest.html#db

*********************************************************
********************* LICENSE ***************************
*********************************************************

This version of the biwi head database in PCL format has 
the same license as the original one and therefore, it is 
made available for RESEARCH PURPOSES ONLY. 

The same license applies to the forest_example.txt forest 
that was trained on this database. 

*********************************************************
*********************************************************
*********************************************************

Reference papers:
G. Fanelli, M. Dantone , J. Gall, A. Fossati, and L. Van Gool
Random Forests for Real Time 3D Face Analysis 
International Journal of Computer Vision, 2012.

G. Fanelli, T. Weise, J. Gall and L. Van Gool
Real Time Head Pose Estimation from Consumer Depth Cameras
33rd Annual Symposium of the German Association for Pattern Recognition (DAGM'11)

Many thanks to Gabriele Fanelli for his permission to upload the dataset to the PCL server.
