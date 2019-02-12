# Software
* Written by Dong Tian @ MERL
* Modified by Maurice Quach @ L2S, CNRS, CentraleSupélec, Université Paris-Saclay

This software is written to compute geometric distortions in a point
cloud, including point-to-point and point-to-plane.

If you use this code for research, please cite:

	@inproceedings{Tian_2017,
		doi = {10.1109/icip.2017.8296925},
		url = {https://doi.org/10.1109%2Ficip.2017.8296925},
		year = 2017,
		month = {sep},
		publisher = {{IEEE}},
		author = {Dong Tian and Hideaki Ochimizu and Chen Feng and Robert Cohen and Anthony Vetro},
		title = {Geometric distortion metrics for point cloud compression},
		booktitle = {2017 {IEEE} International Conference on Image Processing ({ICIP})}
	}

# Modifications
* Changed Readme to Markdown format
* Changes to output format to facilitate parsing

# Compiling notes
## PCL required
Tested with PCL 1.8. May work with some older versions, but not tested yet.
## Compile under Linux, follow the steps:
	cd ./source
	mkdir build; cd build
### Simple compile
The executable pc_error would be produced under ./source/build folder.

	cmake ..
	make
### Eclipse IDE
The executable pc_error_d would be produced under ./run folder.

	cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j12 ..
## Compile under Window, not tested yet
# Calling examples
* ./pc_error -a cloudA.ply

	Just load the single point cloud, check its intrinsic resolutions.
* ./pc_error -a cloudA.ply -A cloudNormalsA.ply

	Estimate normals for cloudA.ply and save the point cloud with normals to
	cloudNormalsA.ply.
* ./pc_error -a cloudOrg.ply -b cloudDec.ply

	Compute the point-to-point and point-to-plane metrics. Only report RMS
	and corresponding PSNR's.
* ./pc_error -a cloudOrg.ply -b cloudDec.ply -d

	Compute the point-to-point and point-to-plane metrics. Report RMS,
	Hausdorff and corresponding PSNR's.
* ./pc_error -a cloudOrg.ply -b cloudDec.ply -f

	Compute the point-to-point and point-to-plane metrics. Force normal
	estimation even normals are provided in the input cloud. This feature is
	for experimental purposes.
## More parameters for experimental purposes
	-k [ --knn ] arg (=12)    Set KNN number of neighbor points for normal
	                          estimation. Default normal estimation method
	-t [ --rtimes ] arg (=0)  Guide to set radius for normal estimation,
	                          times to NN points distances
	-s [ --singlePass ]       Force running a single pass, where the loop
	                          is over the original point cloud. Non-symmetric
	                          reports
	-1 [ --point2point ]      Force to report point-to-point metric only
## Screen snapshot example
	$ ./build/pc_error -a test/sphere100.pcd -b test/noisy_sphere100.pcd
	infile1: test/sphere100.pcd
	infile2: test/noisy_sphere100.pcd
	knn = 12
	force normal estimation: 0

	Failed to find match for field 'rgb'.
	Failed to find match for field 'normal_x'.
	Failed to find match for field 'normal_y'.
	Failed to find match for field 'normal_z'.
	Failed to find match for field 'curvature'.
	Reading file 1 done.
	Failed to find match for field 'rgb'.
	Failed to find match for field 'normal_x'.
	Failed to find match for field 'normal_y'.
	Failed to find match for field 'normal_z'.
	Failed to find match for field 'curvature'.
	Reading file 2 done.
	Minimum and maximum NN distances (intrinsic resolutions): 0.00999999, 0.0217456
	Point cloud sizes for org version, dec version, and the scaling ratio: 10000, 10000, 1

	0. Preparing normals.
		KNN in use: 12
		Normal estimation begin..
		Normal estimation on original point cloud DONE! It takes 0 seconds (in CPU time).
		Converting normal vector DONE. It takes 0 seconds (in CPU time).

	1. Use infile1 (A) as reference, loop over A, use normals on B. (A->B).
		Error computing takes 0 seconds (in CPU time).
		### A->B,rms1,p2point,0.000847707
		### A->B,rms1PSNR,p2point,28.1825
		### A->B,rms1,p2plane,0.000471006
		### A->B,rms1PSNR,p2plane,33.2869
	2. Use infile2 (B) as reference, loop over B, use normals on A. (B->A).
		Error computing takes 0 seconds (in CPU time).
		### B->A,rms2,p2point,0.000847707
		### B->A,rms2PSNR,p2point,28.1825
		### B->A,rms2,p2plane, 0.000471006
		### B->A,rms2PSNR,p2plane,33.2869
	3. Final (symmetric).
		### Symmetric,rmsF,p2point,0.000847707
		### Symmetric,rmsFPSNR,p2point,28.1825
		### Symmetric,rmsF,p2plane,0.000471006
		### Symmetric,rmsFPSNR,p2plane,33.2869
	Job done! 0 seconds elapsed (excluding the time to load the point clouds).
	
