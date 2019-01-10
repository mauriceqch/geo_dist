Notes on the software - Dong Tian @ MERL
(Best viewing under Emacs/org-mode)
* Background
  This software is written to compute geometric distrotions in a point
  cloud, including point-to-point and point-to-plane.
* Compiling notes
** PCL required
   Tested with PCL 1.8. May work with some older versions, but not tested yet.
** Compile under Linux, follow the steps:
   cd ./source
   mkdir build; cd build
*** Simple compile
    The executable pc_error would be produced under ./source/build folder.
    cmake ..
    make
*** Eclipse IDE
    The executable pc_error_d would be produced under ./run folder.
    cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j12 ..
** Compile under Window, not tested yet
* Calling examples
** ./pc_error -a cloudA.ply
   Just load the single point cloud, check its intrinsic resolutions.
** ./pc_error -a cloudA.ply -A cloudNormalsA.ply
   Estimate normals for cloudA.ply and save the point cloud with normals to
   cloudNormalsA.ply.
** ./pc_error -a cloudOrg.ply -b cloudDec.ply
   Compute the point-to-point and point-to-plane metrics. Only report RMS
   and corresponding PSNR's.
** ./pc_error -a cloudOrg.ply -b cloudDec.ply -d
   Compute the point-to-point and point-to-plane metrics. Report RMS,
   Hausdorff and corresponding PSNR's.
** ./pc_error -a cloudOrg.ply -b cloudDec.ply -f
   Compute the point-to-point and point-to-plane metrics. Force normal
   estimation even normals are provided in the input cloud. This feature is
   for experimental purposes.
** More parameters for experimental purposes
   -k [ --knn ] arg (=12)    Set KNN number of neighbor points for normal
                             estimation. Default normal estimation method
   -t [ --rtimes ] arg (=0)  Guide to set radius for normal estimation,
                             times to NN points distances
   -s [ --singlePass ]       Force running a single pass, where the loop
                             is over the original point cloud. Non-symmetric
                             reports
   -1 [ --point2point ]      Force to report point-to-point metric only
** Screen snapshot example
   $ ./pc_error -a /data/bunny/bunny100.ply -b /data/bunny/noisebunny.ply
   infile1: /data/bunny/bunny100.ply
   infile2: /data/bunny/noisebunny.ply
   knn = 12
   force normal estimation: 0

   Reading file 1 done.
   Reading file 2 done.
   Minimum and maximum NN distances (intrinsic resolutions): 0.00156692, 0.00659599
   Point cloud sizes for org version, dec version, and the scaling ratio: 1889, 1889, 1

   0. Preparing normals.
      KNN in use: 12
      Normal estimation begin..
      Normal estimation on original point cloud DONE! It takes 0 seconds (in CPU time).
      Converting normal vector DONE. It takes 0 seconds (in CPU time).

   1. Use infile1 (A) as reference, loop over A, use normals on B. (A->B).
      Error computing takes 0 seconds (in CPU time).
      rms1      (p2point): 0.000771653
      rms1,PSNR (p2point): 18.6372
      rms1      (p2plane): 0.000388834
      rms1,PSNR (p2plane): 24.5903
   2. Use infile2 (B) as reference, loop over B, use normals on A. (B->A).
      Error computing takes 0 seconds (in CPU time).
      rms2      (p2point): 0.000771542
      rms2,PSNR (p2point): 18.6384
      rms2      (p2plane): 0.000388361
      rms2,PSNR (p2plane): 24.6009
   3. Final (symmetric).
      rmsF      (p2point): 0.000771653
      rmsF,PSNR (p2point): 18.6372
      rmsF      (p2plane): 0.000388834
      rmsF,PSNR (p2plane): 24.5903
   Job done! 1 seconds elapsed (excluding the time to load the point clouds).
