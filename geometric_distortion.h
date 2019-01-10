/*
 * Software License Agreement
 *
 *  Point to plane metric for point cloud distortion measurement
 *  Copyright (c) 2016, MERL
 *
 *  All rights reserved.
 *
 *  Contributors:
 *    Dong Tian <tian@merl.com>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef GEOMETRIC_DISTORTION_HPP
#define GEOMETRIC_DISTORTION_HPP

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <mutex>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

namespace pcl {

  namespace geometric_quality {

    class commandPar
    {
    public:
      string file1;
      string file2;

      string normFile;          //! output the normals to this file

      float  rtimes;    //! \times the minimum distance of nearest neighbor
      float  radius;    //! radius to estimate normals. to be derived based on rtimes
      int    knn;       //! knn method to do normal estimation

      bool   force; //! Force to do normal estimation, even if the normals provided in the input
      bool   singlePass; //! Force to run a single pass algorithm. where the loop is over the original point cloud

      bool   hausdorff;         //! true: output hausdorff metric as well

      bool   c2c_only;          //! skip point-to-plane metric
      commandPar()
        {
          file1 = ""; file2 = "";
          normFile = "";
          rtimes = -1.0;
          radius = 1.0;
          knn = 0;
          force = false;
          singlePass = false;
          hausdorff = false;
          c2c_only = false;
        }
    };

    /**!
     * \brief
     *  Store the quality metric for point to plane measurements
     */
    class qMetric {

    public:
      // point-2-point ( cloud 2 cloud ), benchmark metric
      float c2c_rms;            //! store symm rms metric
      float c2c_hausdorff;      //! store symm haussdorf
      float c2c_psnr;
      float c2c_hausdorff_psnr; //! store symm haussdorf

      // point-2-plane ( cloud 2 plane ), proposed metric
      float c2p_rms;            //! store symm rms metric
      float c2p_hausdorff;      //! store symm haussdorf
      float c2p_psnr;
      float c2p_hausdorff_psnr; //! store symm haussdorf

      // point 2 plane ( cloud 2 plane ), proposed metric
      float maxDist;            //! maximum distnace between NN points in reference point cloud

      qMetric()
        {
          c2c_rms = 0; c2c_hausdorff = 0;
          c2p_rms = 0; c2p_hausdorff = 0;
        }
    };

    /*
     * \brief load a point cloud
     * \param[in] file_name: the name of the file to load
     * \param[out] cloud: the resultant templated point cloud
     */
    template<typename PointT> int readcloud(const string &file_name, PointCloud<PointT> &cloud)
    {
      int(*readfunc)( const string &, PointCloud<PointT> & );
      std::string suffix;
      suffix = file_name.substr(file_name.find_last_of(".") + 1);
      if (suffix == "pcd")
        readfunc = io::loadPCDFile;
      else if (suffix == "ply")
        readfunc = io::loadPLYFile;
      else
      {
        cerr << "Error: File " << file_name << " doesn't have a valid suffix" << endl;
        return -1;
      }

      return readfunc(file_name.c_str(), cloud);
    }

    /*
     * \brief write a point cloud into file
     * \param[in] file_name: the name of the file to load
     * \param[out] cloud: the resultant templated point cloud
     */
    template<typename PointT> int writecloud(const string &file_name, PointCloud<PointT> &cloud)
    {
      int(*savefunc)( const string &, const pcl::PointCloud<PointT> &, bool );
      string suffix;
      suffix = file_name.substr(file_name.find_last_of(".") + 1);
      if (suffix == "pcd")
      {
        savefunc = pcl::io::savePCDFile;
      }
      else if (suffix == "ply")
      {
        savefunc = pcl::io::savePLYFile;
      }
      else
      {
        cout << "Error: File " << file_name.c_str() << " doesn't have a valid suffix" << endl;
        return -1;
      }

      return savefunc(file_name.c_str(), cloud, false);
    }

    /**!
     * \function
     *   Compute the minimum and maximum NN distances, find out the
     *   intrinsic resolutions
     * \parameters
     *   @param cloudA: point cloud
     *   @param minDist: output
     *   @param maxDist: output
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    findNNdistances(PointCloud<PointT> &cloudA, float &minDist, float &maxDist)
    {
      maxDist =  numeric_limits<float>::min();
      minDist =  numeric_limits<float>::max();
      double distTmp = 0;
      mutex myMutex;
      search::KdTree<PointT> treeA;
      treeA.setInputCloud(cloudA.makeShared());

#pragma omp parallel for
      for (size_t i = 0; i < cloudA.points.size(); ++i)
      {
        std::vector<int> indices;
        std::vector<float> sqrDist;

        int nFound = treeA.nearestKSearch(cloudA.points[i], 2, indices, sqrDist);
        if ( nFound <= 0)
          cerr << "Error! No NN found!" << endl;

        if (indices[0] != i || sqrDist[1] <= 0.0000000001)
        {
          // Maybe print some warnings
           // cerr << "Error! nFound = " << nFound << ", i, iFound = " << i << ", " << indices[0] << ", " << indices[1] << endl;
           // cerr << "       Distances = " << sqrDist[0] << ", " << sqrDist[1] << endl;
           // cerr << "  Some points are repeated!" << endl;
        }

        else
        {
          // Use the second one. assume the first one is the current point
          myMutex.lock();
          distTmp = sqrt( sqrDist[1] );
          if (distTmp > maxDist)
            maxDist = distTmp;
          if (distTmp < minDist)
            minDist = distTmp;
          myMutex.unlock();
        }
      }
    }

    /**!
     * \function
     *   Convert the MSE error to PSNR numbers
     * \parameters
     *   @param cloudA:  the original point cloud
     *   @param dist: the distortion
     *   @param p: the peak value for conversion
     * \return
     *   psnr value
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> float
    getPSNR(PointCloud<PointT> &cloudA, float dist, float p)
    {
      // @DT: If bounding box is wanted for the peak value
      // PointT pMinA, pMaxA;
      // getMinMax3D( cloudA, pMinA, pMaxA );
      // metric.maxDist = pMaxA;

      float max_energy = p * p;
      float psnr = 10 * log10( max_energy / (dist*dist) );

      return psnr;
    }

    /**!
     * \function
     *   Check if meaningful normals exist.
     * \parameters
     *   @param cloudA:  the original point cloud
     * \return
     *   true: normals are available
     *   false: otherwise
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> bool
    checkNormalsAvailability(PointCloud<PointT> &cloudA)
    {
      size_t sz = cloudA.points.size();
      size_t i = 0;
      if (cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 )
        return true;
      i = sz - 1;
      if (cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 )
        return true;
      i = sz / 2 - 1;
      if (cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 && cloudA.at(i).normal_x != 0 )
        return true;
      return false;
    }

    /**!
     * \function
     *   Derive the normals for the decoded point cloud based on the
     *   normals in the original point cloud
     * \parameters
     *   @param cloudA:  the original point cloud
     *   @param cloudNormalsA: the normals in the original point cloud
     *   @param cloudB:  the decoded point cloud
     *   @param cloudNormalsB: the normals in the original point
     *     cloud. Output parameter
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    scaleNormals(PointCloud<PointT> &cloudA, PointCloud<Normal>::Ptr &cloudNormalsA, PointCloud<PointT> &cloudB, vector< vector<float> > &cloudNormalsB)
    {
      // Prepare the buffer to compute the average normals
      clock_t t1 = clock();

      vector< vector<int> > vecMap( cloudB.points.size() );

      for (size_t i = 0; i < cloudB.points.size(); i++)
      {
        cloudNormalsB[i].push_back(0.0); // x
        cloudNormalsB[i].push_back(0.0); // y
        cloudNormalsB[i].push_back(0.0); // z
        vecMap[i].clear();
      }

      // sum up
      search::KdTree<PointT> treeA;
      treeA.setInputCloud (cloudA.makeShared());
      search::KdTree<PointT> treeB;
      treeB.setInputCloud (cloudB.makeShared());

      for (size_t i = 0; i < cloudA.points.size(); i++)
      {
        // Find the NNs in cloudA
        vector<int> indices;
        vector<float> sqrDist;
        float nX, nY, nZ;
        int nCount;

        treeB.nearestKSearch(cloudA.points[i], 1, indices, sqrDist);
        nX = nY = nZ = 0.0;
        nCount = 0;

        if ( !isnan(cloudNormalsA->at(i).normal_x) && !isnan(cloudNormalsA->at(i).normal_y) && !isnan(cloudNormalsA->at(i).normal_z) )
        {
          cloudNormalsB[indices[0]][0] += cloudNormalsA->at( i ).normal_x;
          cloudNormalsB[indices[0]][1] += cloudNormalsA->at( i ).normal_y;
          cloudNormalsB[indices[0]][2] += cloudNormalsA->at( i ).normal_z;
          vecMap[ indices[0] ].push_back( i );
        }
      }

      // average now
      for (size_t i = 0; i < cloudB.points.size(); i++)
      {
        int nCount = vecMap[i].size();
        if (nCount > 0)      // main branch
        {
          cloudNormalsB[i][0] = cloudNormalsB[i][0] / nCount;
          cloudNormalsB[i][1] = cloudNormalsB[i][1] / nCount;
          cloudNormalsB[i][2] = cloudNormalsB[i][2] / nCount;
        }
        else
        {
          vector<int> indices;
          vector<float> sqrDist;
          treeA.nearestKSearch(cloudB.points[i], 1, indices, sqrDist);

          if ( !isnan(cloudNormalsA->at(indices[0]).normal_x) && !isnan(cloudNormalsA->at(indices[0]).normal_y) && !isnan(cloudNormalsA->at(indices[0]).normal_z) )
          {
            cloudNormalsB[i][0] = cloudNormalsA->at( indices[0] ).normal_x;
            cloudNormalsB[i][1] = cloudNormalsA->at( indices[0] ).normal_y;
            cloudNormalsB[i][2] = cloudNormalsA->at( indices[0] ).normal_z;
          }
          else
          {                     // Should never comes here. The code just for completeness
            cloudNormalsB[i][0] = 0;
            cloudNormalsB[i][1] = 0;
            cloudNormalsB[i][2] = 0;
          }
        }
      }

      clock_t t2 = clock();
      cout << "   Converting normal vector DONE. It takes " << (t2-t1)/CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
    }

    /**!
     * \function
     *   Get the normals for the original point cloud, either by importing
     *   or estimation
     * \parameters
     *   @param cloudA:  the original point cloud
     *   @param normFile: the file name to store the normals
     *   @param cPar: input parameter from command line
     *   @param cloudNormals: output paraemter for the normals
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    getNormals(PointCloud<PointT> &cloudA, string &normFile, commandPar &cPar, PointCloud<Normal>::Ptr cloudNormals)
    {
      if (!cPar.force && checkNormalsAvailability( cloudA ) )
      {
        cout << "   Import existing normal from input" << endl;
        copyPointCloud( cloudA, *cloudNormals );
        cout << "   Normal importing on original point cloud DONE!" << endl;
        return;
      }

      clock_t t1;
      t1=clock();

      if (cPar.knn == 0)
      {
        // Step 0 ------------------
        float minDist;
        float maxDist;
        findNNdistances(cloudA, minDist, maxDist);
        cout << "   Point cloud, to estimate normals: minDist, maxDist = " << minDist << ", " << maxDist << endl;

        cPar.radius = cPar.rtimes * minDist;
        cout << "   Radius in use: " << cPar.radius << endl;
      }
      else
      {
        cout << "   KNN in use: " << cPar.knn << endl;
      }
      cout << "   Normal estimation begin.." << endl;

      // Step 1 ------------------
      // @DT: Compute the normals of A, the reference point cloud
      // Create the normal estimation class, and pass the input dataset to it
      NormalEstimationOMP<PointXYZRGBNormal, Normal> ne;
      ne.setInputCloud(cloudA.makeShared());

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      search::KdTree<PointXYZRGBNormal>::Ptr tree (new search::KdTree<PointXYZRGBNormal>());
      ne.setSearchMethod(tree);

      // // Output datasets
      // PointCloud<Normal>::Ptr cloudNormals (new PointCloud<Normal>);

      if (cPar.knn == 0)
        // Use all neighbors in a sphere of radius
        ne.setRadiusSearch(cPar.radius);
      else
        // Use a fixed number of neighbors
        ne.setKSearch( cPar.knn );

      // Set view points
      Eigen::Vector4f centroid;
      centroid.setZero();
      compute3DCentroid( cloudA, centroid );
      PointT pMin, pMax;
      getMinMax3D( cloudA, pMin, pMax );
      ne.setViewPoint( centroid[0], centroid[1], pMax.z+1.0 );
      // cout << "Centroid: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << " size " << cloudA.size() << endl;
      // cout << "Min:      " << pMin.x << " " << pMin.y << " " << pMin.z << endl;
      // cout << "Max:      " << pMax.x << " " << pMax.y << " " << pMax.z << endl;

      // Compute the features
      ne.compute(*cloudNormals);

      if ( normFile != "" )
      {
        PointCloud<PointXYZRGBNormal>::Ptr cloudWithNormals(new PointCloud<PointXYZRGBNormal>);
        copyPointCloud( cloudA, *cloudWithNormals );
        copyPointCloud( *cloudNormals, *cloudWithNormals );
        writecloud(normFile, *cloudWithNormals);
      }

      // Check if any nan normals
      size_t nanNormal = 0;
      for (size_t i = 0; i < cloudA.points.size(); i++)
      {
        if ( isnan(cloudNormals->at(i).normal_x) || isnan(cloudNormals->at(i).normal_y) || isnan(cloudNormals->at(i).normal_z) )
          nanNormal++;
      }
      if (nanNormal > 0)
      {
        if (cPar.knn == 0)
          cerr << "   ** Warning: nan normals found: " << nanNormal << "! Increase the radius!" << endl;
        else
          cerr << "   ** Warning: nan normals found: " << nanNormal << "! Increase the knn!" << endl;
        // cout << "The points with nan normals would be excluded from metric calculation." << endl;
      }

      cout << "   Normal estimation on original point cloud DONE! It takes " << (clock() - t1) / CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
    }

    /**!
     * function to compute the symmetric quality metric: Point-to-Point and Point-to-Plane
     *   @param cloudA: point cloud, original version
     *   @param cloudB: point cloud, decoded/reconstructed version
     *   @param cPar: input parameters
     *   @param qual_metric: quality metric, to be returned
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    computeGeometricQualityMetric(PointCloud<PointT> &cloudA, PointCloud<PointT> &cloudB, commandPar &cPar, qMetric &qual_metric)
    {
      float minDist;
      float maxDist;
      findNNdistances(cloudA, minDist, maxDist);
      qual_metric.maxDist = maxDist;
      cout << "Minimum and maximum NN distances (intrinsic resolutions): " << minDist << ", " << maxDist << endl;

      // Check cloud size
      size_t orgSize = max( cloudA.points.size(), cloudB.points.size() );
      size_t newSize = min( cloudA.points.size(), cloudB.points.size() );
      float ratio = 1.0 * newSize / orgSize;
      cout << "Point cloud sizes for org version, dec version, and the scaling ratio: " << orgSize << ", " << newSize << ", " << ratio << endl;

      if (cPar.file2 == "" && cPar.normFile == "" ) // If no file2 & no normFile provided, return just after checking the NN
        return;

      // Estimate or import normals, only on original point cloud
      PointCloud<Normal>::Ptr cloudNormalsA (new PointCloud<Normal>);
      if (!cPar.c2c_only)
      {
        cout << endl;
        cout << "0. Preparing normals.\n";
        getNormals(cloudA, cPar.normFile, cPar, cloudNormalsA);
      }

      if (cPar.file2 == "")     // If no file2 provided, return just after normal estimations.
        return;

      // Based on normals on original point cloud, derive normals on reconstructed point cloud
      vector< vector<float> > cloudNormalsB( cloudB.points.size() );
      if (!cPar.c2c_only)
        scaleNormals( cloudA, cloudNormalsA, cloudB, cloudNormalsB );
      cout << endl;

      // Use "a" as reference
      cout << "1. Use infile1 (A) as reference, loop over A, use normals on B. (A->B).\n";
      qMetric metricA;
      metricA.maxDist = maxDist;
      findMetricA( cloudA, cloudB, cPar, cloudNormalsB, metricA );

      cout << "   ### A->B,rms1,p2point," << metricA.c2c_rms << endl;
      cout << "   ### A->B,rms1PSNR,p2point," << metricA.c2c_psnr << endl;
      if (!cPar.c2c_only)
      {
        cout << "   ### A->B,rms1,p2plane," << metricA.c2p_rms << endl;
        cout << "   ### A->B,rms1PSNR,p2plane," << metricA.c2p_psnr << endl;
      }
      if ( cPar.hausdorff )
      {
        cout << "   ### A->B,h1,p2point," << metricA.c2c_hausdorff << endl;
        cout << "   ### A->B,hPSNR1,p2point," << metricA.c2c_hausdorff_psnr << endl;
        if (!cPar.c2c_only)
        {
          cout << "   ### A->B,h1,p2plane," << metricA.c2p_hausdorff << endl;
          cout << "   ### A->B,hPSNR1,p2plane," << metricA.c2p_hausdorff_psnr << endl;
        }
      }

      if (!cPar.singlePass)
      {
        // Use "b" as reference
        cout << "2. Use infile2 (B) as reference, loop over B, use normals on A. (B->A).\n";
        qMetric metricB;
        metricB.maxDist = maxDist;
        findMetricB( cloudA, cloudB, cPar, cloudNormalsA, metricB );

        cout << "   ### B->A,rms2,p2point," << metricB.c2c_rms << endl;
        cout << "   ### B->A,rms2PSNR,p2point," << metricB.c2c_psnr << endl;
        if (!cPar.c2c_only)
        {
          cout << "   ### B->A,rms2,p2plane, " << metricB.c2p_rms << endl;
          cout << "   ### B->A,rms2PSNR,p2plane," << metricB.c2p_psnr << endl;
        }
        if ( cPar.hausdorff )
        {
          cout << "   ### B->A,h2,p2point," << metricB.c2c_hausdorff << endl;
          cout << "   ### B->A,hPSNR2,p2point," << metricB.c2c_hausdorff_psnr << endl;
          if (!cPar.c2c_only)
          {
            cout << "   ### B->A,h2,p2plane," << metricB.c2p_hausdorff << endl;
            cout << "   ### B->A,hPSNR2,p2plane," << metricB.c2p_hausdorff_psnr << endl;
          }
        }

        // Derive the final symmetric metric
        qual_metric.c2c_rms = max( metricA.c2c_rms, metricB.c2c_rms );
        qual_metric.c2p_rms = max( metricA.c2p_rms, metricB.c2p_rms );
        qual_metric.c2c_psnr = min( metricA.c2c_psnr, metricB.c2c_psnr );
        qual_metric.c2p_psnr = min( metricA.c2p_psnr, metricB.c2p_psnr );

        qual_metric.c2c_hausdorff = max( metricA.c2c_hausdorff, metricB.c2c_hausdorff	);
        qual_metric.c2p_hausdorff = max( metricA.c2p_hausdorff, metricB.c2p_hausdorff );
        qual_metric.c2c_hausdorff_psnr = min( metricA.c2c_hausdorff_psnr, metricB.c2c_hausdorff_psnr	);
        qual_metric.c2p_hausdorff_psnr = min( metricA.c2p_hausdorff_psnr, metricB.c2p_hausdorff_psnr );

        cout << "3. Final (symmetric).\n";
        cout << "   ### Symmetric,rmsF,p2point," << qual_metric.c2c_rms << endl;
        cout << "   ### Symmetric,rmsFPSNR,p2point," << qual_metric.c2c_psnr << endl;
        if (!cPar.c2c_only)
        {
          cout << "   ### Symmetric,rmsF,p2plane," << qual_metric.c2p_rms << endl;
          cout << "   ### Symmetric,rmsFPSNR,p2plane," << qual_metric.c2p_psnr << endl;
        }
        if ( cPar.hausdorff )
        {
          cout << "   ### Symmetric,hF,p2point," << qual_metric.c2c_hausdorff << endl;
          cout << "   ### Symmetric,hPSNRF,p2point," << qual_metric.c2c_hausdorff_psnr << endl;
          if (!cPar.c2c_only)
          {
            cout << "   ### Symmetric,hF,p2plane," << qual_metric.c2p_hausdorff << endl;
            cout << "   ### Symmetric,hPSNRF,p2plane," << qual_metric.c2p_hausdorff_psnr << endl;
          }
        }
      }
    }

    /**!
     * \function
     *   To compute "one-way" quality metric: Point-to-Point and
     *   Point-to-Plane. Loop over each point in A. Normals in B to be used
     *
     *   1) For each point in A, find a corresponding point in B.
     *   2) Form an error vector between the point pair.
     *   3) Use the length of the error vector as point-to-point measure
     *   4) Project the error vector along the normals in B, use the length
     *   of the projected error vector as point-to-plane measure
     *
     *   @param cloudA: Reference point cloud. e.g. the original cloud, on
     *     which normals would be estimated. It is the full set of point
     *     cloud. Multiple points in count
     *   @param cloudB: Processed point cloud. e.g. the decoded cloud
     *   @param cPar: Command line parameters
     *   @param cloudNormalsB: Normals for cloudB
     *   @param metric: updated quality metric, to be returned
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    findMetricA(PointCloud<PointT> &cloudA, PointCloud<PointT> &cloudB, commandPar &cPar, vector< vector<float> > &cloudNormalsB, qMetric &metric)
    {
      mutex myMutex;

      // @DT: Compute the projected distance along the normal direction (cloud 2 plane)
      clock_t t2 = clock();
      float max_dist_b_c2p = -std::numeric_limits<float>::max();
      double rms_dist_b_c2p = 0;
      float max_dist_b_c2c = -std::numeric_limits<float>::max();
      double rms_dist_b_c2c = 0;
      size_t num = 0;

      search::KdTree<PointT> treeB;
      treeB.setInputCloud (cloudB.makeShared());
#pragma omp parallel for
      for (size_t i = 0; i < cloudA.points.size(); i++)
      {
        // Find the nearest neighbor in B. store it in 'j'
        vector<int> indices(1);
        vector<float> sqrDist(1);
        treeB.nearestKSearch(cloudA.points[i], 1, indices, sqrDist);
        int j = indices[0];

        // Compute the error vector
        vector<float> errVector(3);
        errVector[0] = cloudA.points[i].x - cloudB.points[j].x;
        errVector[1] = cloudA.points[i].y - cloudB.points[j].y;
        errVector[2] = cloudA.points[i].z - cloudB.points[j].z;

        // Compute point-to-point, which should be equal to sqrt( sqrDist[0] )
        float distProj_c2c = sqrt( errVector[0] * errVector[0] +
                                   errVector[1] * errVector[1] +
                                   errVector[2] * errVector[2] );

        // Compute point-to-plane
        // Normals in B will be used for point-to-plane
        float distProj = 0.0;
        if (!cPar.c2c_only)
        {
          distProj = fabs( errVector[0] * cloudNormalsB[j][0] +
                           errVector[1] * cloudNormalsB[j][1] +
                           errVector[2] * cloudNormalsB[j][2] );
        }

        myMutex.lock();

        num++;
        // mean square distance
        rms_dist_b_c2c += distProj_c2c;
        if (distProj_c2c > max_dist_b_c2c)
          max_dist_b_c2c = distProj_c2c;
        if (!cPar.c2c_only)
        {
          rms_dist_b_c2p += distProj;
          if (distProj > max_dist_b_c2p)
            max_dist_b_c2p = distProj;
        }

        myMutex.unlock();
      }

      rms_dist_b_c2p = rms_dist_b_c2p / num;
      rms_dist_b_c2c = rms_dist_b_c2c / num;

      metric.c2p_rms = rms_dist_b_c2p;
      metric.c2c_rms = rms_dist_b_c2c;
      metric.c2p_hausdorff = max_dist_b_c2p;
      metric.c2c_hausdorff = max_dist_b_c2c;

      // from distance to PSNR. cloudA always the original
      metric.c2c_psnr = getPSNR( cloudA, metric.c2c_rms, metric.maxDist );
      metric.c2p_psnr = getPSNR( cloudA, metric.c2p_rms, metric.maxDist );

      metric.c2c_hausdorff_psnr = getPSNR( cloudA, metric.c2c_hausdorff, metric.maxDist );
      metric.c2p_hausdorff_psnr = getPSNR( cloudA, metric.c2p_hausdorff, metric.maxDist );

      clock_t t3 = clock();
      cerr << "   Error computing takes " << (t3-t2)/CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
    }

    /**!
     * \function
     *   To compute "one-way" quality metric: Point-to-Point and
     *   Point-to-Plane. Loop over each point in B. Normals in A to be used
     *
     *   1) For each point in B, find a corresponding point in A.
     *   2) Form an error vector between the point pair.
     *   3) Use the length of the error vector as point-to-point measure
     *   4) Project the error vector along the normals in A, use the length
     *   of the projected error vector as point-to-plane measure
     *
     *   @param cloudA: Reference point cloud. e.g. the original cloud, on
     *     which normals would be estimated. It is the full set of point
     *     cloud. Multiple points in count
     *   @param cloudB: Processed point cloud. e.g. the decoded cloud
     *   @param cPar: Command line parameters
     *   @param cloudNormalsA: Normals for cloudA
     *   @param metric: updated quality metric, to be returned
     * \note
     *   PointT typename of point used in point cloud
     * \author
     *   Dong Tian, MERL
     */
    template<typename PointT> void
    findMetricB(PointCloud<PointT> &cloudA, PointCloud<PointT> &cloudB, commandPar &cPar, PointCloud<Normal>::Ptr &cloudNormalsA, qMetric &metric)
    {
      mutex myMutex;

      clock_t t2 = clock();
      float max_dist_b_c2p = -std::numeric_limits<float>::max();
      double rms_dist_b_c2p = 0;
      float max_dist_b_c2c = -std::numeric_limits<float>::max();
      double rms_dist_b_c2c = 0;
      size_t num = 0;

      search::KdTree<PointT> treeA;
      treeA.setInputCloud (cloudA.makeShared());
#pragma omp parallel for
      for (size_t i = 0; i < cloudB.points.size(); i++)
      {
        // Find the nearest neighbor in A. store it in 'j'
        vector<int> indices(1);
        vector<float> sqrDist(1);
        treeA.nearestKSearch(cloudB.points[i], 1, indices, sqrDist);
        int j = indices[0];

        // Compute the error vector
        vector<float> errVector(3);
        errVector[0] = cloudB.points[i].x - cloudA.points[j].x;
        errVector[1] = cloudB.points[i].y - cloudA.points[j].y;
        errVector[2] = cloudB.points[i].z - cloudA.points[j].z;

        // Compute point-to-point, which should be equal to sqrt( sqrDist[0] )
        float distProj_c2c = sqrt( errVector[0] * errVector[0] +
                                   errVector[1] * errVector[1] +
                                   errVector[2] * errVector[2] );

        // Compute point-to-plane
        // Normals in A will be used for point-to-plane
        float distProj;
        if (!cPar.c2c_only)
        {
          distProj = fabs( errVector[0] * cloudNormalsA->at(j).normal_x +
                           errVector[1] * cloudNormalsA->at(j).normal_y +
                           errVector[2] * cloudNormalsA->at(j).normal_z );
        }

        myMutex.lock();

        num++;
        // mean square distance
        rms_dist_b_c2c += distProj_c2c;
        if (distProj_c2c > max_dist_b_c2c)
          max_dist_b_c2c = distProj_c2c;
        if (!cPar.c2c_only)
        {
          rms_dist_b_c2p += distProj;
          if (distProj > max_dist_b_c2p)
            max_dist_b_c2p = distProj;
        }

        myMutex.unlock();
      }

      rms_dist_b_c2p = rms_dist_b_c2p / num;
      rms_dist_b_c2c = rms_dist_b_c2c / num;

      metric.c2p_rms = rms_dist_b_c2p;
      metric.c2c_rms = rms_dist_b_c2c;
      metric.c2p_hausdorff = max_dist_b_c2p;
      metric.c2c_hausdorff = max_dist_b_c2c;

      // from distance to PSNR. cloudA always the original
      metric.c2c_psnr = getPSNR( cloudA, metric.c2c_rms, metric.maxDist );
      metric.c2p_psnr = getPSNR( cloudA, metric.c2p_rms, metric.maxDist );
      metric.c2c_hausdorff_psnr = getPSNR( cloudA, metric.c2c_hausdorff, metric.maxDist );
      metric.c2p_hausdorff_psnr = getPSNR( cloudA, metric.c2p_hausdorff, metric.maxDist );

      clock_t t3 = clock();
      cerr << "   Error computing takes " << (t3-t2)/CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
    }

  };

}   //~ namespace pcl

#endif
