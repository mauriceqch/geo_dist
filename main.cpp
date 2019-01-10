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

#ifdef _WIN32
#include "stdafx.h"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <sstream>
#include <boost/program_options.hpp>

#include "geometric_distortion.h"

using namespace boost::program_options;
using namespace pcl::geometric_quality;

void printusage()
{
  cout << "pc_psnr cloud_a cloud_b [radiusTimes]" << endl;
  cout << "  default radiusTimes is 10" << endl;
}

int parseCommand( int ac, char * av[], commandPar &cPar )
{
  try {
    options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "produce help message")
      ("fileA,a", value(&cPar.file1)->required(), "Input file 1") // value< string >
      ("fileB,b", value(&cPar.file2)->default_value(""), "Input file 2")
      ("saveNorm,A", value(&cPar.normFile)->default_value(""), "File name to output the normals of original point cloud")
      ("knn,k", value(&cPar.knn)->default_value(12), "Set KNN number of neighbor points for normal estimation. Default normal estimation method")
      ("rtimes,t", value(&cPar.rtimes)->default_value(0), "Guide to set radius for normal estimation, times to NN points distances")
      ("forceNormalEstimation,f", bool_switch(&cPar.force), "Force normal estimation even normals are provided")
      ("singlePass,s", bool_switch(&cPar.singlePass)->default_value(false), "Force running a single pass, where the loop is over the original point cloud")
      ("hausdorff,d", bool_switch(&cPar.hausdorff)->default_value(false), "Send the Haursdorff metric as well")
      ("point2point,1", bool_switch(&cPar.c2c_only)->default_value(false), "Force to report point-to-point metric only")
      ;

    // positional_options_description p;
    // p.add("rtimes", -1);
    variables_map vm;
    store(parse_command_line(ac, av, desc), vm);

    if (ac == 1 || vm.count("help")) { // @DT: Important too add ac == 1
      cout << "Usage: " << av[0] << " [options]\n";
      cout << desc;
      return 0;
    }

    notify(vm);                 // @DT: Report any missing parameters

    // It is wierd the variables were not set. Force the job
    cPar.file1 = vm["fileA"].as< string >();
    cPar.file2 = vm["fileB"].as< string >();
    cPar.normFile = vm["saveNorm"].as< string >();
    cPar.rtimes = vm["rtimes"].as< float >();
    if (cPar.rtimes == 0)
      cPar.knn = vm["knn"].as< int >();
    else
      cPar.knn = 0;
    cPar.singlePass = vm["singlePass"].as< bool >();
    cPar.c2c_only = vm["point2point"].as< bool >();

    // Safety check
    if (cPar.knn < 0)
    {
      cout << "Error: knn must be non-negative" << endl;
      return 0;
    }

    return 1;
  }

  catch(std::exception& e)
  {
    cout << e.what() << "\n";
    return 0;
  }

  // Confict check
  
}

void printCommand( commandPar &cPar )
{
  cout << "infile1: " << cPar.file1 << endl;
  cout << "infile2: " << cPar.file2 << endl;

  if ( cPar.knn > 0 )
    cout << "knn = " << cPar.knn << endl;
  else
    cout << "rtimes = " << cPar.rtimes << endl;

  if (cPar.normFile != "")
    cout << "save normals of original point cloud to: " << cPar.normFile << endl;
  cout << "force normal estimation: " << cPar.force << endl;
  if (cPar.singlePass)
    cout << "force running a single pass" << endl;

  cout << endl;
}

int main (int argc, char *argv[])
{
  commandPar cPar;
  if ( parseCommand( argc, argv, cPar ) == 0 )
    return 0;

  printCommand( cPar );
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr incloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr incloud2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);


  if (readcloud(cPar.file1, *incloud1))
  {
    cout << "Error reading " << cPar.file1 << endl;
    return -1;
  }
  cout << "Reading file 1 done." << endl;

  if (cPar.file2 != "")
  {
    if (readcloud(cPar.file2, *incloud2))
    {
      cout << "Error reading " << cPar.file2 << endl;
      return -1;
    }
    cout << "Reading file 2 done." << endl;
  }

  // compute the point to plane distances, as well as point to point distances
  struct timeval t1, t2;
  gettimeofday(&t1, NULL);
  pcl::geometric_quality::qMetric qm;
  pcl::geometric_quality::computeGeometricQualityMetric(*incloud1, *incloud2, cPar, qm);
  gettimeofday(&t2, NULL);

  cout << "Job done! " << (t2.tv_sec  - t1.tv_sec) << " seconds elapsed (excluding the time to load the point clouds)." << endl;
  return 0;
}
