#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;
int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile(argv[1], *cloud_in) != 0)
  {
    return -1;
  }

  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile(argv[2], *cloud_out) != 0)
  {
    return -1;
  }
  
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (1.0);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (50000);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  icp.align(Final);

  string filename = argv[3];
  if (pcl::io::savePCDFile(filename, Final, true) == 0)
  {
    cout << "Saved " << filename << "." << endl;
  }
  else PCL_ERROR("Problem saving %s.\n", filename.c_str());

  // pcl::PointCloud<pcl::PointXYZRGB> Output;
  // Output = *cloud_in;
  // Output += Final;
  // Output += *cloud_out;

  // if (pcl::io::savePCDFile(filename, Output, true) == 0)
  // {
  //   cout << "Saved " << filename << "." << endl;
  // }
  // else PCL_ERROR("Problem saving %s.\n", filename.c_str());

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}