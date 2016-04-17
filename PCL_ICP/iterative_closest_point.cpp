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
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.align(Final);

  stringstream stream;
  stream << "output1Cloud_malav5.pcd";
  string filename = stream.str();
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