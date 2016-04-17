#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

using namespace std;

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile(argv[1], *cloud) != 0)
	{
		return -1;
	}
 
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	// Filter out all points with Z values not in the [0-2] range.

	// filter.setFilterFieldName("x");
	// filter.setFilterLimits(-0.1, 0.1);

	// filter.setFilterFieldName("y");
	// filter.setFilterLimits(-0.4, 0.05);
 
	filter.setFilterFieldName("z");
	filter.setFilterLimits(-0.3, 1.0);
	
	filter.filter(*filteredCloud);

	stringstream stream;
	stream << "inputCloud_matty6_trimmed.pcd";
	string filename = stream.str();
	if (pcl::io::savePCDFile(filename, *filteredCloud, true) == 0)
	{
		cout << "Saved " << filename << "." << endl;
	}
	else PCL_ERROR("Problem saving %s.\n", filename.c_str());

}