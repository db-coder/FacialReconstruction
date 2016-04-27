#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

using namespace std; 
int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> aligned_cloud;
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors1(new pcl::PointCloud<pcl::FPFHSignature33>());
 
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *target_points) != 0)
	{
		return -1;
	}
 
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.
 
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
 
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.05);
 
	fpfh.compute(*descriptors);

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation1;
	normalEstimation1.setInputCloud(target_points);
	normalEstimation1.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation1.setSearchMethod(kdtree1);
	normalEstimation1.compute(*normals1);
 
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh1;
	fpfh1.setInputCloud(target_points);
	fpfh1.setInputNormals(normals1);
	fpfh1.setSearchMethod(kdtree1);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh1.setRadiusSearch(0.05);
 
	fpfh1.compute(*descriptors1);

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac;
	//Provide a pointer to the input point cloud and features
	sac.setInputSource(cloud);
	sac.setSourceFeatures(descriptors);
	//Provide a pointer to the target point cloud and features
	sac.setInputTarget(target_points);
	sac.setTargetFeatures(descriptors1);
	//Align input to target to obtain
	sac.align (aligned_cloud);
	//Eigen::Matrix4f transformation = sac.
	//getFinalTransformation ();

	string filename = argv[3];
  	if (pcl::io::savePCDFile(filename, aligned_cloud, true) == 0)
    {
    	cout << "Saved " << filename << "." << endl;
  	}
  	else PCL_ERROR("Problem saving %s.\n", filename.c_str());
}