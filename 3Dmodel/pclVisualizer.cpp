
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <filesystem>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/geometry/planar_polygon.h>



int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

	
	
	boost::filesystem::path full_path(boost::filesystem::current_path());
	std::cout << "Current path is : " << full_path << std::endl;
	std::string filepath = "test_pcd.pcd";
	if (argc >= 2) {
		if (boost::filesystem::exists(argv[1])) //make sure it's actually a file
		{
			filepath = argv[1];
		}
		else {
			std::cerr << "Argument is not file";
		}
	}
	std::cout << "loading " << filepath << ".pcd" << std::endl;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, *cloud) == -1) //* load the file
	{
		std::cout << filepath;
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}


	pcl::copyPointCloud(*cloud, *cloud_xyz);

	//planes

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_xyz);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01); //0.01

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
	PCL_ERROR("Could not estimate a planar model for the given dataset.");
	return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
	<< coefficients->values[1] << " "
	<< coefficients->values[2] << " "
	<< coefficients->values[3] << std::endl;
	
	// Create the filtering object
#if 0
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

#endif // 0

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size() << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(4.0);
	chull.reconstruct(*cloud_hull);

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;
	/*
	*	VISUALIZATION
	*/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int v1(0);
	int v2(0);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("colored", 10, 10, "v1 text", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("filtered", 10, 10, "v2 text", v2);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "colored_cloud", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_projected, "ransac_cloud", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");

	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "filtered_cloud", v2);
	//viewer->addPlane(*coefficients, "prvni_rovina", 0);
	boost::shared_ptr<pcl::PlanarPolygon<pcl::PointXYZ>> poly(new pcl::PlanarPolygon<pcl::PointXYZ>());
	poly->setCoefficients(*coefficients);
	poly->setContour(*cloud_hull);
	viewer->addPolygon(*poly, 255, 0, 0, "stena", 0);
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
