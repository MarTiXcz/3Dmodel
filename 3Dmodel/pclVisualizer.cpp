
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <filesystem>
#include <string>
#include <boost/random.hpp>
#include <time.h>

#include "stdafx.h"
#include "pclVisualizer.h"

using namespace boost;
using namespace pcl;
using namespace::std;

//global variables

PointCloud<PointXYZ>::Ptr g_Cloud_output(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr g_CloudRGB(new PointCloud<PointXYZRGB>);


double random01(void) {
	boost::random::mt19937 rng;
	static boost::uniform_01<boost::mt19937> rn(rng);
	return rn();
}


/**
* Loads pcd file from first argument argv[].
* 
*/
template<typename PointT> inline int
loadPCDFileFromArgument(int argc, char* argv[], PointCloud<PointT> &output) {
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

	if (pcl::io::loadPCDFile<PointT>(filepath, output) == -1) //* load the file
	{
		std::cout << filepath;
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}
	return 0;
}

/*
Reloads original rgb pointcloud to output pointcloud
*/
int reloadOutputPointcloud(void) {
	//g_Cloud_output.reset();
	copyPointCloud(*g_CloudRGB, *g_Cloud_output);
	return 0;
}

void static toggleOriginal(void* viewer_void) {
	static bool init = false;
	static int view = 0;
	static PointCloud<PointXYZ>::Ptr cloud_original(new PointCloud<PointXYZ>);

	static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (!init) {
		copyPointCloud(*g_CloudRGB, *cloud_original);
		init = true;
		viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	}
	switch (view) {
	case 0: {
		viewer->updatePointCloud(cloud_original,"cloud_output");
		view = 1;
		break;
	}
	case 1: {
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
		view = 0;
		break;
	}

	}


}


void filter(pcl::PointCloud<PointXYZ>::Ptr input,
	pcl::PointCloud<PointXYZ>::Ptr output,
	int meanK,
	double StddevMulThresh
	) {
	//Enables using the same pointcloud as input and output
	PointCloud<PointXYZ>::Ptr tmp_cloud(new PointCloud<PointXYZ>);

	
	// Create the filtering object
	StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(input);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(StddevMulThresh);
	sor.filter(*tmp_cloud);
	copyPointCloud(*tmp_cloud,*output );

}

void filterZeroPosition(pcl::PointCloud<PointXYZRGB>::Ptr input,
	pcl::PointCloud<PointXYZRGB>::Ptr output
	) {
	//Enables using the same pointcloud as input and output
	PointCloud<PointXYZRGB>::Ptr tmp_cloud(new PointCloud<PointXYZRGB>);
	
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(g_CloudRGB);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 0.001);
	pass.setFilterLimitsNegative(true);
	pass.filter(*tmp_cloud);
	copyPointCloud(*tmp_cloud, *output);

}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym() == "Home" && event.keyDown())
	{
		std::cout << "home was pressed => reloading original cloud to output" << std::endl;
		reloadOutputPointcloud();
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
	}
	
	if (event.isCtrlPressed() && event.getKeySym() == "F1" && event.keyDown()) {
		std::cout << "CTRL + F1 was pressed => filtering output cloud" << std::endl;
		int meanK;

		double StddevMulThresh;
		cout << "Enter meanK(integer):";
		cin >> meanK;
		cout << "Enter StddevMulThresh(double):";
		cin >> StddevMulThresh;
		cout << endl;

		cout << "Working...";
		filter(g_Cloud_output, g_Cloud_output, meanK, StddevMulThresh);
		cout << "Done." << endl;
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
	} else if (event.getKeySym() == "F1" && event.keyDown()) {
		std::cout << "F1 was pressed => filtering output cloud" << std::endl;
		cout << "Working...";
		filter(g_Cloud_output, g_Cloud_output, 10, 1.0);
		cout << "Done." << endl;
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
	}
	if (event.getKeySym() == "Prior" && event.keyDown()) {
		std::cout << "PgUp was pressed => toggling original cloud" << std::endl;
		toggleOriginal(viewer_void);
	}

	//cout << event.getKeySym() << " " << event.getKeyCode() << endl;
}

int main(int argc, char* argv[])
{
	PointCloud<PointXYZ>::Ptr cloud_xyz(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_projected(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_minus_plane(new PointCloud<PointXYZ>);

	
	loadPCDFileFromArgument(argc, argv, *g_CloudRGB);
	
	filterZeroPosition(g_CloudRGB, g_CloudRGB);


	//copy pointCloud without rgb
	copyPointCloud(*g_CloudRGB, *cloud_xyz);

	//planes

#if 0
			// Create the filtering object
	StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(cloud_xyz);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
#else
	copyPointCloud(*cloud_xyz, *cloud_filtered);
#endif // 0

	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	PointIndices::Ptr inliers(new PointIndices);
	// Create the segmentation object
	SACSegmentation<PointXYZ> seg;
	// Create the filtering object
	ExtractIndices<PointXYZ> extract;
	// Project the model inliers
	ProjectInliers<PointXYZ> proj;
	// Create a Concave Hull representation of the projected inliers
	PointCloud<PointXYZ>::Ptr cloud_hull(new PointCloud<PointXYZ>);
	ConcaveHull<PointXYZ> chull;
	//polygon

	
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	

	
	/*
	*	VISUALIZATION
	*/
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	int v1(0);
	int v2(0);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("colored", 10, 10, "v1 text", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("filtered", 10, 10, "v2 text", v2);

	/*
	* Segmentatiton
	

	int numOfPlanes = 2;
	for (int i = 1; i <= numOfPlanes; i++)
	{
		seg.setDistanceThreshold(0.03); //0.01
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			PCL_ERROR("Could not estimate a planar model for the given dataset.");
			return (-1);
		}


		proj.setModelType(SACMODEL_PLANE);
		proj.setIndices(inliers);
		proj.setInputCloud(cloud_filtered);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		std::cerr << "PointCloud after projection has: "
			<< cloud_projected->points.size() << " data points." << std::endl;


		chull.setInputCloud(cloud_projected);
		chull.setAlpha(4.0); //4 for wall
		chull.reconstruct(*cloud_hull);

		std::cerr << "Concave hull has: " << cloud_hull->points.size()
			<< " data points." << std::endl;

		if (i <= numOfPlanes) {
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_minus_plane);
		}
		cloud_filtered.swap(cloud_minus_plane);
		boost::shared_ptr<PlanarPolygon<PointXYZ>> poly(new PlanarPolygon<PointXYZ>());
		poly->setCoefficients(*coefficients);
		poly->setContour(*cloud_hull);
		double r = random01();
		double g = random01();
		double b = random01();
		viewer->addPolygon(*poly, r, g, b, "stena" + i, v2);
		
	}
	*/
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(g_CloudRGB);
	viewer->addPointCloud<PointXYZRGB>(g_CloudRGB, rgb, "colored_cloud", v1);
	//viewer->addPointCloud<PointXYZ>(cloud_projected, "ransac_cloud", 0);
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");

	viewer->addPointCloud<PointXYZ>(cloud_filtered, "cloud_output", v2);
	//viewer->addPlane(*coefficients, "prvni_rovina", 0);

	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
