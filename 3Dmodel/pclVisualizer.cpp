
#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <string>
#include <boost/random.hpp>
#include <time.h>

//precompiled headers
#include "stdafx.h"

#include "pclVisualizer.h"

using namespace boost;
using namespace pcl;
using namespace::std;

//global variables

PointCloud<PointXYZ>::Ptr g_Cloud_output(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr g_Cloud_previous(new PointCloud<PointXYZ>);
string g_lastShape;
PointCloud<PointXYZRGB>::Ptr g_CloudRGB(new PointCloud<PointXYZRGB>);
int viewPort1 =0;
int viewPort2 =0;

double random01(void) {
	static boost::random::mt19937 rng;
	rng.seed(time(nullptr));
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
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.0, 0.0);
	pass.setFilterLimitsNegative(true);
	pass.filter(*tmp_cloud);
	copyPointCloud(*tmp_cloud, *output);

}

int findPlane(pcl::PointCloud<PointXYZ>::Ptr input,
	pcl::PointCloud<PointXYZ>::Ptr output,
	boost::shared_ptr<PlanarPolygon<PointXYZ>> poly,
	double distanceThreshold
	) {
	//Enables using the same pointcloud as input and output
	PointCloud<PointXYZ>::Ptr tmp_cloud(new PointCloud<PointXYZ>);

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

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);

	seg.setDistanceThreshold(distanceThreshold); //0.01
	seg.setInputCloud(input);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}


	proj.setModelType(SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(input);
	proj.setModelCoefficients(coefficients);
	proj.filter(*tmp_cloud);
	std::cerr << "PointCloud after projection has: "
		<< tmp_cloud->points.size() << " data points." << std::endl;

	chull.setInputCloud(tmp_cloud);
	chull.setAlpha(1.0); //4 for wall
	chull.reconstruct(*cloud_hull);

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;

	
	extract.setInputCloud(input);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*tmp_cloud);
	copyPointCloud(*tmp_cloud, *output);


	poly->setCoefficients(*coefficients);
	poly->setContour(*cloud_hull);

	return 0;
}


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	static int planeNumber = 0;
	if (event.getKeySym() == "Home" && event.keyDown())
	{
		std::cout << "home was pressed => reloading original cloud to output" << std::endl;
		reloadOutputPointcloud();
		viewer->removeAllShapes(viewPort2);
		g_lastShape.clear();
		g_Cloud_previous->clear();
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
	}
	if (event.getKeySym() == "BackSpace" && event.keyDown())
	{
		std::cout << "Backspace was pressed => reloading original cloud to output" << std::endl;
		if (!g_lastShape.empty()) {
			viewer->removeShape(g_lastShape, viewPort2);
			g_lastShape.clear();
		}
		if (!g_Cloud_previous->empty() ) {
			copyPointCloud(*g_Cloud_previous, *g_Cloud_output);
			viewer->updatePointCloud(g_Cloud_output, "cloud_output");
			g_Cloud_previous->clear();
		}
	}
	//filter
	if (event.isCtrlPressed() && event.getKeySym() == "F1" && event.keyDown()) {
		copyPointCloud(*g_Cloud_output, *g_Cloud_previous);

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
		copyPointCloud(*g_Cloud_output, *g_Cloud_previous);

		std::cout << "F1 was pressed => filtering output cloud" << std::endl;
		cout << "Working...";
		filter(g_Cloud_output, g_Cloud_output, 10, 5.0);
		cout << "Done." << endl;
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
	}
	//planes
	if (event.isCtrlPressed() && event.getKeySym() == "F2" && event.keyDown()) {
		boost::shared_ptr<PlanarPolygon<PointXYZ>> poly(new PlanarPolygon<PointXYZ>());
		copyPointCloud(*g_Cloud_output, *g_Cloud_previous);
		double distThresh;

		std::cout << "F2 was pressed => filtering output cloud" << std::endl;
		cout << "Enter distance threshold(double)(0.03):";
		cin >> distThresh;
		cout << endl;
		cout << "Working..." << endl;
		findPlane(g_Cloud_output, g_Cloud_output, poly, distThresh);
		cout << "Done." << endl;
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
		auto r = random01();
		auto g = random01();
		auto b = random01();
		viewer->addPolygon(*poly, r, g, b, "stena" + planeNumber, viewPort2);
		g_lastShape = "stena" + planeNumber;
		planeNumber++;
	} else if (event.getKeySym() == "F2" && event.keyDown()) {
		boost::shared_ptr<PlanarPolygon<PointXYZ>> poly(new PlanarPolygon<PointXYZ>());
		copyPointCloud(*g_Cloud_output, *g_Cloud_previous);
		
		std::cout << "F2 was pressed => filtering output cloud" << std::endl;
		cout << "Working..." << endl;
		findPlane(g_Cloud_output, g_Cloud_output, poly, 0.03);
		cout << "Done." << endl;
		viewer->updatePointCloud(g_Cloud_output, "cloud_output");
		auto r = random01();
		auto g = random01();
		auto b = random01();
		viewer->addPolygon(*poly, r, g, b, "stena" + planeNumber, viewPort2);
		g_lastShape = "stena" + planeNumber;
		planeNumber++;
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
	//passthrough - removes points in x=0 (points in [0,0,0]) 
	filterZeroPosition(g_CloudRGB, g_CloudRGB);


	//copy pointCloud without rgb
	copyPointCloud(*g_CloudRGB, *g_Cloud_output);


	/*
	*	VISUALIZATION
	*/
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewPort1);
	viewer->setBackgroundColor(0, 0, 0, viewPort1);
	viewer->addText("colored", 10, 10, "v1 text", viewPort1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewPort2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, viewPort2);
	viewer->addText("filtered", 10, 10, "v2 text", viewPort2);

	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(g_CloudRGB);
	viewer->addPointCloud<PointXYZRGB>(g_CloudRGB, rgb, "colored_cloud", viewPort1);
	//viewer->addPointCloud<PointXYZ>(cloud_projected, "ransac_cloud", 0);
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");

	viewer->addPointCloud<PointXYZ>(g_Cloud_output, "cloud_output", viewPort2);
	//viewer->addPlane(*coefficients, "prvni_rovina", 0);

	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
