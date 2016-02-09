#pragma once

/*
Returns random double in range 0..1
*/
double random01(void);

/*
Reloads original rgb pointcloud to output pointcloud
*/
int reloadOutputPointcloud(void);

/**
* Loads pcd file from first argument argv[].
*
*/
template<typename PointT> inline int
loadPCDFileFromArgument(int argc, char* argv[], pcl::PointCloud<PointT> &output);


/*
K-nearest neighbours filter
\brief Set the standard deviation multiplier for the distance threshold calculation.
* \details The distance threshold will be equal to: mean + stddev_mult * stddev.
* Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
* \param[in] stddev_mult The standard deviation multiplier.
\param[in] meanK The number of points to use for mean distance estimation.

void filter(pcl::PointCloud<PointXYZ>::Ptr input,
	pcl::PointCloud<PointXYZ>::Ptr output,
	int meanK,
	double StddevMulThresh
	);
	*/
/*
Toggles original point cloud or g_Cloud_output in Viewer
*/
void static toggleOriginal(void* viewer_void);
