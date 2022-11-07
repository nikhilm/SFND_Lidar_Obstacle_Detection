/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    auto randomPointIndex = [&]() {
        return (int) (((float)rand() / (float)RAND_MAX) * cloud->points.size());
    };

	// For max iterations
    for (int iteration = 0; iteration < maxIterations; iteration++) {
        std::unordered_set<int> currentInliersResult;
        int index1 = randomPointIndex();
        int index2 = randomPointIndex();
        // Pick 2 points.
        // Randomly sample subset and fit line
        pcl::PointXYZ point1 = cloud->points[index1];
        pcl::PointXYZ point2 = cloud->points[index2];

        const float coeffA = point1.y - point2.y;
        const float coeffB = point2.x - point1.y;
        const float coeffC = point1.x * point2.y - point1.y * point2.x;

        const float distance_normalizer = sqrt(coeffA * coeffA + coeffB * coeffB);
        float scaled_distance = distanceTol * distance_normalizer;

        for (int pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++) {
            // Measure distance between every point and fitted line
            // If distance is smaller than threshold count it as inlier
            pcl::PointXYZ point = cloud->points[pointIndex];
            float distance = abs(coeffA * point.x + coeffB * point.y + coeffC);
            if (distance <= scaled_distance) {
                currentInliersResult.insert(pointIndex);
            }
        }

        if (currentInliersResult.size() > inliersResult.size()) {
            inliersResult = std::move(currentInliersResult);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto delta = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cerr << "RANSAC " << maxIterations << " took " << delta.count() << "us\n";

    // NOTE(nikhilm): If 2 lines have equal inliers, this one WON'T choose the one where the mean square error is lower or similar.
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spin();
  	}
  	
}
