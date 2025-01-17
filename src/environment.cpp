/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    auto lidar = std::make_unique<Lidar>(cars, 0);
    auto cloud = lidar->scan();
//    renderRays(viewer, lidar->position, cloud);

//    renderPointCloud(viewer, cloud, "original");
    auto pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();
    const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &obstacle_and_plane = pointProcessor->SegmentPlaneCustom(
            cloud, 1000, 0.2);
    renderPointCloud(viewer, obstacle_and_plane.first, "obstacles", Color(1.0, 0, 0));
    renderPointCloud(viewer, obstacle_and_plane.second, "plane", Color(0, 0, 1.0));

    auto clusters = pointProcessor->Clustering(obstacle_and_plane.first, 1.0, 3, 30);
    int clusterId = 0;
    for (const auto &cluster: clusters) {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                         Color(0, (255 - clusterId * 2) / 255.0, 0));
        Box bbox = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, bbox, clusterId);
        clusterId++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

//    renderPointCloud(viewer,inputCloud,"inputCloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, .2, Eigen::Vector4f(-10, -8, -2, 0), Eigen::Vector4f(20, 8, 2, 0));
//    renderPointCloud(viewer, filterCloud, "filterCloud");
    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> &obstacle_and_plane = pointProcessorI->SegmentPlaneCustom(
            filterCloud, 1000, 0.2);
//    renderPointCloud(viewer, obstacle_and_plane.first, "obstacles", Color(1.0, 0, 0));
    renderPointCloud(viewer, obstacle_and_plane.second, "plane", Color(0, 0, 1.0));
    auto clusters = pointProcessorI->ClusteringCustom(obstacle_and_plane.first, .25, 3, 500);
    int clusterId = 0;
    std::array<Color,3> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (const auto &cluster: clusters) {
//        std::cout << "cluster size ";
//        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        Box bbox = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, bbox, clusterId);
        clusterId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}