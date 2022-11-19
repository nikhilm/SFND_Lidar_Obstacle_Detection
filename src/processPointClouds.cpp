// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>
#include "kdtree3d.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filtered_cloud);

    typename pcl::PointCloud<PointT>::Ptr roi_cloud(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*roi_cloud);

    // Remove roof points.
    std::vector<int> roof_indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.4, -1.5, -1.0, 0));
    roof.setMax(Eigen::Vector4f(2.7, 1.5, 0.2, 0));
    roof.setInputCloud(roi_cloud);
    roof.filter(roof_indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (const auto &index : roof_indices) {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(roi_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
     extract.filter(*roi_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return roi_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices)
        plane->points.push_back(cloud->points[index]);

    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract_obstacles;
    extract_obstacles.setInputCloud(cloud);
    extract_obstacles.setIndices(inliers);
    extract_obstacles.setNegative(true);
    extract_obstacles.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset\n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " us" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    std::unordered_set<int> inliersResult;

    auto randomPointIndex = [&]() {
        return (int) (rand() % cloud->points.size());
    };

    for (int i = 0; i < maxIterations; ++i) {
        std::unordered_set<int> currentInliersResult;
        int index1 = randomPointIndex();
        int index2 = randomPointIndex();
        int index3 = randomPointIndex();
        if(index1 == index2 || index2 == index3 || index1 == index3) {
            continue;
        }
        // Pick 3 points.
        PointT point1 = cloud->points[index1];
        PointT point2 = cloud->points[index2];
        PointT point3 = cloud->points[index3];
//        std::cerr << "Point 1 " << point1 << " Point 2 " << point2 << " Point 3 " << point3 << "\n";

        // A plane can be defined by its normal.
        // Vect3 doesn't define cross-product, so we do it ourselves.
        const float coeffA = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        const float coeffB = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        const float coeffC = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        const float coeffD = - (coeffA * point1.x + coeffB * point1.y + coeffC * point1.z);

        const float distance_normalizer = sqrt(coeffA * coeffA + coeffB * coeffB + coeffC * coeffC);
        float scaled_distance = distanceThreshold * distance_normalizer;

        for (int pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++) {
            // Measure distance between every point and fitted line
            // If distance is smaller than threshold count it as inlier
            PointT point = cloud->points[pointIndex];
            float distance = abs(coeffA * point.x + coeffB * point.y + coeffC * point.z + coeffD);
            if (distance <= scaled_distance) {
                currentInliersResult.insert(pointIndex);
            }
        }

        if (currentInliersResult.size() > inliersResult.size()) {
            inliersResult = std::move(currentInliersResult);
        }
    }

    for (const auto &index: inliersResult) {
        inliers->indices.push_back(index);
    }

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset\n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " microseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx: cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
void proximity(int point_id, std::vector<int>& cluster, const typename pcl::PointCloud<PointT>::VectorType& points, KdTree<PointT>* tree, float distanceTol, std::set<int>& visited_indices) {
    visited_indices.insert(point_id);
    cluster.push_back(point_id);
    const auto& nearby_points = tree->search(points[point_id], distanceTol);
    for (const auto &nearby_point_id: nearby_points) {
        if (visited_indices.count(nearby_point_id) == 0) {
            proximity(nearby_point_id, cluster, points, tree, distanceTol, visited_indices);
        }
    }
}

// Assumes points are already inserted into the tree, and that indices in `points` matches the "ids" in the tree.
template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(const typename pcl::PointCloud<PointT>::VectorType& points, KdTree<PointT>* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    // Instead of tracking points (x,y) as visited, we treat their index in the vector as the uniqueness criteria.
    // Lets us avoid float tolerance equality issues, and probably faster too.
    std::set<int> visited_indices;

    for (int i = 0; i < points.size(); ++i) {
        if (visited_indices.count(i) == 0) {
            // This is a new cluster.
            std::vector<int> cluster;
            proximity(i, cluster, points, tree, distanceTol, visited_indices);
            clusters.emplace_back(std::move(cluster));
        }
    }

    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    auto tree = std::make_unique<KdTree<PointT>>();
    int pointId = 0;
    for (const auto &point: cloud->points) {
        tree->insert(point, pointId);
        pointId++;
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud->points, tree.get(), clusterTolerance);

    for (const auto &cluster : cluster_indices) {
        if (cluster.size() < minSize || cluster.size() > maxSize) {
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx: cluster) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}