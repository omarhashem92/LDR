// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

/*
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}
*/

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
 float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    static int countNumTimesOfExecution = 0;
    static unsigned int accumelativeTimeTaken = 0;
    countNumTimesOfExecution++;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*cloud_filtered);

    // Region based filtering
    typename pcl::PointCloud<PointT>::Ptr regioned_cloud (new pcl::PointCloud<PointT>());
    pcl::CropBox< PointT > regionOfInterest;
    regionOfInterest.setInputCloud(cloud_filtered);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.filter(*regioned_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    accumelativeTimeTaken += (float)elapsedTime.count();
    std::cout << "Average filtering took " << accumelativeTimeTaken / 1.0 * countNumTimesOfExecution << " milliseconds" << std::endl;
    return regioned_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane+
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>), planeCloud (new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    //cloud.swap (obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); // holds the indicies of the points that relate to the road plane
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);//The main algorithm to classify the points that relate to the road and others that relate to the obstacles
    seg.setMaxIterations (maxIterations);//The alogirthm is based on drawing a random line and tries to find the minimum distance to other points so each time we draw a line is an iteration
    seg.setDistanceThreshold (distanceThreshold);//thickness of the road plane so that the points that not relate to the road plane are considered obstacles

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // ENDTODO:: Fill in this function to find inliers for the cloud.
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    static unsigned long long countNumTimesOfExecution = 0;
    static unsigned long long accumelativeTimeTaken = 0;
    countNumTimesOfExecution++;
    accumelativeTimeTaken += elapsedTime.count();
    std::cout << "Average plane segmentation took " << accumelativeTimeTaken / 1.0 * countNumTimesOfExecution << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;//the return of the function that will contain (n) elements, where (n) is the number of clusters. Each element contains the cloud-points which are related to the cluster

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);//create a Kdtree object to find the nearest neighbours in log(n)
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;//This vector will contain (n) elements, where (n) is the number of clusters. Each element contains the cloud-indicies which are related to that cluster
    pcl::EuclideanClusterExtraction<PointT> ec;//use euclidean distance method for finding distance between points
    ec.setClusterTolerance (clusterTolerance);//The distance between points to be considered in the same cluster
    ec.setMinClusterSize (minSize);//minimum cluster size to avoid taking noise as clusters
    ec.setMaxClusterSize (maxSize);//maximum cluster size to avoid taking 2 clusters as 1 big cluster
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);//fill the cluster indices vector
    
    //Create a loop that will fill the clusters vector with the actual cloud points
    for(pcl::PointIndices a_cluster : cluster_indices)
    {//find a cluster from the clusters vector
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new  pcl::PointCloud<PointT>);
        for(int a_point_idx : a_cluster.indices)
        {
            PointT point = cloud->points[a_point_idx];
            cloud_cluster->points.push_back(point);
        }
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