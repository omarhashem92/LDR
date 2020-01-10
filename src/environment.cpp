/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "myProcessPointCloud.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "processPointClouds.cpp"
#include "myProcessPointCloud.cpp"

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
 const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
    typedef pcl::PointXYZI pointT ;

    float filterRes = 0.2f;//The voxel box resultion in meters
    float minX = -7.0f, maxX = 30.0f;//Controls the longitudinal movement of the car
    float minY = -5.0f, maxY = 7.0f;//Controls the lateral movement of the car. Maximize the view on the right of the driver as it is the driver side
    float minZ = -2.0f, maxZ = 0.5f;//controls the height of the lidar
    Eigen::Vector4f minPoint(minX,minY,minZ,1);
    Eigen::Vector4f maxPoint(maxX,maxY,maxZ,1);
    auto filtered_cloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    //renderPointCloud(viewer,filtered_cloud,"filteredCloud");

    int maxIterations = 100;
    float distThresh = 0.2;
    std::pair<pcl::PointCloud<pointT>::Ptr, pcl::PointCloud<pointT>::Ptr> segmentCloud =
      pointProcessorI->SegmentPlane(filtered_cloud, maxIterations, distThresh);
    auto obstCloud = segmentCloud.first;
    auto planeCloud = segmentCloud.second;
    //renderPointCloud(viewer,obstCloud,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,planeCloud,"planeCloud",Color(0,1,0));

    //show clustering output
    float clusterTolerance = 0.3;
    int minSize = 20, maxSize = 500;
    std::vector<pcl::PointCloud<pointT>::Ptr> cloudClusters = pointProcessorI->Clustering(obstCloud, clusterTolerance,
     minSize, maxSize);
    auto red = Color(1,0,0);
    auto yellow = Color(1,1,0);
    auto blue = Color(0,0,1);
    std::vector<Color> colors = {red, yellow, blue};//Colors for clusters
    int clusterId = 0;
    for(pcl::PointCloud<pointT>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointProcessorI->numPoints(cluster);//function that prints number of points in the cluster
        std::string obs_cloud_str = "obstCloud" + std::to_string(clusterId);
        renderPointCloud(viewer,cluster, obs_cloud_str, colors[clusterId % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderPointCloud(viewer, cloud, "PCD_ahmed");
    
    ProcessPointClouds<pcl::PointXYZ> *ProcessPointCloudsObjectPtr = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ProcessPointCloudsObjectPtr->SegmentPlane(cloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    //show clustering output
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ProcessPointCloudsObjectPtr->Clustering(
        segmentCloud.first, 1.0, 3, 30);
    auto red = Color(1,0,0);
    auto green = Color(0,1,0);
    auto blue = Color(0,0,1);
    std::vector<Color> colors = {red, green, blue};//Colors for clusters
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //ProcessPointCloudsObjectPtr->numPoints(cluster);//function that prints number of points in the cluster
        std::string obs_cloud_str = "obstCloud"+std::to_string(clusterId);
        renderPointCloud(viewer,cluster, obs_cloud_str, colors[clusterId]);
        Box box = ProcessPointCloudsObjectPtr->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new lidar::myProcessPointCloud::myProcessPointCloud<pcl::PointXYZI>();
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
        {
            //break;
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }
}
