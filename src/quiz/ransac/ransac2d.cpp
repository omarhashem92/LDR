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

//plane coefficents are a, b, c, d
//point to compare the distance with the line is defined by: (x, y, z)
float getPointDistance(float a, float b, float c, float d, float x, float y, float z)
{
	//apply the equation: d = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
	float dist = fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
	return dist;
}

std::vector<float> findLineCoefficents(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3)
{
	//plane equation Ax + By + Cz + D = 0
	float a = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
	float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
	float c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
	float d = -(a * x1 + b * y1 + c * z1);
	std::vector<float> lineCoefficents = {a, b, c, d};
	return lineCoefficents;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
 float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	float point1Idx, point2Idx, point3Idx, maxCountPointsOnLine = 0;
	float bestPoint1Idx = 0, bestPoint2Idx = 0, bestPoint3Idx = 0;
	for(int j = 0; j < maxIterations; ++j)
	{
		// Randomly sample subset and fit line
		point1Idx = rand() % cloud->points.size();
		point3Idx = point2Idx = point1Idx;
		while(point1Idx == point2Idx)//make sure that the two indices are not equal
			point2Idx = rand() % cloud->points.size(); 
		
		while(point3Idx == point2Idx || point3Idx == point1Idx)//make sure that the two indices are not equal
			point3Idx = rand() % cloud->points.size(); 

		pcl::PointXYZ point1 = cloud->points[point1Idx];
		pcl::PointXYZ point2 = cloud->points[point2Idx];
		pcl::PointXYZ point3 = cloud->points[point3Idx];
		std::vector<float> lineCoeffs = findLineCoefficents(point1.x, point1.y, point1.z,
		 point2.x, point2.y, point2.z , point3.x, point3.y, point3.z);
		float a = lineCoeffs[0];
		float b = lineCoeffs[1];
		float c = lineCoeffs[2];
		float d = lineCoeffs[3];

		float countPointsOnLine = 0;
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(int i = 0; i < cloud->points.size(); ++i)
		{
			pcl::PointXYZ aPoint = cloud->points[i];
			float x = aPoint.x, y = aPoint.y, z = aPoint.z;
			float dist = getPointDistance(a, b, c, d, x, y, z);
			if(dist < distanceTol)
			{
				countPointsOnLine++;
			}
		}
		if(countPointsOnLine > maxCountPointsOnLine)
		{
			maxCountPointsOnLine = countPointsOnLine;
			bestPoint1Idx = point1Idx;
			bestPoint2Idx = point2Idx;
			bestPoint3Idx = point3Idx;
		}
	}
	
	// Return indicies of inliers from fitted line with most inliers
	pcl::PointXYZ bestPoint1 = cloud->points[bestPoint1Idx];
	pcl::PointXYZ bestPoint2 = cloud->points[bestPoint2Idx];
	pcl::PointXYZ bestPoint3 = cloud->points[bestPoint3Idx];
	std::vector<float> bestLineCoeffs = findLineCoefficents(bestPoint1.x, bestPoint1.y, bestPoint1.z,
	  bestPoint2.x, bestPoint2.y, bestPoint2.z, bestPoint3.x, bestPoint3.y, bestPoint3.z);
	float bestA = bestLineCoeffs[0];
	float bestB = bestLineCoeffs[1];
	float bestC = bestLineCoeffs[2];
	float bestD = bestLineCoeffs[3];
	for(int i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ aPoint = cloud->points[i];
		float x = aPoint.x, y = aPoint.y, z = aPoint.z;
		float dist = getPointDistance(bestA, bestB, bestC, bestD, x, y, z);
		if(dist < distanceTol)
		{
			inliersResult.insert(i);
		}
	}
	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.2);

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
  	  viewer->spinOnce ();
  	}
  	
}
