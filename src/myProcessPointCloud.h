//////////////////////////////////
// Author:   Ahmed Bakr
// version:  0.2
//////////////////////////////////
#ifndef MY_PROCESSOR_POINT_CLOUD_H_
#define MY_PROCESSOR_POINT_CLOUD_H_


#include <unordered_set>
#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"


namespace lidar
{
    namespace myProcessPointCloud
        {
        template <typename PointT>
        class myProcessPointCloud   : public ProcessPointClouds<PointT>
            {
            private:
                KdTree *tree;

                static std::vector<float> getLineCoefficents(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3);
                static float getPointDistance(float a, float b, float c, float d, float x, float y, float z);

            public:
                myProcessPointCloud()
                {
                    tree = new KdTree();
                }
                static std::unordered_set<int> ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
                void insertPointsInKdTree(typename pcl::PointCloud<PointT>::Ptr cloud);
                std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, float distanceTol, int minSize, int maxSize);
                void proximity(int idx, std::vector<int> &cluster, std::vector<bool> &isProcessed, const std::vector<std::vector<float>> &points, float distanceTol);
               
                std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
                    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) override;

                std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                            float clusterTolerance, int minSize, int maxSize) override;


                std::vector< pcl::PointCloud<PointT>::Ptr> myClustering( pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

                void myProximity(int idx, std::vector<int> &cluster, std::vector<bool> &isProcessed, const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);
                std::vector<std::vector<int>> myEuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);


            };
    } // namespace myProcessPointCloud
} // namespace lidar

#endif