/*------------------------------------------------------------------*\
   Some useful point cloud utilites
\*------------------------------------------------------------------*/
#include <iostream> 

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry> 


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

template<typename PointT>
void printPointCloudStats(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string name);

template<typename PointT>
void printPointCloudStatsEx(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string name);



