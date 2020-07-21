/*------------------------------------------------------------------*\
   Useful point cloud utilites
\*------------------------------------------------------------------*/

#include "pointCloudUtils.h"


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr pointCloud (new pcl::PointCloud<PointT>);

    // Read from File
    if (pcl::io::loadPCDFile<PointT> (file, *pointCloud) == -1) 
    {
        PCL_ERROR ("***ERROR: File read failed\n");
    }
    std::cerr << "Loaded " << pointCloud->points.size () << " points from " << file << std::endl;

    return pointCloud;
}


template<typename PointT>
void printPointCloudStats(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string name) 
{
    float minX = pointCloud->points[0].x, maxX = pointCloud->points[0].x;
    float minY = pointCloud->points[0].y, maxY = pointCloud->points[0].y;
    float minZ = pointCloud->points[0].z, maxZ = pointCloud->points[0].z;
    
    for (auto it = pointCloud->points.begin(); it != pointCloud->points.end(); ++it ) {
        if ( (*it).x < minX)  minX = (*it).x;
        if ( (*it).x > maxX)  maxX = (*it).x;
        if ( (*it).y < minY)  minY = (*it).y;
        if ( (*it).y > maxY)  maxY = (*it).y;
        if ( (*it).z < minZ)  minZ = (*it).z;
        if ( (*it).z > maxZ)  maxZ = (*it).z;
    }

    std::cout << name << " Input Cloud Size: " << pointCloud->size() << " points" << std::endl;
    std::cout << name << " Input Cloud Min ( " << minX << ", " << minY << ", " << minZ << ")" << std::endl;
    std::cout << name << " Input Cloud Max ( " << maxX << ", " << maxY << ", " << maxZ << ")" << std::endl;
    Eigen::Vector4f origin = pointCloud->sensor_origin_;
    std::cout << name << " Sensor Origin: ("  << origin(0) << "," << origin(1) << "," << origin(2) << ")" << std::endl;
}



template<typename PointT>
void printPointCloudStatsEx(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string name) 
{
    pcl::PointXYZ minPoint, maxPoint;

    pcl::getMinMax3D(*pointCloud, minPoint, maxPoint);
 
    std::cout << name << " Input Cloud Size: " << pointCloud->size() << " points" << std::endl;
    std::cout << name << " Input Cloud Min ( " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << ")" << std::endl;
    std::cout << name << " Input Cloud Max ( " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << ")" << std::endl;
    Eigen::Vector4f origin = pointCloud->sensor_origin_;
    std::cout << name << " Sensor Origin: ("  << origin(0) << "," << origin(1) << "," << origin(2) << ")" << std::endl;
}





 