/*-------------------------------------------------------------------*\

  NAME
    simple_pcd_viewer


  DESCRIPTION
    Load a PCD formatted point cloud file that has been provided 
    as a command line parameter and display it in PCLVisualizer Window.
    
    PCD File format:
    http://pointclouds.org/documentation/tutorials/pcd_file_format.html

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/

#include <iostream>
#include <fstream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


/*---------------------------------------------------------------*\
    Read PCD File into pcl::PointCloud 
\*---------------------------------------------------------------*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string fileName)
{

    typename pcl::PointCloud<PointT>::Ptr pointCloud (new pcl::PointCloud<PointT>);

    // Read from File
    if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1) 
    {
        PCL_ERROR("***ERROR: File read failed!\n");
    }
    std::cout << "Loaded " << pointCloud->points.size () << " points from file '" << fileName << "'" << std::endl;

    return pointCloud;
}



int main(int argc, char **argv)
{

    /*---------------------------------------------------------------*\
       Check command line parameters
    \*---------------------------------------------------------------*/
    if (argc < 2) {
        std::cout << argv[0] << ": Visualize Point Cloud provided in PCD file" << std::endl;
        std::cout << "Usage: " << argv[0] << " PCD_filename" << std::endl;
        return 1;
    }


    std::string fileName = argv[1];

    // Instatiate XYZ Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);

    pointCloud = loadPcd<pcl::PointXYZ>(fileName);

    std::string name = "Point Cloud";

    // Intstatiate Point Cloud Viewer 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Simple Point Cloud Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Select color based off of X value
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(pointCloud, "x"); //Render by x field
	viewer->addPointCloud<pcl::PointXYZ>(pointCloud, fildColor, name); //Show point cloud, where fildColor is color display

  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    // Main Loop
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}

