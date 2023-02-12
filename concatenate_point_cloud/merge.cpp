#include <pcl/io/pcd_io.h>

int
main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZ>);

    // Read two PCD files from disk.
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudA);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloudB);

    // Create cloud "C", with the points of both "A" and "B".
    *cloudC = (*cloudA) + (*cloudB);
    pcl::io::savePCDFile<pcl::PointXYZ>(argv[3], *cloudC); 
}

