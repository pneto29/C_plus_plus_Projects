/**
 * validationlib.h
 * Polycarpo Souza Neto
 * Grouping common functions in reference methods in a single .h
 */

#include <limits>
#include <fstream>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>


typedef pcl::PointCloud<PointT> PointCloud;



bool compareX(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return (p1.x < p2.x) ? true : false;
}

bool compareY(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return (p1.y < p2.y) ? true : false;
}

bool compareZ(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return (p1.z < p2.z) ? true : false;
}
