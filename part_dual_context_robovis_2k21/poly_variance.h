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
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#define PI 3.14159265


pcl::PointXYZ variance(PointCloud cloud, Eigen::Vector4f centroid)
{
    pcl::PointXYZ variance;

    variance.x = 0.0;
    variance.y = 0.0;
    variance.z = 0.0;
    for (int i = 0; i < cloud.width; i++) {
        variance.x += (cloud.points[i].x - centroid[0]) * (cloud.points[i].x - centroid[0]);
        variance.y += (cloud.points[i].y - centroid[1]) * (cloud.points[i].y - centroid[1]);
        variance.z += (cloud.points[i].z - centroid[2]) * (cloud.points[i].z - centroid[2]);
    }

    variance.x /= cloud.width - 1;
    variance.y /= cloud.width - 1;
    variance.z /= cloud.width - 1;

    return variance;
}
