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

double computeRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){
    //double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(target);

    double fitness_score = 0.0;

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < source->points.size (); ++i){
        //Avoid NaN points as they crash nn searches
        if(!pcl_isfinite((*source)[i].x)){
            continue;
        }

        // Find its nearest neighbor in the target
        tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range*max_range){
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0){
        //cout << "nr: " << nr << endl;
        //cout << "fitness_score: " << fitness_score << endl;
        return sqrt(fitness_score / nr);
    }else{
        return (std::numeric_limits<double>::max ());
    }
}


