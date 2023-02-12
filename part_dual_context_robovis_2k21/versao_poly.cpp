
/* Includes */
#define PI 3.14159265
#include <algorithm>
#include <string>		// Necessário para usar strings
#include <vector>
#include <math.h>
#include <iostream>
#include <ctime>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/visualization/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <bits/stdc++.h>

#include "poly_rmse.h"
//#include "poly_variance.h"
#include "poly_compare.h"
#include "poly_sort.h"
//#include "poly_entropy.h"
#include "container.h"
#include "sort.h"
#include "lessthan.h"


#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>




/* Defines */
//#define PI 3.14159265

/* Namespace and typedefs */
using namespace std;
using std::vector;
using namespace std::chrono;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef enum{X, Y, Z} axis;

pcl::PointXYZ variance(PointCloud cloud, Eigen::Vector4f centroid);

vector<PointCloud::Ptr> cloudPartitionate(PointCloud::Ptr cloud, int num_part);
std::ostream& bold_on(std::ostream& os)
{
    return os << "\e[1m";
}

std::ostream& bold_off(std::ostream& os)
{
    return os << "\e[0m";
}


/* Main function */
int main (int argc, char** argv)
{
    clock_t begin;
    begin = clock();

    PointCloud::Ptr source (new PointCloud);
    PointCloud::Ptr target (new PointCloud);
    PointCloud::Ptr target_u (new PointCloud);
  //  PointCloud::Ptr cloud_c (new PointCloud);
  //  PointCloud::Ptr cloud_d (new PointCloud);



    // Load target cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target) == -1) {
        PCL_ERROR ("Couldn't read target cloud\n");
        return (-1);
    }

    // Load source cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *source) == -1) {
        PCL_ERROR ("Couldn't read source cloud\n");
        return (-1);
    }
 //   int lf = atof(argv[7]);
    if (source->width >= 500000){
        // Filtering input scan to roughly 10% of original size to increase speed of registration.

        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud (source);
        approximate_voxel_filter.filter (*source);

        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter2;
        approximate_voxel_filter2.setLeafSize (0.2, 0.2, 0.2);
        approximate_voxel_filter2.setInputCloud (target);
        approximate_voxel_filter2.filter (*target);

    }
    else{

    }


    // Load theta
    float theta = atof(argv[3]);
    // float theta = atof(argv[4]);
    // float theta = atof(argv[5]);

    // Load partitions size
    //int maxCloudSize = source->width > target->width ? source->width : target->width;
    int maxCloudSize = target->width;
    // int kmin = round(maxCloudSize / atof(argv[7]));
    int kmax = round(maxCloudSize / atof(argv[4]));

    //  cout << "k_min:  " << kmin << endl;
    cout << "k_max:  " << kmax << endl;

    // Find misaligned target

    Eigen::Affine3f trans_x = Eigen::Affine3f::Identity();
    Eigen::Affine3f trans_y = Eigen::Affine3f::Identity();
    Eigen::Affine3f trans_z = Eigen::Affine3f::Identity();


    trans_x.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
    trans_y.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    trans_z.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud (*target, *target_u, trans_x);
    pcl::transformPointCloud (*target_u, *target_u, trans_y);
    pcl::transformPointCloud (*target_u, *target_u, trans_z);


    // Calculate reference RMSE
    double rmse_ref = computeRMSE(target, target_u, numeric_limits<double>::max ());
    cout << bold_on <<"Stop Criteria (Threshold):  " << bold_off <<  rmse_ref << endl;


    vector<int> axes_s (3);

    // Calculate PCA

    Eigen::Vector4f pcaCentroid_trans;
    pcl::compute3DCentroid(*target, pcaCentroid_trans);
    Eigen::Matrix3f covariance_trans;
    pcl::computeCovarianceMatrixNormalized(*target, pcaCentroid_trans, covariance_trans);
    // eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_trans, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

    std::cout << bold_on << "Eigenvalue va(3x1):\n" << bold_off << eigenValuesPCA << std::endl;
    std::cout << bold_on << "Covariance matrix: \n " << bold_off << covariance_trans << endl;


    //Which eigenvalue is most relevant?

    if (eigenValuesPCA(0,0) >= eigenValuesPCA(1,0) && eigenValuesPCA(0,0) >= eigenValuesPCA(2,0)) {
        axes_s[0] = X;
        if (eigenValuesPCA(1,0) >= eigenValuesPCA(2,0)) {
            axes_s[1] = Y;
            axes_s[2] = Z;
        } else {
            axes_s[1] = Z;
            axes_s[2] = Y;
        }
    } else if (eigenValuesPCA(1,0) >= eigenValuesPCA(2,0)) {
        axes_s[0] = Y;
        if (eigenValuesPCA(0,0) >= eigenValuesPCA(2,0)) {
            axes_s[1] = X;
            axes_s[2] = Z;
        } else {
            axes_s[1] = Z;
            axes_s[2] = X;
        }
    } else {
        axes_s[0] = Z;
        if (eigenValuesPCA(0,0) >= eigenValuesPCA(1,0)) {
            axes_s[1] = X;
            axes_s[2] = Y;
        } else {
            axes_s[1] = Y;
            axes_s[2] = X;
        }
    }

    double  eigentropy = -(eigenValuesPCA(0,0)*log(eigenValuesPCA(0,0)) + eigenValuesPCA(1,0)*log(eigenValuesPCA(1,0))+ eigenValuesPCA(2,0)*log(eigenValuesPCA(2,0)));
    //double  sum_eigenvalues = eigenValuesPCA(0,0) + eigenValuesPCA(1,0) + eigenValuesPCA(2,0);
    double  omnvariance = cbrt(eigenValuesPCA(0,0)*eigenValuesPCA(1,0)*eigenValuesPCA(2,0));
    cout << bold_on <<"Eigentropy: \n" <<  bold_off << eigentropy << endl ;
    cout << bold_on << "Omnvariance: \n" <<  bold_off << omnvariance << endl;
    //cout << bold_on << "Sum_Eigenvalues: \n" <<  bold_off << sum_eigenvalues << endl ;

    if (log(eigentropy) < 0.47 && log(omnvariance < 0.47)) {
        cout << bold_on << "Alignment Core: ICP (Besl and McKay 1992) \n" <<endl ;
    } else{
        cout << bold_on  <<"Alignment Core: Generalized ICP (Segal 2009) \n" <<endl ;

    }

    //    eigentropy = log(eigentropy);
    double rmse_min = -1.0;
    Eigen::Matrix4f rt_min;

    PointCloud::Ptr aux_cloud (new PointCloud);

    bool stop = false;

    int final_axis;
    for (int k = 0.5*kmax; k < kmax; k++) {

        quickSort(source->points.begin(), source->points.end(), compareX);
        vector<PointCloud::Ptr> partitionsx_s = cloudPartitionate(source, k);

        quickSort(target->points.begin(), target->points.end(), compareX);
        vector<PointCloud::Ptr> partitionsx_t = cloudPartitionate(target, k);


        // quickSortand partitionate in Y
        quickSort(source->points.begin(), source->points.end(), compareY);
        vector<PointCloud::Ptr> partitionsy_s = cloudPartitionate(source, k);

        quickSort(target->points.begin(), target->points.end(), compareY);
        vector<PointCloud::Ptr> partitionsy_t = cloudPartitionate(target, k);

        // quickSortand partitionate in Z
        quickSort(source->points.begin(), source->points.end(), compareZ);
        vector<PointCloud::Ptr> partitionsz_s = cloudPartitionate(source, k);

        quickSort(target->points.begin(), target->points.end(), compareZ);
        vector<PointCloud::Ptr> partitionsz_t = cloudPartitionate(target, k);

        // Loop for each partition
        for(int i=k-1; i>0;i=i-1){
            //for (int i = 0; i < k; i++) {

            /*      std::string name_out =  "./luis_" +  std::to_string(i)  + ".pcd";
            pcl::io::savePCDFileASCII (name_out , *partitionsy_s[i]);
            pcl::io::savePCDFileASCII ("slice_conv.pcd" , *partitionsy_s[i]);
*/

            for (int p = 0; p < 3; p++) {
                //double localEntropy = 0;


                if (log(eigentropy) < 0.47 && log(omnvariance) < 0.47) {


                    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                    switch (axes_s[p]) {
                    case X:

                        icp.setInputSource(partitionsx_s[i]);
                        icp.setInputTarget(partitionsx_t[i]);

                        break;
                    case Y:

                        icp.setInputSource(partitionsy_s[i]);
                        icp.setInputTarget(partitionsy_t[i]);

                        break;
                    case Z:

                        icp.setInputSource(partitionsz_s[i]);
                        icp.setInputTarget(partitionsz_t[i]);

                        break;

                    }
                    icp.setMaximumIterations(atoi(argv[5]));

                    icp.align(*aux_cloud);

                    // Transform source cloud
                    Eigen::Matrix4f rt = icp.getFinalTransformation();
                    pcl::transformPointCloud(*source, *aux_cloud, rt);
                 //   *cloud_c = (*aux_cloud) + (*target);
                 //   *cloud_d = (*target_u) + (*target);

                    // Check RMSE
                    double rmse = computeRMSE(target, aux_cloud, numeric_limits<double>::max ());
                    if (rmse < rmse_min || rmse_min == -1.0 ) {


                        final_axis = axes_s[p];
                        rmse_min = rmse;
                        //  localEntropy = computeEntropy(cloud_c);
                        // cout << "Entropy: \n" << localEntropy << endl;

                        rt_min = rt;
                    }

                    if (rmse_min <= rmse_ref) {
                        stop = true;
                        break;
                    }
                }
                else {
                    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

                    switch (axes_s[p]) {
                    case X:

                        icp.setInputSource(partitionsx_s[i]);
                        icp.setInputTarget(partitionsx_t[i]);


                        break;
                    case Y:

                        icp.setInputSource(partitionsy_s[i]);
                        icp.setInputTarget(partitionsy_t[i]);

                        break;
                    case Z:

                        icp.setInputSource(partitionsz_s[i]);
                        icp.setInputTarget(partitionsz_t[i]);

                        break;

                    }


                    icp.setMaximumIterations(atoi(argv[5]));

                    icp.align(*aux_cloud);

                    // Transform source cloud
                    Eigen::Matrix4f rt = icp.getFinalTransformation();
                    pcl::transformPointCloud(*source, *aux_cloud, rt);
                   // *cloud_c = (*aux_cloud) + (*target);
                   // *cloud_d = (*target_u) + (*target);

                    // Check RMSE
                    double rmse = computeRMSE(target, aux_cloud, numeric_limits<double>::max ());
                    if (rmse < rmse_min || rmse_min == -1.0) {
                        final_axis = axes_s[p];
                        //  localEntropy = computeEntropy(cloud_c);
                        // cout << "Entropy: \n" << localEntropy << endl;

                        rmse_min = rmse;
                        rt_min = rt;
                    }

                    if (rmse_min <= rmse_ref) {
                        stop = true;
                        break;
                    }

                }  }
            if (stop) {
                break;
            }
        }
        if (stop) {
            break;
        }
    }


    if (!stop) {
        pcl::transformPointCloud(*source, *aux_cloud, rt_min);
    }

    double total_time = (clock() - begin) / (double)CLOCKS_PER_SEC;

    Eigen::Matrix3f r (rt_min.block<3,3>(0, 0));
    Eigen::AngleAxisf angle_axis;
    angle_axis.fromRotationMatrix(r);

    ofstream output;
    output.open(argv[6]);
    output << "time: " << total_time << endl;
    output << "max partitions: " << kmax << endl;
    output << "final source" << final_axis <<endl;
    output << "rmse: " << rmse_min << endl;
    output << "rmse reference: " << rmse_ref << endl;

    output << "rt: \n" << rt_min << endl;
    output << "angle: " << (angle_axis.angle()*180.0)/M_PI << endl;
    output << "axis: \n" << angle_axis.axis() << endl;

    output.close();

    double elem1, elem2, elem3, angle123,axis_angle;
    elem1 = rt_min(0,0);

    elem2 = rt_min(1,1);
    elem3= rt_min(2,2);
    angle123= (elem1+elem2+elem3-1)/2;
    axis_angle = acos (angle123) * 180.0 / PI;

    cout << bold_on << "Rot. in degree: " << bold_off << axis_angle << endl;
    cout << bold_on << "time in seconds: " << bold_off << total_time << endl;

    // Visualization
    int sizePoints = 2;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer (" (L) INITIAL POSE //  (R) BEFORE "));

    // Adicionando ambas as nuvens de pontos
    int v0,v1;
    viewer1->setBackgroundColor(255,255,255);
    viewer1->createViewPort(0,0,0.5,1,v0);
    viewer1->createViewPort(0.5,0,1,1,v1);
    viewer1->addPointCloud(target, "target");
    viewer1->addPointCloud(source, "source",v0);
    viewer1->addPointCloud(aux_cloud, "registered",v1);
    //viewer1->addCoordinateSystem(10,"ref");



    //Display cloud_out
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"source");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "source");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 1, 0,"target");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "target");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 0, 1,"registered");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "registered");


    ////////////////////////////////////////////////////////////////////////////////////////////////

    while(!viewer1->wasStopped())
    {
        viewer1->spin();
    }

    return 0;
}


vector<PointCloud::Ptr> cloudPartitionate(PointCloud::Ptr cloud, int num_part)
{
    // Number of points per partition
    int part_size = floor(cloud->width/num_part);

    vector<PointCloud::Ptr> partitions;
    for (int i = 0; i < num_part; i++) {
        PointCloud::Ptr tmp (new PointCloud);
        tmp->width  = part_size;
        tmp->height = 1;

        for (int j = i*part_size; j < (i+1) * part_size; j++) {
            tmp->points.push_back(cloud->points[j]);
        }

        partitions.push_back(tmp);
    }

    return partitions;
}

