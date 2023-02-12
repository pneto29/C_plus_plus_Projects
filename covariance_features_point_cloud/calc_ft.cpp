
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <bits/stdc++.h>



/* Defines */
#define PI 3.14159265

/* Namespace and typedefs */
using namespace std;
using std::vector;
using namespace std::chrono;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef enum{X, Y, Z} axis;

pcl::PointXYZ variance(PointCloud cloud, Eigen::Vector4f centroid);

vector<PointCloud::Ptr> cloudPartitionate(PointCloud::Ptr cloud, int num_part);

/* Main function */
int main (int argc, char** argv)
{
    PointCloud::Ptr target (new PointCloud);

    // Load target cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target) == -1) {
        PCL_ERROR ("Couldn't read target cloud\n");
        return (-1);
    }

    //legend
    // eig(0,0) = lamb_3
    // eig(1,0) = lamb_2
    // eig(2,0) = lamb_1

    Eigen::Vector3f v3(0,0,1);

    Eigen::Vector4f pcaCentroid_trans;
    pcl::compute3DCentroid(*target, pcaCentroid_trans);
    Eigen::Matrix3f covariance_trans;
    pcl::computeCovarianceMatrixNormalized(*target, pcaCentroid_trans, covariance_trans);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_trans, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

    std::cout << "Eigenvalue va(3x1):\n" << eigenValuesPCA << std::endl;
    std::cout << "Cov_Matrix :\n " << covariance_trans << endl;


    double sum_eigenvalues, omni, anistropy, planarity, linearity, scattering, change_curvature, eigenentropy,  verticality;

    //#1
    anistropy = (eigenValuesPCA(2,0) - eigenValuesPCA(0,0))/ eigenValuesPCA(2,0);
    std::cout << "anistropy : " << anistropy << endl;


    //#2
    eigenentropy = -(eigenValuesPCA(0,0)*log(eigenValuesPCA(0,0)) + eigenValuesPCA(1,0)*log(eigenValuesPCA(1,0)) + eigenValuesPCA(2,0)*log(eigenValuesPCA(2,0)));
    std::cout << " eigenentropy : " <<  eigenentropy << endl;


    //#3
    omni = pow(eigenValuesPCA.prod(),1/3);
    std::cout << " omnivariance : " << omni << endl;


    //#4
    sum_eigenvalues = eigenValuesPCA.sum();
    std::cout << "sum_eigenvalues : " << sum_eigenvalues << endl;


    //#5
    planarity = eigenValuesPCA(1,0) - eigenValuesPCA(0,0)/eigenValuesPCA(2,0);
    std::cout << "Planarity : " << planarity << endl;


    //#6
    linearity = eigenValuesPCA(2,0) - eigenValuesPCA(1,0)/eigenValuesPCA(2,0);
    std::cout << "Linearity : " << linearity << endl;


    //#7
    scattering = eigenValuesPCA(0,0)/ eigenValuesPCA(2,0);
    std::cout << "scattering : " << scattering << endl;


    //#8
    change_curvature = eigenValuesPCA(0,0)/ (eigenValuesPCA(0,0) + eigenValuesPCA(1,0) + eigenValuesPCA(2,0));
    std::cout << "change_curvature : " << change_curvature << endl;



}

