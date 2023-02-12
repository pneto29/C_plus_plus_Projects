/*********************************************************************************************************************
Main Function for point cloud registration with KEYPOINT Four Point COngruent Sets
Last modified: July 19, 2021

Reference:
Theiler, Pascal Willy, Jan Dirk Wegner, and Konrad Schindler. "Keypoint-based 4-points congruent sets–automated marker-less registration of laser scans.
" ISPRS journal of photogrammetry and remote sensing 96 (2014): 149-163.
Responsible for implementation:
Documentation: https://pointclouds.org/documentation/classpcl_1_1registration_1_1_f_p_c_s_initial_alignment.html
**********************************************************************************************************************/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>   // 
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/math/special_functions/round.hpp>
#include <pcl/registration/ia_kfpcs.h>

#include "validationlib.h"
using namespace std;
int
main(int argc, char** argv)
{
    pcl::console::TicToc time;
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
    {
        PCL_ERROR("Failed to read target cloud\n");
        return (-1);
    }

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *input_cloud) == -1)
    {
        PCL_ERROR("Failed to read source cloud  \n");
        return (-1);
    }

    time.tic();
    //inicialization
    pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs;
    kfpcs.setInputSource(input_cloud);  //
    kfpcs.setInputTarget(target_cloud);  //
    kfpcs.setApproxOverlap(0.4);// Defina a sobreposição aproximada entre a origem e o destino.
    kfpcs.setLambda(0.5);
    kfpcs.setDelta(0.2);//Incremento de fator constante para ponderar parâmetros de cálculo internos.
    kfpcs.setMaxComputationTime(1000);//Defina o tempo máximo de cálculo (em segundos).
    kfpcs.setNumberOfSamples(1000); //Defina o número de amostras de nuvem de ponto de origem a serem usadas durante o registro
    pcl::PointCloud<pcl::PointXYZ>::Ptr kfpcs_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    kfpcs.align(*kfpcs_cloud);
    cout << "KFPCS time registration： " << time.toc() << " ms" << endl;
    cout << "Rotation matrix：\n" << kfpcs.getFinalTransformation() << endl;


    // Use the created transformation to transform the source point cloud
    pcl::transformPointCloud(*input_cloud, *output_cloud, kfpcs.getFinalTransformation());

    double rms = computeCloudRMS(target_cloud, output_cloud, std::numeric_limits<double>::max ());
    std::cout << "RMS: " << rms << std::endl;

    // Save the converted source point cloud as the final transformed output
    //  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

    //
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Mostrar nuvem de pontos"));
    viewer->setBackgroundColor(0, 0, 0);  //

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(input_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(output_cloud, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }

    return (0);
}
