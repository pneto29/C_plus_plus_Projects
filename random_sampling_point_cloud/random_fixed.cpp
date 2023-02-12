/*********************************************************************************************************************
Main Function for point cloud registration with Generalized ICP
Last modified: August 24, 2020

Reference:
Vitter, Jeffrey Scott. "Faster methods for random sampling." Communications of the ACM 27.7 (1984): 703-718.

Responsible for implementation: ***
Documentation: https://pointclouds.org/documentation/classpcl_1_1_random_sample.html
**********************************************************************************************************************/
#include "validationlib.h"
#include <pcl/filters/random_sample.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char** argv){
    PointCloudT::Ptr cloud_in (new PointCloudT), rnd_sampled_cloud (new PointCloudT);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer ("Viewer"));

    pcl::io::loadPCDFile(argv[1], *cloud_in);
    int points = atoi(argv[2]);
    //float s = cloud->width;

    pcl::RandomSample<PointT> rand_sample;
    rand_sample.setInputCloud(cloud_in); //source cloud
    rand_sample.setSample(points); // number of output points
    rand_sample.filter(*rnd_sampled_cloud); //downsampling and output to rnd_sampled_cloud
    std::vector<int> index; rand_sample.filter(index); //extract the index of the sampling point

    pcl::io::savePCDFileASCII (argv[3],*rnd_sampled_cloud);
    //Visualization
    int sizePoints = 2;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("OUT_RND_SAMPLE"));

    // Adicionando ambas as nuvens de pontos
    viewer2->setBackgroundColor(255,255,255);

    viewer2->addPointCloud(rnd_sampled_cloud, "rnd_sampled_cloud");

    //Configurando rnd_sampled_cloud
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"rnd_sampled_cloud");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "rnd_sampled_cloud");

    while(!viewer2->wasStopped())
    {
        viewer2->spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    system("pause");
}
