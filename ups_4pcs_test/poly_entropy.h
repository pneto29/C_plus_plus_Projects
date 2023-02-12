#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>


double computeEntropy( pcl::PointCloud< PointT >::Ptr cloud ){

	Eigen::Vector4f centroid;
	Eigen::Matrix3f covarianceMatrixNormalized = Eigen::Matrix3f::Identity();;

	// estimate the XYZ centroid and the normalized covariance matrix
	pcl::compute3DCentroid (*cloud, centroid);
	pcl::computeCovarianceMatrixNormalized (*cloud, centroid, covarianceMatrixNormalized);

	// compute the determinant and return the entropy
   
    //Only the following conversions can be done with static_cast, 
    //except when such conversions would cast away constness or volatility. 
	double determinant = static_cast<double>((( 2 * M_PI * M_E) * covarianceMatrixNormalized).determinant());

    //entender o pq o registro bom está associado ao determinante dessa amtriz, ver Razlaw(15)*
	return 0.5f*log(determinant);
}
