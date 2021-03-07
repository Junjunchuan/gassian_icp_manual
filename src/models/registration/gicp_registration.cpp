/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */
#include "lidar_localization/models/registration/gicp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

GICPRegistration::GICPRegistration(
    const YAML::Node& node
) : gicp_ptr_(new pcl::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    float maximum_correspondence_distance = node["maximum_correspondence_distance"].as<float>();
    int maximum_iterations = node["maximum_iterations"].as<int>();
    int maximum_optimizer_iterations = node["maximum_optimizer_iterations"].as<int>();
    int ransac_iterations = node["ransac_iterations"].as<int>();
    float ransac_outlier_rejection_threshold = node["ransac_outlier_rejection_threshold"].as<float>();
    float transformation_epsilon = node["transformation_epsilon"].as<float>();
    bool use_reciprocal_correspondence = node["use_reciprocal_correspondence"].as<bool>();

    SetRegistrationParam(maximum_correspondence_distance, maximum_iterations, maximum_optimizer_iterations,ransac_iterations,ransac_outlier_rejection_threshold,transformation_epsilon,use_reciprocal_correspondence);
}
//第二个构造函数
GICPRegistration::GICPRegistration(
      float maximum_correspondence_distance, 
      int maximum_iterations, 
      int maximum_optimizer_iterations, 
      int ransac_iterations,
      float ransac_outlier_rejection_threshold,
      float transformation_epsilon,
      bool use_reciprocal_correspondence
) : gicp_ptr_(new pcl::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(maximum_correspondence_distance, maximum_iterations, maximum_optimizer_iterations,ransac_iterations,ransac_outlier_rejection_threshold,transformation_epsilon,use_reciprocal_correspondence);
}

bool GICPRegistration::SetRegistrationParam(
      float maximum_correspondence_distance, 
      int maximum_iterations, 
      int maximum_optimizer_iterations, 
      int ransac_iterations,
      float ransac_outlier_rejection_threshold,
      float transformation_epsilon,
      bool use_reciprocal_correspondence
) {
    gicp_ptr_->setMaxCorrespondenceDistance(maximum_correspondence_distance);
    gicp_ptr_->setMaximumIterations(maximum_iterations);
    //gicp_ptr_->setMaximumOptimizerIterations(maximum_optimizer_iterations);
    gicp_ptr_->setRANSACIterations(ransac_iterations);
    gicp_ptr_->setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);
    gicp_ptr_->setTransformationEpsilon(transformation_epsilon);
    gicp_ptr_->setEuclideanFitnessEpsilon(0.36);
    gicp_ptr_->setUseReciprocalCorrespondences(use_reciprocal_correspondence);
    LOG(INFO) << "GICP params:" << std::endl
              << "maximum_correspondence_distance: " << maximum_correspondence_distance << ", "
              << "maximum_iterations: " << maximum_iterations << ", "
              << "maximum_optimizer_iterations: " << maximum_optimizer_iterations << ", "
              << "ransac_iterations: " <<ransac_iterations << ","
              << "ransac_outlier_rejection_threshold: " << ransac_outlier_rejection_threshold << ","
              << "transformation_epsilon: " << transformation_epsilon << ","
              << "use_reciprocal_correspondence: " << use_reciprocal_correspondence
              << std::endl;

    return true;
}

bool GICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    gicp_ptr_->setInputTarget(input_target);

    return true;
}

bool GICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    gicp_ptr_->setInputSource(input_source);
    gicp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = gicp_ptr_->getFinalTransformation();
    
    if (gicp_ptr_->hasConverged()) {
      LOG(INFO) << "GICP converged." << std::endl
	        << "The score is " << gicp_ptr_->getFitnessScore();
      } else {
          LOG(INFO) << "GICP did not converge.";
      }

    return true;
}

}
