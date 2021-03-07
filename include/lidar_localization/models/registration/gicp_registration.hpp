/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_GICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_GICP_REGISTRATION_HPP_

#include <pcl/registration/gicp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class GICPRegistration: public RegistrationInterface {
  public:
    GICPRegistration(const YAML::Node& node);
    GICPRegistration(
      float maximum_correspondence_distance, 
      int maximum_iterations, 
      int maximum_optimizer_iterations, 
      int ransac_iterations,
      float ransac_outlier_rejection_threshold,
      float transformation_epsilon,
      bool use_reciprocal_correspondence
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(
      float maximum_correspondence_distance, 
      int maximum_iterations, 
      int maximum_optimizer_iterations, 
      int ransac_iterations,
      float ransac_outlier_rejection_threshold,
      float transformation_epsilon,
      bool use_reciprocal_correspondence
    );

  private:
    pcl::GeneralizedIterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr gicp_ptr_;
};
}

#endif
