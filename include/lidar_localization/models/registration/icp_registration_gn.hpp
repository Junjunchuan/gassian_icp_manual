/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_GN_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_GN_HPP_

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "sophus/se3.hpp"
namespace lidar_localization {
class ICPRegistrationGN: public RegistrationInterface {
  public:
    ICPRegistrationGN(const YAML::Node& node);
    ICPRegistrationGN(
      float max_corr_dist,  
      int max_iter
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(
      float max_corr_dist, 
      int max_iter
    );
    void Icp_Guass_Newton(const CloudData::CLOUD_PTR   &input_cloud );       // 计算旋转矩阵 
  private:
      CloudData::CLOUD_PTR target_cloud_;        
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;
      float max_correspond_distance_;     // 阈值
      int max_iterator_;                                     //最大迭代次数
      float acc_err_;
      
      Eigen::Matrix3f  rotation_matrix_;       //旋转矩阵
      Eigen::Vector3f  translation_vector_;     //平移矩阵
      Eigen::Matrix4f  transformation_pose_;       // 转换矩阵 
};
}

#endif
