/*
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */
#include "lidar_localization/models/registration/icp_registration_gn.hpp"

#include "glog/logging.h"
#include <Eigen/Dense>

namespace lidar_localization {

ICPRegistrationGN::ICPRegistrationGN(
    const YAML::Node& node
) : kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>) {
    
    float max_corr_dist = node["max_corr_dist"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist,max_iter);
}

ICPRegistrationGN::ICPRegistrationGN(
    float max_corr_dist,  
    int max_iter
) :  kdtree_ptr_(new pcl :: KdTreeFLANN<CloudData :: POINT> ) {

    SetRegistrationParam(max_corr_dist, max_iter);
}

bool ICPRegistrationGN::SetRegistrationParam(
    float max_corr_dist, 
    int max_iter
) {
    max_correspond_distance_  = max_corr_dist;
    max_iterator_ =  max_iter;


    LOG(INFO) << "ICPGN params:" << std::endl
              << "max_corr_dist: " << max_corr_dist << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool ICPRegistrationGN::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    target_cloud_.reset(new CloudData::CLOUD);
    target_cloud_ = input_target;
    kdtree_ptr_ ->setInputCloud(input_target);

    return true;
}

bool ICPRegistrationGN::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    transformation_pose_  = predict_pose; 
    
    rotation_matrix_ = transformation_pose_.block<3,  3>(0,  0) ;    
    translation_vector_  = transformation_pose_.block<3,  1>(0,  3);  

    Icp_Guass_Newton(input_source);      


    pcl::transformPointCloud(*input_source,   *result_cloud_ptr,  transformation_pose_);   // 对点云进行变换
    result_pose = transformation_pose_;

    return true;
}

void ICPRegistrationGN::Icp_Guass_Newton(const CloudData::CLOUD_PTR   &input_source){
    CloudData::CLOUD_PTR  transformed_cloud(new CloudData::CLOUD);
    int knn = 1;     // 搜索最近点
    int iterator_num = 0;
    acc_err_ = 0.0;
    while(iterator_num < max_iterator_)
    {
        pcl::transformPointCloud(*input_source,*transformed_cloud,transformation_pose_);    // 对点云进行变换
        Eigen::Matrix<float,6,6> Hessian = Eigen::Matrix<float,6,6>::Zero();
        Eigen::Matrix<float,6,1> g = Eigen::Matrix<float,6,1>::Zero();


        for(size_t i =0; i < transformed_cloud->size();  ++i)
        {
            auto origin_point = input_source->at(i);
            if(!pcl::isFinite(origin_point))
                continue;
            auto transformed_point = transformed_cloud->at(i);
            std::vector<float> distances;
            std::vector<int> indexs;     
            kdtree_ptr_->nearestKSearch(transformed_point,knn,indexs,distances);      // knn搜索最临近点
	    //去掉未找到关联点
            if(distances[0] > max_correspond_distance_)
            {
                continue;
            }
            Eigen::Vector3f closet_point  = Eigen::Vector3f(target_cloud_->at(indexs[0]).x,   target_cloud_->at(indexs[0]).y ,
                                                                target_cloud_->at(indexs[0]).z );
            // 计算原始点在目标点云中最临近点的距离
            Eigen::Vector3f err = 
                Eigen::Vector3f(transformed_point.x,transformed_point.y,transformed_point.z) - closet_point;

            Eigen::Matrix<float,3,6> Jacobian = Eigen::Matrix<float,3,6>::Zero();
	    Jacobian.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
	    Jacobian.block<3,3>(0,3) = -rotation_matrix_* Sophus::SO3f::hat(Eigen::Vector3f(origin_point.x,origin_point.y,origin_point.z)) ;
            Hessian  +=  Jacobian.transpose()* Jacobian; 
            g += -Jacobian.transpose()*err;
	    acc_err_ +=  sqrt(pow(err[0], 2) + pow(err[1], 2) + pow(err[2], 2));
        }

        iterator_num++;
        if(Hessian.determinant() == 0)
        {
                continue;
        }
        Eigen::Matrix<float,6,1> delta_x =  Hessian.inverse()*g;

        translation_vector_ += delta_x.head<3>();
        auto  delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
        rotation_matrix_ *= delta_rotation.matrix();

        transformation_pose_.block<3,3>(0,0) = rotation_matrix_;
        transformation_pose_.block<3,1>(0,3) = translation_vector_;
	/*std::cout << "acc_err: " << acc_err_ << std::endl;
	if(acc_err_ < 0.1 )
	{
		std::cout << "acc_err: " << acc_err_ << std::endl;
		break;
	}*/
    }

}
}
