#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigen>
int
main (int argc, char** argv)
{
  // 填入点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/chongda517/catkin_velodyne/src/localization_in_auto_driving/lidar_localization/slam_data/key_frames/key_frame_0.pcd", *input_cloud) == -1)
  {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
  }
  std::cout << "Loaded " << input_cloud->size() << " data points from room_scan1.pcd" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/chongda517/catkin_velodyne/src/localization_in_auto_driving/lidar_localization/slam_data/key_frames/key_frame_1.pcd", *target_cloud) == -1)
  {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
   }
  std::cout << "Loaded " << target_cloud->size() << " data points from room_scan2.pcd" << std::endl;
  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
	float err = 1.0;
	Eigen::Matrix3f R_12;
	R_12 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Vector3f T_12;
	T_12[0] = 0;
	T_12[1] = 0;
	T_12[2] = 0;
	Eigen::Matrix4f H_12;
	H_12 << R_12(0, 0), R_12(0, 1), R_12(0, 2), T_12[0],
		R_12(1, 0), R_12(1, 1), R_12(1, 2), T_12[1],
		R_12(2, 0), R_12(2, 1), R_12(2, 2), T_12[2],
		0, 0, 0, 1;

	Eigen::Matrix3f R_final = R_12;
	Eigen::Vector3f T_final = T_12;
	Eigen::Matrix4f H_final = H_12;
  
  int iterator_num = 0;
  int max_correspond_distance = 1.36;
  while(iterator_num < 30 )
  {
	iterator_num++;
	std::vector<float> errs;
	std::vector<pcl::PointXYZ> outPointsTarget;
	std::vector<pcl::PointXYZ> outPointsSource;
	//errs.clear();
	errs.clear();
	outPointsTarget.clear();
	outPointsSource.clear();
	//计算最临近点
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input_cloud,   *result_cloud_ptr,  H_12);//转换点云
        //k近邻查找
        pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree_ptr;
	kdtree_ptr.setInputCloud(target_cloud);
	int knn = 1;
        for(size_t i = 0;i < result_cloud_ptr->size();++i)
	{
		auto transformed_point = result_cloud_ptr->at(i);
                std::vector<float> distances;
            	std::vector<int>indexs; 
		kdtree_ptr.nearestKSearch(transformed_point,knn,indexs,distances);      // knn搜索
		if(distances[0] > max_correspond_distance)
			continue;
		errs.push_back(distances[0]);
		outPointsTarget.push_back(target_cloud->at(indexs[0]));
		outPointsSource.push_back(result_cloud_ptr->at(i));
	}
	//计算点云中心坐标
        size_t N = outPointsSource.size();
	Eigen::Vector3f p1,p2;
	p1 << 0,0,0;
	p2 << 0,0,0;
        
	for(size_t i = 0 ;i < N; i++)
	{
		p1 += Eigen::Vector3f(outPointsSource[i].x,outPointsSource[i].y,outPointsSource[i].z);
		p2 += Eigen::Vector3f(outPointsTarget[i].x,outPointsTarget[i].y,outPointsTarget[i].z);
		err += errs[i];
	}
	p1 /= N;
	p2 /= N;
	err /= N;
        //std::cout << "p1:" << p1 << "p2:" << p2 << "err:" << err << "\n"; 
	//去中心化
	std::vector<Eigen::Vector3f> q1,q2;
	//std::cout << "result_cloud_ptr: " << result_cloud_ptr->size() << " " << "outPoints: " << outPoints.size()  << "N: " << N;
	//std::vector<pcl::PointXYZ> q1,q2;
        
	for(size_t i = 0 ;i < N; i++)
	{
		Eigen::Vector3f m1,m2;
		m1 = Eigen::Vector3f(outPointsSource[i].x,outPointsSource[i].y,outPointsSource[i].z) - p1;
		m2 = Eigen::Vector3f(outPointsTarget[i].x,outPointsTarget[i].y,outPointsTarget[i].z) - p2;
		q1.push_back(m1);
		q2.push_back(m2);
		
	}
	 
	// compute q1*q2^T
	Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
	for (size_t i = 0; i < N; i++)
	{
		W += q1[i] * q2[i].transpose();
	}
		// SVD on W
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();
 		R_12 = U* (V.transpose());
		T_12 = p1 - R_12 * p2;
 
		H_12 << R_12(0, 0), R_12(0, 1), R_12(0, 2), T_12[0],
			R_12(1, 0), R_12(1, 1), R_12(1, 2), T_12[1],
			R_12(2, 0), R_12(2, 1), R_12(2, 2), T_12[2],
			0, 0, 0, 1;
 
		R_final = R_12*R_final;  //更新旋转矩阵
		T_final = R_12*T_final + T_12; //更新平移矩阵
		cout << "R_final: " << R_final << " \n"<< endl; 
		
		if(err < 0.114)
		{
			cout << "err:" << err << endl;
			break;
		}
  }
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(input_cloud);
  icp.setInputTarget(target_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;*/

  //使用创建的变换对未过滤的输入点云进行变换

 
  return (0);
}
