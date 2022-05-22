/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-14 22:24:26
 * @Description: 
 * @Others: 
 */
#include "utility.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Algorithm/PointClouds/registration/ceres_edgeSurfFeatureRegistration.hpp"
#include "Algorithm/PointClouds/processing/Filter/voxel_grid.hpp"

//pcl显示点云
void visualizer_points(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (target_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (target_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem (1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
    }
}

void test()
{
    for (int i = 1; i < 1000; i++)
    {
        for (int j = 1; j < 1000; j++)
        {
            for (int k = 1; k < 1000; k++)
            {
                double t = std::sqrt( (i * i / k + j * j / i + k*k/j )); 
            }
        }
    }
}

int main()
{
    // std::string cloud_surf_path = "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loam_surf.pcd";
    // std::string cloud_edge_path = "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loam_edge.pcd";
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_surf(
    //     new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge(
    //     new pcl::PointCloud<pcl::PointXYZ>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_surf_path, *cloud_surf) == -1)//*打开点云文件
	// {
	// 	PCL_ERROR("Couldn't read file test_pcd.pcd\n");
	// 	return(-1);
	// }
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_edge_path, *cloud_edge) == -1)//*打开点云文件
	// {
	// 	PCL_ERROR("Couldn't read file test_pcd.pcd\n");
	// 	return(-1);
	// }

    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // float theta = 5 * M_PI / 180 ;      // The angle of rotation in radians
    // transform (0,0) = cos (theta);
    // transform (0,1) = -sin(theta);    
    // transform (1,0) = sin (theta);     
    // transform (1,1) = cos (theta);
    // transform (0,3) = 0.9;
    // transform (1,3) = 0.4;
    // transform (2,3) = 0.5;
    // std::cout<<"transform: "<<transform<<std::endl;
    // // Executing the transformation
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_surf_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud (*cloud_surf, *transformed_surf_cloud, transform);
    // //visualizer_points(cloud_surf, transformed_surf_cloud);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_edge_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud (*cloud_edge, *transformed_edge_cloud, transform);
    // //visualizer_points(cloud_edge, transformed_edge_cloud);

    // std::unique_ptr<Algorithm::RegistrationBase<pcl::PointXYZ>> registration_ptr;
    // registration_ptr.reset(
    //     new Algorithm::EdgeSurfFeatureRegistration<pcl::PointXYZ>("loam_edge", "loam_surf")); 

    // registration_ptr->SetInputSource(make_pair("loam_edge", cloud_edge));
    // registration_ptr->SetInputSource(make_pair("loam_surf", cloud_surf));
    
    // Algorithm::VoxelGridFilter<pcl::PointXYZ> down_sampling_edge_; 
    // Algorithm::VoxelGridFilter<pcl::PointXYZ> down_sampling_surf_;  
    // down_sampling_edge_.Reset("VoxelGrid", 0.1);
    // down_sampling_surf_.Reset("VoxelGrid", 2 * 0.1);
    // std::cout<<"before downsample size: "<<transformed_edge_cloud->size()<<std::endl;
    // transformed_edge_cloud = down_sampling_edge_.Filter(transformed_edge_cloud); 
    // transformed_surf_cloud = down_sampling_surf_.Filter(transformed_surf_cloud); 
    // std::cout<<"after downsample size: "<<transformed_edge_cloud->size()<<std::endl;
    // Sensor::CloudContainer<pcl::PointXYZ> input;
    // input.pointcloud_data_.insert(make_pair("loam_edge", transformed_edge_cloud)); 
    // input.pointcloud_data_.insert(make_pair("loam_surf", transformed_surf_cloud)); 
    // registration_ptr->SetInputTarget(input); 
    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // registration_ptr->Solve(T); 
    // std::cout<<"T: "<<T.matrix()<<std::endl;
    // std::cout<<"new"<<std::endl;
    // TicToc tt;
    // tt.tic();
    // for (int i = 1; i < 1000; i++)
    // {
    //     for (int j = 1; j < 1000; j++)
    //     {
    //         for (int k = 1; k < 1000; k++)
    //         {
    //             double t = std::sqrt( (i * i / k + j * j / i + k*k/j ) / 0.45534); 
    //         }
    //     }
    // }
    // tt.toc("test ");
    TicToc tt;
    tt.tic();
    #pragma omp parallel for num_threads(2)
    for (uint8_t i = 0; i < 2; i++)
    {
        // std::cout<<"OpenMP Test, 线程编号为: "<< omp_get_thread_num()<<std::endl;
        std::cout<<"begin Process, id "<<i<<std::endl;
        TicToc tt;
        tt.tic();
        test(); 
        tt.toc("test ");
        // std::cout<<"id: "<<static_cast<uint16_t>(id)<<"process done, before: "<<data.lidar_data_container_[i].first.size() <<
        // " after: "<< feature_frame[id]->size()<<std::endl;
    }
    tt.toc("all test ");
    return 1;
}