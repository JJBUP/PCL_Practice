//
// Created by JJB on 2022-5-17.
//
#include<iostream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/features/normal_3d_omp.h>//法线特征
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include <pcl/filters/voxel_grid.h>// 直方图的可视化 方法2

int main() {



    //读取点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr org_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./table_scene_lms400.pcd", *org_cloud);
    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(org_cloud);
    voxelGrid.setLeafSize(0.02,0.02,0.02);
    voxelGrid.filter(*cloud);

    //计算法线
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setNumberOfThreads(8);
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(kdtree);//使用search下面的kdtree
    normalEstimation.setRadiusSearch(0.04);
//    normalEstimation.setViewPoint(0,0,0);
    normalEstimation.compute(*normal_cloud);
    std::cout << "normal has bean entimated" << std::endl;
    //估计FPFH
//    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33>fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(8);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature(new pcl::PointCloud<pcl::FPFHSignature33>());//phf特征
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normal_cloud);
    fpfh.setSearchMethod(kdtree);
    fpfh.setRadiusSearch(0.08);
    fpfh.compute(*fpfh_feature);
    std::cout << "cloud point size is:" << cloud->size() << std::endl;
    std::cout << "pfh feature size is:" << fpfh_feature->size() << std::endl;


    //可视化
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*fpfh_feature, 300, "cloud");
    plotter.plot();

    return 0;

}