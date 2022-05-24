//
// Created by JJB on 2022-5-17.
//
#include<iostream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/features/normal_3d_omp.h>//法线特征
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2

int main(){
    //读取点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("bunny.pcd",*cloud);
    //计算法线
    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> normalEstimation;
    normalEstimation.setNumberOfThreads(8);
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(kdtree);//使用search下面的kdtree
    normalEstimation.setRadiusSearch(0.02);
//    normalEstimation.setViewPoint(0,0,0);
    normalEstimation.compute(*normal_cloud);
    std::cout<<"normal has bean entimated"<<std::endl;
    //估计PFH
    pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125>pfh;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_feature(new pcl::PointCloud<pcl::PFHSignature125>());//phf特征
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normal_cloud);
    pfh.setSearchMethod(kdtree);
    pfh.setRadiusSearch(0.02);
    pfh.compute(*pfh_feature);
    std::cout<<"cloud point size is:"<<cloud->size()<<std::endl;
    std::cout<<"fpfh feature size is:"<<pfh_feature->size()<<std::endl;
    //可视化
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfh_feature,300);
    plotter.plot();

    return 0;

}