//
// Created by JJB on 2022-5-13.
//
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <string>
using namespace std;

int
main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read("table_scene_lms400.pcd", *cloud);
    cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
    //1.******创建分割对象******
    //创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//用于保存参数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//用于保存内点索引
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必须配置，设置分割的模型类型、所用随机参数估计方法
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);// 距离阈值 单位m。距离阈值决定了点被认为是局内点时必须满足的条件//距离阈值表示点到估计模型的距离最大值。
    //2.******创建点云提取对象******
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    //3.******点云可视化******
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    //4.******提取平面个数******
    int plane_num = 2;
    //5.******循环分割平面******
    for (int i = 0; i < plane_num; i++){
        //分割平面
        seg.setInputCloud(cloud);//输入点云
        seg.segment(*inliers, *coefficients);//实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
        extract.setInputCloud(cloud);
        if (inliers->indices.size() == 0) {
            return (-1);
        }
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //-----------输出平面模型的系数 a,b,c,d-----------
        cout << "Model coefficients: " << coefficients->values[0] << " "
             << coefficients->values[1] << " "
             << coefficients->values[2] << " "
             << coefficients->values[3] << endl;
        cout << "Model inliers: " << inliers->indices.size() << endl;
        //提取平面
        extract.setNegative(false);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);
        //提取剩余点云
        extract.setNegative(true);
        extract.filter(*cloud);

        //可视化
        int R = 255 * rand() / (RAND_MAX + 1.0);
        int G = 255 * rand() / (RAND_MAX + 1.0);
        int B = 255 * rand() / (RAND_MAX + 1.0);
        string cloud_name = "cloud" + to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud_plane(cloud_plane, R, G, B);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, color_cloud_plane,cloud_name);

    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return (0);
}