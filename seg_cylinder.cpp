//
// Created by JJB on 2022-5-14.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int
main(int argc, char **argv) {
    //------------------------------读取点云数据---------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
    cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;
    //--------------------------------直通滤波-----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");//将Z轴不在（0，1.5）范围内的点过滤掉
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);//剩余的点储存在cloud_filtered中后续使用
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
    //--------------------------------计算法线-----------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    n.setSearchMethod(kdtree);
    n.setInputCloud(cloud_filtered);
    n.setKSearch(50);
    n.compute(*normals);
    //------------------------------创建分割、提取、保存的对象---------------------------------
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;//分割对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;//提取点云对象
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients); //保存分割平面模型参数
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients); //保存分割圆柱模型参数
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);//保存平面内点索引
    pcl::PointIndices::Ptr outliers_plane(new pcl::PointIndices);//保存平面外点索引
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//保存圆柱内点索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());//保存平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outplane(new pcl::PointCloud<pcl::PointXYZ>());//保存平面外点云
    pcl::PointCloud<pcl::Normal>::Ptr normal_outplane(new pcl::PointCloud<pcl::Normal>);//保存平面外点法线
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());//保存圆柱

    //-------------------------------提取平面模型（不提取平面不能很好的拟合圆柱）------------------------------
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals);
    seg.segment(*inliers_plane, *coefficients_plane);//获取平面模型系数和平面上的点
    cout << "Plane coefficients: " << *coefficients_plane << endl;
    //-----------------------------提取平面----------------------------
    //提取平面
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;
    //提取平面外的点
    extract.setNegative(true);
    extract.filter(*cloud_outplane);
    //提取平面外的法线
    pcl::ExtractIndices<pcl::Normal> extract_Normal;
    extract_Normal.setInputCloud(normals);
    extract_Normal.setIndices(inliers_plane);
    extract_Normal.setNegative(true);
    extract_Normal.filter(*normal_outplane);
    //-------------------------------提取圆柱体模型------------------------------
    //为圆柱体分割创建分割对象，并设置参数
    seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
    seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
    seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
    seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
    seg.setMaxIterations(5000);               //设置迭代的最大次数
    seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值
    seg.setRadiusLimits(0, 0.1);              //设置估计出圆柱模型的半径范围
    seg.setInputCloud(cloud_outplane);
    seg.setInputNormals(normal_outplane);
    //获取圆柱模型系数和圆柱上的点
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
    //-----------------------------提取圆柱----------------------------
    extract.setInputCloud(cloud_outplane);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);
    //---------------可视化，从左到右依次是直通滤波后点云,平面,圆柱------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("segment display"));
    //直通滤波后点云
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.33, 1, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addPointCloud(cloud_filtered, "cloud_filtered", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_filtered");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
    //平面
    int v2(1);
    viewer->createViewPort(0.33, 0.0, 0.66, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addPointCloud(cloud_plane, "cloud_plane", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_plane");
    //圆柱
    int v3(2);
    viewer->createViewPort(0.66, 0.0, 1, 1, v3);
    viewer->setBackgroundColor(0, 0, 0, v3);
    viewer->addPointCloud(cloud_cylinder, "cloud_cylinder", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_cylinder");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cylinder");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}