//
// Created by JJB on 2022-5-14.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main(int argc, char **argv) {
    // ------------------加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1) {
        PCL_ERROR("Cloud reading failed.");
        return (-1);
    }
    // ------------------体素滤波下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_grid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;//创建体素滤波对象
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);//设置叶子大小为1cm
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*cloud_voxel_grid);

    // ------------------欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    vector<pcl::PointIndices> cluster_indices; // 聚类索引
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
    ec.setClusterTolerance(0.2);               // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
    ec.setMinClusterSize(100);                 // 设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(25000);               // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(kd_tree);               // 设置点云的搜索机制
    ec.setInputCloud(cloud_voxel_grid);        // 设置输入点云
    ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
    std::cout << "cluster_indices has finished,size =:" << cluster_indices.size()<< endl;

    // ------------------保存聚类结果并可视化
    //提取点云
    pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
    extractIndices.setNegative(false);
    extractIndices.setInputCloud(cloud_voxel_grid);
    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setWindowName("欧式聚类");

    int begin = 0;
    for (pcl::PointIndices cluster_i: cluster_indices) {

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = cluster_i;
        extractIndices.setIndices(indices);
        extractIndices.filter(*cloud_cluster);

        // 可视化相关的代码
        int R = 255 * rand() / (RAND_MAX + 1.0);
        int G = 255 * rand() / (RAND_MAX + 1.0);
        int B = 255 * rand() / (RAND_MAX + 1.0);
        begin++;
        string cloud_name = "cloud" + to_string(begin);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, R, G, B);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, color, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
    }
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return (0);
}