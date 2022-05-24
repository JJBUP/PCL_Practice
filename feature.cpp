//
// Created by JJB on 2022-5-7.
//
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
int main() {

//    读入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd", *cloud);
//    降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr vg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
    vg_filter.setLeafSize(0.01,0.01,0.01);
    vg_filter.setInputCloud(cloud);
    vg_filter.filter(*vg_cloud);
//    估算法向量特征
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(vg_cloud);//设定输入点云
//    设定索引
    // 准备一个indices索引集合，为了简单起见，我们直接使用点云的前10%的点
//    std::vector<int> indices (std::floor (cloud->points.size () / 10));
//    for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;
//    // 设置indices索引
//
//    std::shared_ptr<const pcl::Indices> indicesptr (new std::vector<int> (indices));
//    normalEstimation.setIndices (indicesptr);
//    设定搜索面
    normalEstimation.setSearchSurface(cloud);

//    设定搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdTree);//指定一棵KDtree
    normalEstimation.setRadiusSearch(0.03);//设定半径搜索范围为3厘米
//    计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);//用于保存法向量
    normalEstimation.compute(*cloud_normal);
//    可视化

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("normal"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(vg_cloud, cloud_normal);//这个可视化只展示法向量
    std::cerr<<"normal has computed end"<<endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}