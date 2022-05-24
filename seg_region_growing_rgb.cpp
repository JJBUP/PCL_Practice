#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/filter.h>
int
main(int argc, char **argv) {

    // -----------------------------------------读取点云数据--------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("office2.pcd", *cloud_org) == -1) {
        PCL_ERROR("READ PCD ERROR!");
        return (-1);
    }
    //去除点云中的无效点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointIndices::Ptr cloud_indices(new pcl::PointIndices);
    pcl::Indices cloud_indices;
    pcl::removeNaNFromPointCloud(*cloud_org,*cloud,cloud_indices);


    // ------------------------------------法向量和表面曲率估计-----------------------------------
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);

    // ------------------------------------基于颜色的区域生长-------------------------------------
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setMinClusterSize(500);                        // 一个聚类需要的最小点数
    reg.setMaxClusterSize(1000000);                    // 一个聚类需要的最大点数
    reg.setSearchMethod(tree);                         // 搜索方法
    reg.setDistanceThreshold(10);                      // 设置距离阈值，用于聚类相邻点搜索
    reg.setPointColorThreshold(6);                     // 设置两点颜色阈值
    reg.setRegionColorThreshold(50);                   // 设置两类区域颜色阈值
    reg.setNumberOfRegionNeighbours(100);              // 此方法允许设置用于查找相邻段的邻居数。
    //可选参数
    reg.setSmoothModeFlag(true);                       // 设置是否使用平滑阈值，设置为true则需提供法线及法线夹角阈值
    reg.setSmoothnessThreshold(30 / 180.0 * M_PI);     // 设置平滑阈值，即法向量夹角的阈值
    reg.setNormalTestFlag(true);
    reg.setInputNormals(normals);                      // 法向量和表面曲率
    reg.setCurvatureTestFlag(true);                    // 设置是否使用曲率的阈值，设置为true则需提供曲率及曲率阈值
    reg.setCurvatureThreshold(0.05);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);                             // 获取分割聚类的结果，分割结果保存在点云索引的向量中。

    // -------------------------------------结果可视化--------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();//获得rgb分割后的带颜色的点云

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                             "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}