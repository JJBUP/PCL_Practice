//
// Created by JJB on 2022-5-14.
//
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>//区域生长
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using PointT = pcl::PointXYZ;

int
main(int argc, char** argv)
{
    // -------------------------------------------加载点云---------------------------------------------
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile <PointT>("office2.pcd", *cloud) == -1)
    {
        PCL_ERROR("Cloud reading failed.");
        return (-1);
    }
    //---------------------------------------法线和表面曲率估计---------------------------------------
    pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
    n.setInputCloud(cloud);   // 设置法线估计对象输入点集
    n.setSearchMethod(tree);  // 设置搜索方法
    n.setNumberOfThreads(6);  // 设置openMP的线程数
    n.setKSearch(50);         // 设置用于法向量估计的k近邻数目
    n.compute(*normals);      // 计算并输出法向量

    //--------------------------------------------区域生长-------------------------------------------
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(1000);                         // 一个聚类需要的最小点数
    reg.setMaxClusterSize(1000000);                    // 一个聚类需要的最大点数
    reg.setSearchMethod(tree);                         // 搜索方法
    reg.setNumberOfNeighbours(50);                     // 搜索的邻域点的个数
    reg.setInputCloud(cloud);                          // 输入点云
    reg.setInputNormals(normals);                      // 输入的法线
    reg.setSmoothnessThreshold(pcl::deg2rad(3.0));     // 设置平滑阈值，即法向量夹角的阈值(这里的3.0是角度制)
    reg.setCurvatureThreshold(1.0);                    // 设置曲率的阈值

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);                             // 获取聚类的结果，分割结果保存在点云索引的向量中。

    std::cout << "region growing class num:" << clusters.size() << std::endl;        // 输出聚类的数量
    std::cout << "first class point num:" << clusters[0].indices.size() <<  endl; // 输出第一个聚类的数量

    //----------------------------------------可视化结果----------------------------------------
    //获得完整点云cloud 的彩色版，自动根据区域生长算法上色
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);// 显示RGB
    viewer->setWindowName("区域生长分割");
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}