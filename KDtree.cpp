//
// Created by JJB on 2022-5-3.
//
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>

int main() {
//创建读取的PCL数据指针kdtree后的指针
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //加载pcd文件
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);


    //定义kdtree对象
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    //设置要搜索的点云对象，将点云数据存储到kdtree中
    kdtree.setInputCloud(cloud);
    //用于保存的紧邻点的索引,距离
    std::vector<int> indices_nearest;
    std::vector<float> distances_nearest;
    std::vector<int> indices_radius;
    std::vector<float> distances_radius;

    //设置起始点 p
    auto p1 = cloud->points[0];
    auto p2 = cloud->points[cloud->size()-1];



    //查询距离p最近的k个点
    int k = 10000;
    int size = kdtree.nearestKSearch(p1, k, indices_nearest, distances_nearest);
    std::cout << "search point : " << size << std::endl;



    // 查询point半径为radius邻域球内的点
    double radius = 0.5;
    size = kdtree.radiusSearch(p2, radius, indices_radius, distances_radius);
    std::cout << "search point : " << size << std::endl;


    //将原始点云渲染为绿色,方便后期展示
//
//    std::for_each(cloud->begin(), cloud->end(), [](pcl::PointXYZRGB& point)
//    {point.r = 0; point.g = 255; point.b = 0; });
    for(pcl::PointXYZRGB &cloud_point_i:cloud->points)
    {
        cloud_point_i.r=255;
        cloud_point_i.g=255;
        cloud_point_i.b=255;
    }

    //将查询到的索引改为红色
    for (auto i: indices_nearest) {
        cloud->points[i].r = 255;
        cloud->points[i].g = 0;
        cloud->points[i].b = 0;

    }

    for(auto i:indices_radius){
        cloud->points[i].r=0;
        cloud->points[i].g=0;
        cloud->points[i].b=255;
    }

    //可视化
    //创建可视化对象

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("kd tree"));
    int v1 = 0;

    viewer->addPointCloud(cloud,"kd_nearest_radius",v1);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    system("pause");
    return 0;

}