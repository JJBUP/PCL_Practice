#include <iostream>
#include<pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/pcd_io.h>

int main() {
    //定义点云对象 指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passT;
    pcl::visualization::CloudViewer viewer("pcd　viewer");// 显示窗口的名字
    //产生随机点云数据
    cloud_ptr->width = 10000;
    cloud_ptr->height = 1;
    cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
     for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
        cloud_ptr->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_ptr->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_ptr->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
//    //载入点云
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/CLionProjects/PCL/PCL_practice/homework.pcd"
//                                            , *cloud_ptr) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file homework.pcd \n");
//        return (-1);
//    }

//    输出未滤波前的点
//    std::cout << "Loaded "
//              << cloud_ptr->width * cloud_ptr->height
//              << " data points from test_pcd.pcd with the following fields: "
//              << std::endl;
//    for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
//        std::cout << "    " << cloud_ptr->points[i].x
//                  << " " << cloud_ptr->points[i].y
//                  << " " << cloud_ptr->points[i].z << std::endl;


    // 创建滤波器对象

    passT.setInputCloud(cloud_ptr);//设置输入点云
    passT.setFilterFieldName("z");
    passT.setFilterLimits(0, 512);
    // pass.setKeepOrganized(true); // 保持 有序点云结构===============
    //pass.setFilterLimitsNegative (true);//标志为false时保留范围内的点
    passT.filter(*cloud_filtered_ptr);
    // 输出滤波后的点云
//    std::cerr << "Cloud after filtering滤波后: " << std::endl;
//    for (size_t i = 0; i < cloud_filtered_ptr->points.size(); ++i)
//        std::cerr << "    " << cloud_filtered_ptr->points[i].x << " "
//                  << cloud_filtered_ptr->points[i].y << " "
//                  << cloud_filtered_ptr->points[i].z << std::endl;
    // 程序可视化

//    viewer.showCloud(cloud_ptr,"未过滤");
    viewer.showCloud(cloud_filtered_ptr,"过滤后");
    while (!viewer.wasStopped()) {
        // Do nothing but wait.
    }

    return (0);
}
