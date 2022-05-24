//
// Created by JJB on 2022-5-5.
//
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main() {
//    1.创建点云保存对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_negative(new pcl::PointCloud<pcl::PointXYZ>);

//    2.读入点云
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);

//    3.创建同居滤波进行离群过滤
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    输入点云
    sor.setInputCloud(cloud);
//    设置临近点数目
    sor.setMeanK(50);
//    设置方差
    sor.setStddevMulThresh(1);
//    执行滤波
    sor.filter(*cloud_filtered);
//    展示
    pcl::visualization::PCLVisualizer viewer("统计滤波");
    int v1 = 0;
    int v2 = 1;
    int v3 = 2;
//    分窗口，坐标为左下角
    viewer.createViewPort(0, 0, 0.5, 1, v1);//左边
    viewer.createViewPort(0.5, 0.5, 1, 1, v2);//右上
    viewer.createViewPort(0.5, 0, 1, 0.5, v3);//左上

    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);
    sor.setNegative(true);
    sor.filter(*cloud_filtered_negative);
    viewer.addPointCloud(cloud_filtered_negative, "cloud_filtered", v3);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}
