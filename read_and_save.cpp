//
// Created by JJB on 2022-5-2.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) {
//    1.创建读取点云PointXYZ类型的指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//    2.读取点云
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_mug_stereo_textured.pcd", * cloud)==-1) {
        PCL_ERROR("Could not open pcd!");
        return(-1);
    }
//    3.可视化点云
    //使用cloudviewer 可视化
    pcl::visualization::CloudViewer viewer("could viewer");
    viewer.showCloud(cloud);
    // 循环判断是否退出
    while (!viewer.wasStopped()) {
        // 你可以在这里对点云做很多处理
    }

    // 使用PCLVisualizer可视化
    // 创建PCLVisualizer指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_visualization (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl_visualization->setBackgroundColor (0.05, 0.05, 0.05, 0);
    while (!pcl_visualization->wasStopped()){

    }
//    4.保存点云
}