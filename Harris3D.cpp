//
// Created by JJB on 2022-5-22.
//
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    //读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("./wolf1.pcd", *cloud)) {
        std::cout << "pcd file read fail" << endl;
        return -1;
    }
    //-----------------------------Harris关键点提取----------------------------------
    //强度信息：注意此处PCL的point类型设置为<pcl::PointXYZI>,即除了x、y、z坐标还必须包含强度信息
    //        因为Harris的评估值保存在输出点云的(I)分量中，Harris输出点云中的(I)非传统点云中的强度信息
    //可视化：后续在保存和可视化输出点云的时候需要通过点的索引来重新获取。
    //-------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr harris_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);     // 提供指向输入数据集的指针
    harris.setMethod(harris.LOWE);   // 设置要计算响应的方法（可以不设置）
    harris.setRadiusSearch(0.6);     // 设置用于关键点检测的最近邻居的球半径
    harris.setRadius(0.6);           // 设置法线估计和非极大值抑制的半径。
    harris.setNonMaxSupression(true);// 是否应该应用非最大值抑制
    harris.setThreshold (0.002);     // 设置角点检测阈值，只有当非极大值抑制设置为true时才有效
    harris.setRefine(true);          // 检测到的关键点是否需要细化，设置为true时，关键点为点云中的点
    harris.setNumberOfThreads(4);    // 初始化调度程序并设置要使用的线程数
    harris.compute(*harris_cloud);
    std::cout << "keypoint size is: " << harris_cloud->size() << endl;
    //获得关键点
    //pcl::PointXYZI格式无法正确保存和显示，这里通过索引从原始点云中获取位于点云中特征点
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::ConstPtr keypoint_indices = harris.getKeypointsIndices();
    pcl::copyPointCloud(*cloud, *keypoint_indices, *keypoint_cloud);
    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->setWindowName("keypoint");
    viewer->setBackgroundColor(255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoint_color(cloud, 255, 0, 0);
    viewer->addPointCloud(cloud, color, "cloud");
    viewer->addPointCloud(keypoint_cloud, keypoint_color, "keypoint_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"keypoint_cloud");
    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
    }
}
