//
// Created by JJB on 2022-5-18.
//
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

int main() {
    //---------加载点云---------
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("wolf1.pcd", *source_cloud);
    pcl::io::loadPCDFile("wolf2.pcd", *target_cloud);
    std::cerr << "source cloud size:" << source_cloud->size() << std::endl;
    std::cerr << "target cloud size:" << target_cloud->size() << std::endl;

    //---------初始ICP---------
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);// 源点云
    icp.setInputTarget(target_cloud);// 目标点云
    //该设置需要查看soource 和target的大小，以及距离大小，不然设置最大距离过小将会提前结束
    //icp.setMaxCorrespondenceDistance(1);// 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setMaximumIterations(50);// 最大迭代次数
    icp.setEuclideanFitnessEpsilon(0.001);// 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setTransformationEpsilon(1e-10);// 为终止条件设置最小转换差异
    icp.align(*icp_cloud);
    std::cout << icp.getRANSACIterations() << endl;
    std::cout << std::endl << "ICP has converged, score : " << icp.getFitnessScore() << std::endl;
    std::cout << "transformation Matrix : " << std::endl << icp.getFinalTransformation() << std::endl;
    //---------可视化---------
    pcl::visualization::PCLVisualizer viewer;
    viewer.setWindowName("ICP");
    int v0 = 0, v1 = 1;
    viewer.createViewPort(0, 0, 0.5, 1, v0);
    viewer.createViewPort(0.5, 0, 1, 1, v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_color();
    viewer.addPointCloud(target_cloud, target_color, "target_cloud", v0);
    viewer.addPointCloud(source_cloud, source_color, "source_cloud", v0);
    viewer.addPointCloud(target_cloud, target_color, "traget_cloud", v1);
    viewer.addPointCloud(icp_cloud, source_color, "icp_cloud", v1);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}