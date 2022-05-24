#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>               // NDT配准
#include <pcl/filters/voxel_grid.h> // 体素滤波
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
int main() {

    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("wolf1.pcd", *source_cloud);
    pcl::io::loadPCDFile("wolf2.pcd", *target_cloud);
    cout<<"load finish"<<endl;
    if (source_cloud->empty() || target_cloud->empty()) {
        std::cerr << "please check pcd files are correct!" << std::endl;
        return -1;
    } else {
        std::cerr << "source cloud size:" << source_cloud->size() << std::endl;
        std::cerr << "target cloud size:" << target_cloud->size() << std::endl;
    }

    // -------------NDT进行配准--------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(source_cloud);    // 设置要配准的点云
    ndt.setInputTarget(target_cloud);   // 设置点云配准目标
    ndt.setStepSize(2);                 // 为More-Thuente线搜索设置最大步长
    ndt.setResolution(0.1);             // 设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setMaximumIterations(35);       // 设置匹配迭代的最大次数
    ndt.setTransformationEpsilon(0.01); // 为终止条件设置最小转换差异
    //-------------计算变换矩阵--------------
    ndt.align(*ndt_source_cloud);
    cout << "NDT has converged:" << ndt.hasConverged()
         << " score: " << ndt.getFitnessScore() << endl;
    cout << "Applied " << ndt.getMaximumIterations() << endl;
    cout << "MATRIX:\n" << ndt.getFinalTransformation() << endl;
//    //使用变换矩阵对未过滤的源点云进行变换
//    pcl::transformPointCloud(*source_cloud, *output_cloud, ndt.getFinalTransformation());

    //-------------点云可视化----------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D-NDT Registration"));

    //对源点云着色并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_color, "source cloud");
    //对目标点云着色（红色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    //对转换后的源点云着色（绿色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ndt_source_color(ndt_source_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(ndt_source_cloud, ndt_source_color, "output cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return (0);
}