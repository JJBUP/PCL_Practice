//
// Created by JJB on 2022-5-10.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv) {
//    读取原始点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);  // 获取pcd文件

//-------------使用多项式对点云数据进行重构，并计算法向量
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建用于搜索的kd树
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointNormal>);//用于保存优化后的点云和法线
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;//创建用于多项式重构的变量
    mls.setComputeNormals(true);//设置计算法向量
    //设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    // 重构
    mls.process(*mls_cloud);  // void 	process (PointCloudOut &output) override
//-----------------------获得原始点云法向量
    //原数据的法线feature
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    //降采样后的点作为索引点估计法向量
    pcl::PointCloud<pcl::PointXYZ>::Ptr vg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setLeafSize(0.01, 0.01, 0.01);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*vg_cloud);

    //计算保存原始法向量
    pcl::PointCloud<pcl::Normal>::Ptr org_vg_normal(new pcl::PointCloud<pcl::Normal>);
    normal_estimation.setInputCloud(vg_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(kdTree);//指定一棵KDtree
    normal_estimation.setRadiusSearch(0.03);//设定半径搜索范围为3厘米
    normal_estimation.compute(*org_vg_normal);
//-----------------------可视化
    pcl::visualization::PCLVisualizer viewer;
    int v0 = 0, v1 = 1;
    viewer.createViewPort(0,0,1,0.5,v0);
    viewer.createViewPort(0,0.5,1,1,v1);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(vg_cloud,org_vg_normal,100,0.02,"vg_cloud",v0);
    viewer.addPointCloudNormals<pcl::PointNormal>(mls_cloud,100,0.02,"mls_cloud",v1);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}
