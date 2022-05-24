#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>       // 根据索引提取点云
#include <pcl/filters/voxel_grid.h>            // 体素滤波
#include <pcl/kdtree/kdtree.h>                 // kd树
#include <pcl/sample_consensus/method_types.h> // 采样方法
#include <pcl/sample_consensus/model_types.h>  // 采样模型
#include <pcl/ModelCoefficients.h>             // 模型系数
#include <pcl/segmentation/sac_segmentation.h> // 随机采样分割
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类分割
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

int main()
{
    //--------------------------读取桌面场景点云---------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("city_4.pcd", *cloud);
    cout << "read point size: " << cloud->points.size()  << endl;

    //---------------------------体素滤波下采样----------------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);
    cout << "voxel grid filtered size : " << cloud_filtered->points.size() << endl;

    //--------------------创建平面模型分割的对象并设置参数-----------------------
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);    // 分割模型,平面模型
    seg.setMethodType(pcl::SAC_RANSAC);       // 参数估计方法,随机采样一致性　
    seg.setMaxIterations(100);                // 最大的迭代的次数
    seg.setDistanceThreshold(0.15);           // 设置符合模型的内点阈值

    // -------------sac分割平面，保存50%左右的点云,正好去除两个平面---------
    int n_points = (int)cloud_filtered->points.size();// 下采样后点云数量
    while (cloud_filtered->points.size() > 0.6 * n_points)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);// 分割
        if (inliers->indices.size() == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }
        //---------------------------根据索引提取点云-------------------------------
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);         // 提取符合平面模型的内点
        extract.setNegative(false);
        //--------------------------平面模型内点------------------------------------
        extract.filter(*cloud_plane);
        cout << "in_plane point size : " << cloud_plane->points.size() << "." << endl;
        //-------------------移去平面局内点，提取剩余点云---------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;         // 剩余点云
        cout << "out_plane point size : " << cloud_filtered->points.size() << "." << endl;
    }

    // --------------欧式聚类的算法对sac平面分割后的数据的聚类分割----------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    vector<pcl::PointIndices> cluster_indices;        // 点云团索引
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
    ec.setClusterTolerance(0.2);                     // 设置近邻搜索的搜索半径为2cm（也即两个不同聚类团点之间的最小欧氏距离）
    ec.setMinClusterSize(100);                        // 设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(25000);                      // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);                         // 设置点云的搜索机制
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);                      // 从点云中提取聚类，并将点云索引保存在cluster_indices中

    //------------可视化，迭代访问点云索引cluster_indices,给不同类的点云上色---------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("planar segment"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
    extractIndices.setInputCloud(cloud_filtered);
    int begin = 0;
    for (pcl::PointIndices cluster_i: cluster_indices) {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = cluster_i;
        extractIndices.setIndices(indices);
        extractIndices.filter(*cloud_cluster);

        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << endl;

        int R = 255 * rand() / (RAND_MAX + 1.0);
        int G = 255 * rand() / (RAND_MAX + 1.0);
        int B = 255 * rand() / (RAND_MAX + 1.0);
        begin++;
        string cloud_name = "cloud" + to_string(begin);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, R, G, B);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, color, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return (0);
    //ps:点云+= 为将两个点云connect，common中的算法，conmmon还能将normal 和xyz合起来
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_all(new pcl::PointCloud<pcl::PointXYZ>);
    //        *cloud_cluster_all += *cloud_cluster;
}