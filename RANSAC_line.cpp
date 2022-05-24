//
// Created by JJB on 2022-5-13.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

int main() {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
//    {
//        PCL_ERROR("点云读取失败 \n");
//        return (-1);
//    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // populate our PointCloud with points
    cloud->width = 5000;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {

        if (i % 10 == 0) { // 直线
            cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);

        } else { // 5/4的点可能会散落在直线外
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
        }
    }


    //------------------------------------------RANSAC框架--------------------------------------------------------
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(
            new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);//定义RANSAC算法模型
    ransac.setDistanceThreshold(0.01);//设定距离阈值
    ransac.setMaxIterations(500);     //设置最大迭代次数
    ransac.setProbability(0.95);      //设置从离群值中选择至少一个样本的期望概率
    ransac.computeModel();            //拟合平面
    vector<int> inliers;              //用于存放内点索引的vector
    ransac.getInliers(inliers);       //获取内点索引


    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);  //获取拟合平面参数，coeff分别按顺序保存a,b,c,d

    cout << "平面模型系数coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << "\t "
         << coeff[4] << "\t " << coeff[5] << endl;


    //--------------------------------根据内点索引提取拟合的平面点云-----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr sac_line(new pcl::PointCloud<pcl::PointXYZ>);//设置为提取拟合到的线
    pcl::PointCloud<pcl::PointXYZ>::Ptr sac_none_line(new pcl::PointCloud<pcl::PointXYZ>);//设置为提取非拟合线的其他点
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);
    pcl::ExtractIndices<pcl::PointXYZ> extractIndices;//创建拟合对象
    extractIndices.setInputCloud(cloud);//设置输入点云
    pcl::PointIndices::Ptr indices_inliers(new pcl::PointIndices);//创建索引对象指针
    indices_inliers->indices.swap(inliers);//将ransac提取的索引赋值到索引指针
    extractIndices.setIndices(indices_inliers);//提取对象设置提取索引
    extractIndices.filter(*sac_line);//提取拟合线
    extractIndices.setNegative(true);
    extractIndices.filter(*sac_none_line);//提取拟合平面其他部分

    //-------------------------------------------可视化-------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show cloud line"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sac_line_color(sac_line, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sac_none_line_color(sac_none_line, 255, 255, 255);

    viewer->addPointCloud(sac_line,sac_line_color,"cloud");
    viewer->addPointCloud(sac_none_line,sac_none_line_color, "plane cloud");

/*    // 显示拟合出来的平面
    pcl::ModelCoefficients plane;
    plane.values.push_back(coeff[0]);
    plane.values.push_back(coeff[1]);
    plane.values.push_back(coeff[2]);
    plane.values.push_back(coeff[3]);

    viewer->addPlane(plane, "plane",v2);*/

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}
