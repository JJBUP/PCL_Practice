//
// Created by JJB on 2022-5-22.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

using namespace std;
// 基于Z梯度估计3D点云的SIFT关键点
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator () (const PointXYZ &p) const
        {
            return p.z;
        }
    };
}

int main(int argc, char *argv[])
{
    pcl::StopWatch watch; // 计时器
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("wolf1.pcd", *cloud_xyz);
    //-----------------------------SIFT算法参数----------------------------------
    const float min_scale = 0.1f;           // 设置尺度空间中最小尺度的标准偏差
    const int n_octaves = 3;                  // 设置尺度空间层数，越小则特征点越多
    const int n_scales_per_octave = 20;       // 设置尺度空间中计算的尺度个数
    const float min_contrast = 0.1f;       // 设置限制关键点检测的阈值
    //----------------------------SIFT关键点检测---------------------------------
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud_xyz);            // 设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);               // 创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);    // 设置限制关键点检测的阈值,高斯
    sift.compute(result);                     // 执行sift关键点检测，保存结果在result
    cout << "Extracted " << result.size() << " keypoints" << endl;
    cout << "SIFT uses time "<< watch.getTimeSeconds() << "s" << endl;

    //为了可视化需要将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
    //方法一
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);

    //方法二
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud_sift = *cloud_temp;
    pcl::PointXYZ point;
    for (int i = 0; i < result.size(); i++)
    {
        point.x = result.at(i).x;
        point.y = result.at(i).y;
        point.z = result.at(i).z;
        cloud_sift.push_back(point);
    }*/

    // pcl::io::savePCDFileBinary("SIFT.pcd", *cloud_temp);//保存成Bnary格式

    //---------------------可视化输入点云和关键点----------------------------
    pcl::visualization::PCLVisualizer viewer("Sift keypoint");
    viewer.setWindowName("SIFT keypoint ditect");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_xyz, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
    viewer.addPointCloud(cloud_temp, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}