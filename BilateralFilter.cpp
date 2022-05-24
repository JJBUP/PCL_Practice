//
// Created by JJB on 2022-5-5.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/visualization/pcl_visualizer.h>
void bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    // Apply the filter
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(input);
    fbf.setSearchMethod(tree1);
    fbf.setStdDev(0.1);
    fbf.setHalfSize(0.1);
    fbf.filter(*output);
}
int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 需要PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>(argv[1], *cloud);

    bilateralFilter(cloud, cloud_filtered);
    return (0);
}