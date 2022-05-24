//
// Created by JJB on 2022-5-6.
//
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

int
main(int, char**)
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;
    CloudType::Ptr cloud(new CloudType);
    cloud->is_dense = false;
    // 生成点云
    PointType p;
    for (unsigned int i = 0; i < 5; ++i)
    {
        p.x = p.y = p.z = static_cast<float> (i);
        cloud->push_back(p);
    }

    std::cout << "Cloud has " << cloud->points.size() << " points." << std::endl;

    //-----------根据索引提取点云------------------
    // 1、取得需要的索引
    pcl::PointIndices indices;
    indices.indices.push_back(0);
    indices.indices.push_back(2);
    // 2、索引提取器
    pcl::ExtractIndices<PointType> extr;
    extr.setInputCloud(cloud);//设置输入点云
    extr.setIndices(std::make_shared<const pcl::PointIndices>(indices));//设置索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    extr.filter(*output);     //提取对应索引的点云

    std::cout << "Output has " << output->points.size() << " points." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);

    extr.setNegative(true);   // 提取对应索引之外的点
    extr.filter(*cloud_other);

    std::cout << "Other has " << cloud_other->points.size() << " points." << std::endl;

    return (0);
}