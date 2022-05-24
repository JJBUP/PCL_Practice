#include <iostream>
#include<pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/pcd_io.h>

int main() {
    //������ƶ��� ָ��
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passT;
    pcl::visualization::CloudViewer viewer("pcd��viewer");// ��ʾ���ڵ�����
    //���������������
    cloud_ptr->width = 10000;
    cloud_ptr->height = 1;
    cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
     for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
        cloud_ptr->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_ptr->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_ptr->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
//    //�������
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/CLionProjects/PCL/PCL_practice/homework.pcd"
//                                            , *cloud_ptr) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file homework.pcd \n");
//        return (-1);
//    }

//    ���δ�˲�ǰ�ĵ�
//    std::cout << "Loaded "
//              << cloud_ptr->width * cloud_ptr->height
//              << " data points from test_pcd.pcd with the following fields: "
//              << std::endl;
//    for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
//        std::cout << "    " << cloud_ptr->points[i].x
//                  << " " << cloud_ptr->points[i].y
//                  << " " << cloud_ptr->points[i].z << std::endl;


    // �����˲�������

    passT.setInputCloud(cloud_ptr);//�����������
    passT.setFilterFieldName("z");
    passT.setFilterLimits(0, 512);
    // pass.setKeepOrganized(true); // ���� ������ƽṹ===============
    //pass.setFilterLimitsNegative (true);//��־Ϊfalseʱ������Χ�ڵĵ�
    passT.filter(*cloud_filtered_ptr);
    // ����˲���ĵ���
//    std::cerr << "Cloud after filtering�˲���: " << std::endl;
//    for (size_t i = 0; i < cloud_filtered_ptr->points.size(); ++i)
//        std::cerr << "    " << cloud_filtered_ptr->points[i].x << " "
//                  << cloud_filtered_ptr->points[i].y << " "
//                  << cloud_filtered_ptr->points[i].z << std::endl;
    // ������ӻ�

//    viewer.showCloud(cloud_ptr,"δ����");
    viewer.showCloud(cloud_filtered_ptr,"���˺�");
    while (!viewer.wasStopped()) {
        // Do nothing but wait.
    }

    return (0);
}
