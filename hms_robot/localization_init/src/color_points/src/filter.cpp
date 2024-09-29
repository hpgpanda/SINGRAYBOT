#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "sensor_msgs/PointCloud2.h"

int main()
{
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/gk/外高桥煤厂/2chang.pcd", *cloud) == -1)
    {
        std::cerr << "Failed to load input point cloud!" << std::endl;
        return -1;
    }

    // 创建体素滤波对象
    pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setInputCloud(cloud);
    voxelgrid.setLeafSize(0.3f, 0.3f, 0.3f);  // 设置体素网格的尺寸

    // 执行体素滤波
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    voxelgrid.filter(*filteredCloud);


    // 保存滤波后的点云数据
    pcl::io::savePCDFileBinary("/home/gk/外高桥煤厂/B_filter.pcd", *filteredCloud);
    std::cout << "Filtered point cloud saved." << std::endl;

    return 0;
}
