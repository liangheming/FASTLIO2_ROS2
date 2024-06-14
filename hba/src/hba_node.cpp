#include <iostream>
#include <fstream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>

#include "hba/blam.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

void fromStr(const std::string &str, std::string &file_name, Pose &pose)
{
    std::stringstream ss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }
    assert(tokens.size() == 8);
    file_name = tokens[0];
    pose.t = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    pose.r = Eigen::Quaterniond(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7])).normalized().toRotationMatrix();
}

int main(int argc, char *argv[])
{
    pcl::PCDReader reader;
    Config config;

    double scan_resolution = 0.1;
    size_t skip_num = 1;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(scan_resolution, scan_resolution, scan_resolution);

    BLAM blam(config);

    std::filesystem::path p_dir(argv[1]);
    std::filesystem::path txt_file = p_dir / "poses.txt";
    if (!std::filesystem::exists(txt_file))
    {
        std::cout << "poses.txt not found" << std::endl;
        return 0;
    }
    std::ifstream ifs(txt_file);
    std::string line;
    std::string file_name;
    Pose pose;

    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> raw_points;

    while (std::getline(ifs, line))
    {

        fromStr(line, file_name, pose);
        std::filesystem::path pcd_file = p_dir / "patches" / file_name;
        if (!std::filesystem::exists(pcd_file))
        {
            std::cerr << "pcd file not found" << std::endl;
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        reader.read(pcd_file, *cloud);
        raw_points.push_back(cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        input_cloud->reserve(cloud->size());
        for (size_t i = 0; i < cloud->size(); i += skip_num)
        {
            input_cloud->push_back(cloud->points[i]);
        }

        voxel_grid.setInputCloud(input_cloud);
        voxel_grid.filter(*input_cloud);

        blam.addCloudAndPose(input_cloud, pose);
        // std::cout << file_name << ":" << pose.t.transpose() << std::endl;
        // std::cout << "cloud size:" << cloud->size() << std::endl;
    }
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PCDWriter writer;
    Vec<Pose> &poses = blam.poses();
    // std::cout << poses[0].r << std::endl;
    std::cout << poses[0].t.transpose() << std::endl;
    // for (size_t i = 0; i < raw_points.size(); i++)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     Eigen::Quaterniond qi(poses[i].r);
    //     V3D ti = poses[i].t;
    //     pcl::transformPointCloud(*raw_points[i], *temp_cloud, ti, qi);
    //     *cloud_world += *temp_cloud;
    // }
    // writer.writeBinaryCompressed("/home/zhouzhou/temp/cloud_world1.pcd", *cloud_world);

    // blam.buildVoxels();
    blam.optimize();

    // cloud_world->clear();
    std::cout << "==========" << std::endl;
    // std::cout << poses[0].r << std::endl;
    std::cout << poses[0].t.transpose() << std::endl;

    std::cout << blam.H().rows() << ":" << blam.H().cols() << std::endl;

    std::cout << blam.H().block<12, 12>(0, 0) << std::endl;
    // for (size_t i = 0; i < raw_points.size(); i++)
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     Eigen::Quaterniond qi(poses[i].r);
    //     V3D ti = poses[i].t;
    //     pcl::transformPointCloud(*raw_points[i], *temp_cloud, ti, qi);
    //     *cloud_world += *temp_cloud;
    // }
    // writer.writeBinaryCompressed("/home/zhouzhou/temp/cloud_world2.pcd", *cloud_world);

    return 0;
}