#include <iostream>
#include <fstream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>

#include "hba/blam.h"

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
        blam.addCloudAndPose(cloud, pose);
        // std::cout << file_name << ":" << pose.t.transpose() << std::endl;
        // std::cout << "cloud size:" << cloud->size() << std::endl;
    }
    blam.buildVoxels();

    std::cout << "plane count:" << blam.planeCount() << std::endl;
    int flat_count = 0;
    for (auto it = blam.voxelMap().begin(); it != blam.voxelMap().end(); it++)
    {
        if (it->second->isPlane())
        {
            flat_count++;
        }
    }
    std::cout << "flat count:" << flat_count << std::endl;
    std::cout << blam.voxelMap().size() << std::endl;

    return 0;
}