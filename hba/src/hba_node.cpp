#include <iostream>
#include <fstream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include "hba/blam.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "hba/hba.h"

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

int main1(int argc, char *argv[])
{
    pcl::PCDReader reader;
    BLAMConfig config;

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
        voxel_grid.setInputCloud(cloud);
        voxel_grid.filter(*cloud);
        blam.insert(cloud, pose);
    }

    std::cout << "start optimize" << std::endl;
    std::cout << blam.poses()[0].t.transpose() << std::endl;
    blam.optimize();
    std::cout << "after optimize" << std::endl;
    std::cout << blam.poses()[0].t.transpose() << std::endl;
    std::cout << blam.H().block<12, 12>(0, 0) << std::endl;

    std::cout << "plane count with sub: " << blam.planeCount(true) << " :: " << blam.planes().size() << std::endl;
    std::cout << "plane count no sub: " << blam.planeCount(false) << std::endl;
    return 0;
}

int main(int argc, char *argv[])
{
    pcl::PCDReader reader;
    HBAConfig config;

    double scan_resolution = 0.1;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(scan_resolution, scan_resolution, scan_resolution);

    HBA hba(config);

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
        voxel_grid.setInputCloud(cloud);
        voxel_grid.filter(*cloud);
        hba.insert(cloud, pose);
    }
    pcl::io::savePCDFileBinary("/home/zhouzhou/temp/before_opt.pcd", *hba.getMapPoints());
    hba.optimize();
    pcl::io::savePCDFileBinary("/home/zhouzhou/temp/after_opt.pcd", *hba.getMapPoints());
    hba.writePoses("/home/zhouzhou/temp/refined_poses.txt");
}