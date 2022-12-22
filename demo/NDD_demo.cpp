
/**
  * @file NDD_demo.cpp
  * @author julian 
  * @date 12/21/22
 */
 
// local lib
#include "NDD.h"
#include "demo/util.h"

// c++ standard lib
#include <iostream>
#include <thread>
#include <set>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

namespace fs = std::filesystem;

void print_usage()
{
    std::cout << "usage: ./ndd_demo <poses.txt> <pcl_dir>\n"
              << "       Poses are expected to contain at least x, y, z coordinates in a txt or csv file\n"
              << "       Keyframes are expected to be .pcd files\n";
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        print_usage();
        return 0;
    }
    
    auto poses_txt = fs::path(argv[1]);
    auto keyframe_dir = fs::path(argv[2]);
    
    ndd::NDDManager ndd;
    pcl::PointCloud<pcl::PointXYZ> path;
    std::vector<fs::path> keyframe_paths;
    demo::read_txt(poses_txt, path);
    demo::index_keyframes(keyframe_dir, keyframe_paths);
    
    if (path.size() != keyframe_paths.size())
    {
        throw std::runtime_error("number of keyframes and poses to now match!");
    }
    
    pcl::Correspondences correspondences;
    correspondences.reserve(path.size() / 5);
    auto path_corr = path;
    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (const auto& pcl_file: keyframe_paths)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcl_file, cloud) != 0)
        {
            throw std::runtime_error("Couldn't readfile" + pcl_file.string());
        }
    
        ndd.makeAndSaveNDDAndKeys(cloud);
        auto [curr_id, loop_id, yaw] = ndd.detectLoopClosureID();
        if (loop_id != -1)
        {
            correspondences.emplace_back(demo::generate_correspondence(curr_id, loop_id, path));
        }
    }
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(255, 255, 255);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(path.makeShared(), 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(path.makeShared(), single_color1, "path");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "path");
    
    viewer->addCorrespondences<pcl::PointXYZ>(path.makeShared(), path_corr.makeShared(), correspondences, "correspondences");
    
    viewer->initCameraParameters();
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    
    return 0;
}
