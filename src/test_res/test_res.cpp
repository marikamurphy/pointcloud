#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointType;

int
main (int argc, char** argv)
{

    pcl::PointCloud<PointType>::Ptr source_cloud;
    int cloudNumber = 1000;

    std::string file_name = "rotated.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;

  if (pcl::io::loadPCDFile (full_file_path.str(), *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << std::endl;
      
    }


    return 0;
}