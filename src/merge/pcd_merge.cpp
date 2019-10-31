#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
namespace fs = std::filesystem;

typedef pcl::PointXYZRGBA PointType;
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber );
void readFilenames(std::vector<std::string> filenames, std::string filepath);

int
main (int argc, char** argv)
{

  // Grab all the PCD file names from the pcd_files folder
  std::vector<std::string> filenames;
  readFilenames(filenames, "pcd_files");

  // Load pcd file
  pcl::PointCloud<PointType>::Ptr source_cloud (new pcl::PointCloud<PointType> ());
    if (pcl::io::loadPCDFile (filenames[0], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << filenames[0] << std::endl << std::endl;
      return -1;
    }

  
    
  /*  Using a Affine3f
  */
  float theta = M_PI/4; // The angle of rotation in radians
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nAffine3f transformation matrix:\n");
  std::cout << transform.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

  
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation");

  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud,  "original_cloud");
  viewer.addPointCloud (transformed_cloud,  "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}

//save each captured point cloud to .pcd file in the pcd_files folder (in build folder)
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber){
    
    std::string file_name = "cloud.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;
    //pcl::io::savePCDFile( full_file_path.str(), *cloud, true ); // Binary format
    pcl::io::savePCDFile( full_file_path.str(), *cloud, false ); // ASCII format
    std::cout << "Saving cloud #" << cloudNumber << endl;
    
    
}

//read all the .pcd files from filepath
void readFilenames(std::vector<std::string> filenames, std::string filepath){
    
    for (const auto & entry : fs::directory_iterator(filepath)){
      std::cout << entry.path() << std::endl;
      filesnames.push_back(entry.path());
}
    }
}        