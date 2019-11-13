#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#define NUM_ROTATIONS 12

typedef pcl::PointXYZRGBA PointType;
void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber );
void transform_pointCloud(pcl::PointCloud<PointType>::Ptr source_cloud, int cloudNumber);

int
main (int argc, char** argv)
{

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation");
  //viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  

  for(int i = 0; i < NUM_ROTATIONS; i++){
    // Load pcd file
    pcl::PointCloud<PointType>::Ptr source_cloud (new pcl::PointCloud<PointType> ());
    transform_pointCloud(source_cloud, i);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud,  "transformed_cloud" +i);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud"+i);
    viewer.spinOnce();

  }
  

  
  //viewer.addPointCloud (transformed_cloud,  "transformed_cloud");

  
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}

//save each captured point cloud to .pcd file in the pcd_files folder (in build folder)
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber){
    
    std::string file_name = "rotated.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;
    //pcl::io::savePCDFile( full_file_path.str(), *cloud, true ); // Binary format
    pcl::io::savePCDFile( full_file_path.str(), *cloud, false ); // ASCII format
    std::cout << "Saving cloud #" << cloudNumber << endl;
    
    
}

void transform_pointCloud(pcl::PointCloud<PointType>::Ptr source_cloud, int cloudNumber){

    std::string file_name = "cloud.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;

  if (pcl::io::loadPCDFile (full_file_path.str(), *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << std::endl;
      
    }

  /*  Using a Affine3f
  */
  float theta = cloudNumber * M_PI * 12.5 / 180.0; // The angle of rotation in radians, M_PI
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of .9 meters on the z axis.
  //TODO... figure out how to set this
  transform.translation() << 0.0 , 0.0, -1.0;

   // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *source_cloud, transform);

  transform = Eigen::Affine3f::Identity();
  // The same rotation matrix as before; theta radians around Y axis
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

   // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *source_cloud, transform);

  // Print the transformation
  printf ("\nAffine3f transformation matrix:\n");
  std::cout << transform.matrix() << std::endl;
 
}
