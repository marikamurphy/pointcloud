#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/statistical_outlier_removal.h>

//triangulation stuff
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>


#define NUM_ROTATIONS 29

typedef pcl::PointXYZRGBA PointType;
void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber );
void transform_pointCloud(pcl::PointCloud<PointType>::Ptr source_cloud, int cloudNumber);
void filterCloud(pcl::PointCloud<PointType>::Ptr cloud, std::string axis, double minLimit, double maxLimit );
void makeCube(pcl::PointCloud<pcl::PointXYZ>::Ptr cube){
  int n_step = 100;
  cube->width  = n_step * n_step * 6;
  cube->height = 1;
  cube->points.resize (cube->width * cube->height);
  //savePointCloudToFile(final_cloud, 1000);
  
float xe = 0.06;
float xs = 0.0;
float ye = 0.06;
float ys = 0.0;
float ze  = 0.06;
float zs = 0.0;

   //Compute edge sizes 
float x_size = xe - xs;
float y_size = ye - ys;
float z_size = ze - zs;

//Compute steps
float x_step = x_size/n_step;
float y_step = y_size/n_step;
float z_step = z_size/n_step;

//Points per edge (including ending point)
int p = 0;

  for (int i = 0; i < n_step; i++) { //move on x axis

    for (int j = 0; j < n_step; j++) { //move on y axis

        cube->points[p].x = xs + x_step * i;
        cube->points[p].y = ys + y_step * j;
        cube->points[p].z = zs;
        cube->points[p+1].x = xs + x_step * i;
        cube->points[p+1].y = ys + y_step * j;
        cube->points[p+1].z = ze;

        cube->points[p+2].x = xs + x_step * i;
        cube->points[p+2].y = ys;
        cube->points[p+2].z = zs + z_step * j;
        cube->points[p+3].x = xs + x_step * i;
        cube->points[p+3].y = ye;
        cube->points[p+3].z = zs + z_step * j;

        cube->points[p+4].x = xs;
        cube->points[p+4].y = ys + y_step * i;
        cube->points[p+4].z = zs + z_step * j;
        cube->points[p+5].x = xe;
        cube->points[p+5].y = ys + y_step * i;
        cube->points[p+5].z = zs + z_step * j;

        p+=6;
    }
  }
}
int
main (int argc, char** argv)
{
  
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation");
  //viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  
   //load in the very first cloud
    pcl::PointCloud<PointType>::Ptr final_cloud (new pcl::PointCloud<PointType> ());
    transform_pointCloud(final_cloud, 0);
    cout << "enter 0 for icp: ";
    int icp = 0;
    cin >>icp;
    cout <<endl;
  for(int i = 1; i < NUM_ROTATIONS; i++){
    
    // Load pcd file
    pcl::PointCloud<PointType>::Ptr cloud1 (new pcl::PointCloud<PointType> ());
    transform_pointCloud(cloud1, i);

    
    if(icp == 0){
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setInputSource(cloud1); //This cloud will be transformed to match
      icp.setInputTarget(final_cloud); //this cloud. The result is stored in the cloud provided as input below.
      icp.setMaximumIterations(1);
      icp.align(*cloud1); //Overwrite the source cloud with the transformed cloud.

    }
    

    *final_cloud = *final_cloud + *cloud1; //Merge the two now aligned and matched clouds.
    
  }
  
   // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointType> sor;
  sor.setInputCloud (final_cloud);
  sor.setMeanK (500);
  sor.setStddevMulThresh (.15);
  sor.filter (*final_cloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cube(new pcl::PointCloud<pcl::PointXYZ> ());
  makeCube(cube);


  // We add the point cloud to the viewer and pass the color handler
    //viewer.addPointCloud (cube,  "transformed_cloud" );
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "transformed_cloud");
    //viewer.spinOnce();
    //viewer.addPointCloud (final_cloud,  "transformed_cloud2" );
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud2");
   // viewer.spinOnce();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    copyPointCloud(*final_cloud, *cloud_xyz);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
     
    icp2.setInputSource(cloud_xyz); //This cloud will be transformed to match
     
    icp2.setInputTarget(cube); //this cloud. The result is stored in the cloud provided as input below.
     
    icp2.setMaximumIterations(1);
    
    icp2.align(*cloud_xyz);
    cout <<"fitness score: "<< icp2.getFitnessScore() << endl;
    
  
    // viewer.addPointCloud (cloud_xyz,  "transformed_cloud2" );
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud2");
    // viewer.spinOnce();


    //triangulation
    // Normal estimation*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  copyPointCloud(*final_cloud, *cloud);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (50);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.005);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(0); // 10 degrees
  gp3.setMaximumAngle(M_PI/4); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  pcl::io::saveVTKFile ("mesh.vtk", triangles);

  viewer.setBackgroundColor (0, 0, 0);
viewer.addPolygonMesh(triangles,"meshes",0);
//viewer.addCoordinateSystem (1.0);
viewer.initCameraParameters ();
  while (!viewer.wasStopped ()) { 
    viewer.spinOnce ();
  }

  return 0;
}

//save each captured point cloud to .pcd file in the pcd_files folder (in build folder)
void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber){
    
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
  float theta = cloudNumber * M_PI * 12 / 180.0; // The angle of rotation in radians, M_PI
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // The same rotation matrix as before; theta radians around Y axis
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

   // Executing the transformation
  pcl::transformPointCloud (*source_cloud, *source_cloud, transform);

  // Print the transformation
  //printf ("\nAffine3f transformation matrix:\n");
  //std::cout << transform.matrix() << std::endl;


  //now filter
  filterCloud(source_cloud, "x", -0.1, 0.1);
  filterCloud(source_cloud, "y", -0.1, 0.1);
  filterCloud(source_cloud, "z", 0.0, 0.2);

 
}

void filterCloud(pcl::PointCloud<PointType>::Ptr cloud, std::string axis, double minLimit, double maxLimit ){
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (minLimit, maxLimit);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    //we filter the cloud based off color: get rid of green
      // build the condition
    int rMin = 220;
    int gMax = 150;
    int bMin = 220;
    pcl::ConditionOr<PointType>::Ptr color_cond (new pcl::ConditionOr<PointType> ());
    color_cond->addComparison (pcl::PackedRGBComparison<PointType>::Ptr (new pcl::PackedRGBComparison<PointType> ("r", pcl::ComparisonOps::GT, rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<PointType>::Ptr (new pcl::PackedRGBComparison<PointType> ("g", pcl::ComparisonOps::LT, gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<PointType>::Ptr (new pcl::PackedRGBComparison<PointType> ("b", pcl::ComparisonOps::GT, bMin)));

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem;
  condrem.setCondition (color_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true); 

    // apply filter
    condrem.filter (*cloud);

}
