#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
// basic file operations
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <csignal>

#include <Eigen/Dense>
#include <Eigen/Geometry> //? do we need ?

//PCL stuff
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/color.h>


#include <boost/thread/thread.hpp> //sleep 
#include <unistd.h> //UNIX Standard function definitions
#include <fcntl.h> //File control
#include <errno.h> //Error number def
#include <termios.h> //POSIX terminal control
#include <termio.h>

using namespace std;
using namespace Eigen;
#define NUM_ROWS 18

ofstream myfile;
int numPoints = 0;
//double z_avg; //??????????????????
double y_avg;
MatrixXf points(NUM_ROWS,3);
string filename = "center_rotMat.txt";
bool alvar = true;

//stuff for alvar
Matrix3f globRotMat;
Vector3f globCenterMat;


typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>); 

void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber );
pcl::PointCloud<PointType>::Ptr filterCloud(pcl::PointCloud<PointType>::Ptr cloud, std::string axis, double minLimit, double maxLimit );


MatrixXf cartToHom(MatrixXf pointsC) {
    MatrixXf pointsH(pointsC.rows() + 1, pointsC.cols());
    for(size_t row = 0; row < pointsC.rows(); row++) {
        for(size_t col = 0; col < pointsC.cols(); col++) {
            pointsH(row, col) = pointsC(row, col);
        }
    }
    for(size_t col = 0; col < pointsC.cols(); col++) {
        pointsH(pointsC.rows(), col) = 1.0f;
    }
    return pointsH;
}

Matrix3f calc_rot_Mat(){
  JacobiSVD<MatrixXf> svd(cartToHom(points.transpose()), ComputeThinV);
  //the very last column describes the rotation to fit coordinates to plane
  MatrixXf vMat = svd.matrixV();
  //create 2 vectors to describe both coord systems
  Vector3f newCoord(vMat(0,3), vMat(1,3), vMat(2,3));
  Vector3f newCoord2(vMat(0,3), vMat(2,3), vMat(1,3));
  Vector3f oldCoord(0.0,0.0,1.0);
  Vector3f oldCoord2(0.0,1.0,0.0);
  //quaternion describes transformation b/w 2 vectors
  Quaternion<float> quad = Quaternion<float>::FromTwoVectors(newCoord,oldCoord);
  quad.normalize();
  Quaternion<float> quad2 = Quaternion<float>::FromTwoVectors(newCoord2,oldCoord2);
  quad2.normalize();
  //we transform the quaterion into a rotation matrix so we can easily multiply it by the points
  Matrix3f rotMat = quad.toRotationMatrix();
  cout << "rotMat" <<endl << rotMat<<endl;
  globRotMat = quad2.toRotationMatrix();
  cout << "globRotMat" <<endl << globRotMat<<endl;
  return rotMat;

}

/*Solution by method of least squares:
  A*c = b, c' = argmin(||A*c - b||^2)
  A = [x y 1], b = [x^2+y^2] */
Vector3f calc_center(Matrix3f rotMat){
  //we transpose all the points onto the plane
  MatrixXf rotatedPoints =  rotMat * points.transpose();
  cout <<"rotated points" <<endl<<rotatedPoints <<endl;
  //float zCenter = rotatedPoints.row(2).mean(); //save this for later
  float yCenter = rotatedPoints.row(2).mean(); //save this for later
  //now we go from 3D -> 2D homogenous points by setting z's = 1.0
  for(int col = 0; col < rotatedPoints.cols(); col++){
      rotatedPoints(2, col)= 1.0;
  }
  //transpose to put into correct form for svd
  MatrixXf A = rotatedPoints.transpose();
  cout << "A:"<<endl<< A <<endl;
  JacobiSVD<MatrixXf> svdCenter(A, ComputeFullV|ComputeFullU);
  //now calculate the b matrix to solve for (b = x^2+y^2)
  VectorXf b(A.rows());
  for (int i = 0; i < A.rows(); i++){
      b(i)= A(i,0)*A(i,0)+A(i,1)*A(i,1);
  }
  //solution: (2x, 2y, use to calc radius)
  Vector3f b_soln = svdCenter.solve(b);
  cout <<"soln"<<endl<< b_soln  <<endl;
  float xCenter = b_soln(0)/2;
  //float yCenter = b_soln(1)/2;
  float zCenter = b_soln(1)/2;
  //cout << "X: " << xCenter << endl <<"Y: "<<yCenter<<endl <<"Z"<<zCenter<<endl;
  Vector3f centerMat(xCenter,yCenter,zCenter);
  //cout <<"centerMat on plane: "<<centerMat<<endl<<"in our coords: " <<rotMat.transpose() * centerMat<<endl;
  cout <<"centerMat on plane: "<<centerMat<<endl;
  return centerMat;
}
/*
void saveMat(MatrixXf mat){
  myfile.open (filename);
  for(int r = 0; r < mat.rows(); r++){
    for(int c = 0; c < mat.cols(); c++){
      myfile << mat(r,c) << "\n";
    }
  }
  myfile.close();
}
 */

void saveVec(VectorXf vec){
  myfile.open (filename);
  for(int c = 0; c < vec.size(); c++){
    myfile << vec(c) << "\n";
  }
  myfile.close();
}

void cb(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
      
      double x, y, z;
      x=req.markers[0].pose.pose.position.x;
      y=req.markers[0].pose.pose.position.y;
      z=req.markers[0].pose.pose.position.z;
      ROS_INFO("x=%1.4f  y=%1.4f  z=%1.4f", x, y, z);
      
      double save=0;
      cout <<"Please enter: ";
      cin >> save;

      //if the user enters 1, save the coordinates, or calculate the center
      if(save == 1){
        //need to log at least NUM_ROWS to points
        if(numPoints < NUM_ROWS){
          points(numPoints,0) = x;
          //points(numPoints,1) = y;
          //points(numPoints,2) = z;
          points(numPoints,1) = z;
          points(numPoints,2) = y;
         
          y_avg+=y; 
          //z_avg +=z;
          numPoints+=1;
          cout << "num coords: " << numPoints << endl;
        }
        //once we have enough points, use least square to fit circle
        //and fine the center
        else{
          cout <<"points"<<endl<<points<<endl;
          Matrix3f rotMat = calc_rot_Mat();
          globCenterMat = calc_center(rotMat);
          //saveMat(rotMat);
          globCenterMat = globRotMat.transpose() * globCenterMat;
          saveVec(globCenterMat);
          cout << "File saved" <<endl;
          alvar = false;
        } 
      }
      //if the user enters 2, shutdown the program
      else if(save == 2){
            //alvar = false;
      }
      //if the user enters anything else, continue
      else{
        save = 0;
        //raise(SIGINT); //exit process
      }
    }// end if statement
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  pcl::fromPCLPointCloud2( *cloud, *cloud_ptr);
  cloud_ptr = filterCloud(cloud_ptr, "x", -0.5, 0.5);
  cloud_ptr = filterCloud(cloud_ptr, "y", -0.5, 0.5);
  cloud_ptr = filterCloud(cloud_ptr, "z", -0.5, 1.5);

  //Eigen::Affine3f transform(globRotMat);
  // The same rotation matrix as before; theta radians around Y axis
   // Executing the transformation
  //pcl::transformPointCloud (*cloud_ptr, *cloud_ptr, transform);

}


int main(int argc, char **argv) {

  
  ros::init(argc, argv, "arlistener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  cout <<"Enter: \n 1: log coordinates\n 2:  shutdown\n Anthing else to continue" <<endl;
  while(alvar && ros::ok()){
    ros::spinOnce();
  }


  // PCL Visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
      
//x=0.0583  y=0.1873  z=0.8153
  viewer->addCoordinateSystem (1.0, globCenterMat(0) , globCenterMat(1), globCenterMat(2), "Point Cloud Viewer", 0);

  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb);
  // Spin
  

  while( !viewer->wasStopped()  && ros::ok()){
        // Update Viewer
        ros::spinOnce ();
        viewer->spinOnce();
          if( cloud_ptr && cloud_ptr->size() != 0 ){
              /* Processing Point Cloud */
              // Update Point Cloud
              if( !viewer->updatePointCloud( cloud_ptr, "cloud" ) ){
                  viewer->addPointCloud( cloud_ptr, "cloud" );
                  viewer->resetCameraViewpoint( "cloud" );
              }
          }
        
    }

  
  return 0;

}

//save each captured point cloud to .pcd file in the pcd_files folder (in build folder)
void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber){
    
    std::string file_name = "cloud.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;
    //pcl::io::savePCDFile( full_file_path.str(), *cloud, true ); // Binary format
    pcl::io::savePCDFile( full_file_path.str(), *cloud, false ); // ASCII format
    std::cout << "Saving cloud #" << cloudNumber << endl;
    
    
}

//filter the cloud to fit within minLimit and maxLimit along axis
pcl::PointCloud<PointType>::Ptr filterCloud(pcl::PointCloud<PointType>::Ptr cloud, std::string axis, double minLimit, double maxLimit ){
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (minLimit, maxLimit);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    return cloud_filtered;

}