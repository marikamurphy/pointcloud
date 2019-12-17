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
#define NUM_ROWS 9

#define NUM_ROTATIONS 29

ofstream myfile;
int numPoints = 0;
MatrixXf points(NUM_ROWS,3);
MatrixXf points2(NUM_ROWS,3);//for basic way
string filename = "center_rotMat.txt";
bool alvar = true;
double x_avg = 0.0;
double y_avg = 0.0;
double z_avg = 0.0;
int fd;

//stuff for alvar
Vector3f globCenterMat;
Vector3f globCenterMat2;//for basic way

Matrix3f globRotMat;

Quaternionf globQuat;


typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>); 

int rotate(int fd);
void savePointCloudToFile(pcl::PointCloud<PointType>::Ptr cloud, int cloudNumber );
pcl::PointCloud<PointType>::Ptr filterCloud(pcl::PointCloud<PointType>::Ptr cloud, std::string axis, double minLimit, double maxLimit );

int open_port(){ //-1 is a error
  int port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  /*O_RDWR POSIX read write
  O_NOCTTY: not a "controlling terminal"
  O_NDELAY": Ignore DCD signal state*/

  if(port == -1){
    perror("open_port: Unable to open /dev/ttyACM0 - ");
  }else fcntl(port, F_SETFL, 0);
  return (port);
}

int set_port(int port){
  struct termios options;
  tcgetattr(port, &options);
  cfsetispeed(&options, B9600); //Typical way of setting baud rate. Actual baud rate are contained in the c_ispeed and c_ospeed members
  cfsetospeed(&options, B9600);
  
  options.c_cflag |= (CLOCAL|CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag |= CS8;     //No parity, 8bits, 1 stop bit (8N1)
  options.c_cflag &= ~CRTSCTS;//CNEW_RTSCTS; //Turn off flow control
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Make sure that Canonical input is off (raw input data)
  options.c_iflag &= ~(IXON | IXOFF | IXANY); //Turn off software control flow
  options.c_oflag &= ~OPOST; //Raw output data
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;

  return tcsetattr (port, TCSANOW, &options); //Make changes now without waiting for data to complete.
}

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
  centerMat = globRotMat.transpose() * centerMat;
  //cout <<"centerMat on plane: "<<centerMat<<endl<<"in our coords: " <<rotMat.transpose() * centerMat<<endl;
  cout <<"centerMat on plane: "<<centerMat<<endl;
  return centerMat;
}

void findCircle(float x1, float y1, float x2, float y2, float x3, float y3) 
{ 
    float x12 = x1 - x2; 
    float x13 = x1 - x3; 
  
    float y12 = y1 - y2; 
    float y13 = y1 - y3; 
  
    float y31 = y3 - y1; 
    float y21 = y2 - y1; 
  
    float x31 = x3 - x1; 
    float x21 = x2 - x1; 
  
    // x1^2 - x3^2 
    float sx13 = pow(x1, 2) - pow(x3, 2); 
  
    // y1^2 - y3^2 
    float sy13 = pow(y1, 2) - pow(y3, 2); 
  
    float sx21 = pow(x2, 2) - pow(x1, 2); 
    float sy21 = pow(y2, 2) - pow(y1, 2); 
  
    float f = ((sx13) * (x12) 
             + (sy13) * (x12) 
             + (sx21) * (x13) 
             + (sy21) * (x13)) 
            / (2 * ((y31) * (x12) - (y21) * (x13))); 
    float g = ((sx13) * (y12) 
             + (sy13) * (y12) 
             + (sx21) * (y13) 
             + (sy21) * (y13)) 
            / (2 * ((x31) * (y12) - (x21) * (y13))); 
  
    float c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1; 
  
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0 
    // where centre is (h = -g, k = -f) and radius r 
    // as r^2 = h^2 + k^2 - c 
    float h = -g; 
    float k = -f; 
    x_avg+=h;
    z_avg+=k;
    cout << "Centre = (" << h << ", " << k << ")" << endl; 
} 


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
      globQuat.x() = req.markers[0].pose.pose.orientation.x;
      globQuat.y() = req.markers[0].pose.pose.orientation.y;
      globQuat.z() = req.markers[0].pose.pose.orientation.z;
      globQuat.w() = req.markers[0].pose.pose.orientation.w;
      ROS_INFO("x=%1.4f  y=%1.4f  z=%1.4f w=%1.4f", globQuat.x(), globQuat.y(), globQuat.z(), globQuat.w());

      //double save=0;
      //cout <<"Please enter: ";
      //cin >> save;

      //if the user enters 1, save the coordinates, or calculate the center
      //if(save == 1){
        //need to log at least NUM_ROWS to points
        if(numPoints < NUM_ROWS){
          points(numPoints,0) = x;
          //points(numPoints,1) = y;
          //points(numPoints,2) = z;
          points(numPoints,1) = z;
          points(numPoints,2) = y;

          points2(numPoints,0) = x;
          points2(numPoints,1) = y;
          points2(numPoints,2) = z;
         
          numPoints+=1;
          cout << "num coords: " << numPoints << endl;
          rotate(fd);
        }
        //once we have enough points, use least square to fit circle
        //and fine the center
        else{
          cout <<"points"<<endl<<points<<endl;
          for(int r = 0; r < NUM_ROWS; r+=3){
            findCircle(points2(r,0), points2(r,2), points2(r+1,0), points2(r+1,2), points2(r+2,0), points2(r+2,2));
            y_avg += (points2(r,1) +points2(r+1,1) +points2(r+2,1));
          }
         
          globCenterMat2(0) = x_avg*3/numPoints;
          globCenterMat2(1) = y_avg/ numPoints;
          globCenterMat2(2) = z_avg*3/numPoints;

          Matrix3f rotMat = calc_rot_Mat();
          globCenterMat = calc_center(rotMat);
          cout << "method 1: " <<globCenterMat <<endl;
          //saveVec(globCenterMat);
          cout <<"method 2: "<< globCenterMat2 <<endl;
          //cout << "File saved" <<endl;
          alvar = false;
        }

      //}
      //if the user enters 2, shutdown the program
      //else if(save == 2){
            //alvar = false;
      //}
      //if the user enters anything else, continue
      //else{
        //save = 0;
        //raise(SIGINT); //exit process
      //}
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
  cloud_ptr = filterCloud(cloud_ptr, "x", globCenterMat(0)-.15, globCenterMat(0)+.15);
  cloud_ptr = filterCloud(cloud_ptr, "y", globCenterMat(1)-.10, globCenterMat(1)+.05);
  cloud_ptr = filterCloud(cloud_ptr, "z", globCenterMat(2)-0.15, globCenterMat(2)+0.15);

Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of .9 meters on the z axis.
  transform.translation() << -globCenterMat(0), -globCenterMat(1), -globCenterMat(2);
   // Executing the transformation
  pcl::transformPointCloud (*cloud_ptr, *cloud_ptr, transform);
  Eigen::Affine3f rotate(globQuat.toRotationMatrix().transpose());
  // The same rotation matrix as before; theta radians around Y axis
   // Executing the transformation
  pcl::transformPointCloud (*cloud_ptr, *cloud_ptr, rotate);

}


int main(int argc, char **argv) {


  std::cout << "Setting up connection to arduino" << endl; //********
    //open up connection to arduino
    fd = open_port();
    set_port(fd);
    
    usleep(1000000); //Wait for 1 sec
    int n = write(fd, "b", 1); //test that it works
    if(n < 0){
        std::cout <<" Connection to arduino failed" <<endl;
        close(fd);
        raise(SIGINT); //exit process
    }// *********************************************************
  
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
  viewer->addCoordinateSystem (1.0, 0 , 0, 0, "Point Cloud Viewer", 0);

  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());
  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb);
  // Spin
  ros::spinOnce ();
  viewer->spinOnce();
  double save=0;
  cout <<"Please enter any char to continue: ";
  cin >> save;
  
  //track how many point clouds have been saved
  static int cloudNumber = 0;

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
              //need to rotate NUM_ROTATIONS and save each
                if(cloudNumber < NUM_ROTATIONS){
                    //save point cloud
                    savePointCloudToFile(cloud_ptr, cloudNumber);
                    //rotate the turntable, check it was sucessful
                    rotate(fd);//*********************************************************
                    //increment number of point clouds captured
                    cloudNumber++; 
                }
                //we've finished!
                else if(cloudNumber >= NUM_ROTATIONS){
                    std::cout << "Finished capture" << endl;
                    close(fd); //*********************************************************
                    raise(SIGINT); //exit process
                }
                //else we haven't started capturing
          }  
    }
  return 0;

}

//send signal to arduino to rotate and sleep to make sure that it completes
int rotate(int fd){
        int n;
        //send signal to arduino to rotate the stepper motor
        n = write(fd, "r", 1);
        //if we failed, print an error message so the user knows
        if( n < 0 ){
            close(fd);
            std::cout << "ERROR: unable to write data to arduino" << endl;
            raise(SIGINT);
        }
        //give turntable time to actually rotate before taking picture
        boost::this_thread::sleep( boost::posix_time::milliseconds(4000) );
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