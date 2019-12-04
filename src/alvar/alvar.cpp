#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
// basic file operations
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
#define NUM_ROWS 18

//ofstream myfile;
int numPoints = 0;
double z_avg;
MatrixXf points(NUM_ROWS,4);



void cb(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
      
      double x, y, z;
      x=req.markers[0].pose.pose.position.x;
      y=req.markers[0].pose.pose.position.y;
      z=req.markers[0].pose.pose.position.z;
      ROS_INFO("x=%1.2f  y=%1.2f  z=%1.2f", x, y, z);
     
      double save=0;
      cout <<"Please enter: ";
      cin >> save;
      //if the user enters 1, save the coordinates, or calculate the center
      if(save == 1){
       // myfile << "{" <<x <<","<<y <<","<<z <<"}\n";
        //need to log at least NUM_ROWS to points
        if(numPoints < NUM_ROWS){
          points(numPoints,0) = x;
          points(numPoints,1) = y;
          points(numPoints,2) = z;
          points(numPoints,3) = 1.0;
          z_avg+=z;
          numPoints+=1;
          cout << "num coords: " << numPoints << endl;
        }
        //once we have enough points, use least square to fit circle
        //and fine the center
        else{
          JacobiSVD<MatrixXf> svd(points, ComputeThinV);
          MatrixXf vMat = svd.matrixV();
          cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << vMat << endl;
          /* 
          myfile.open ("axis_coordinates.txt");
          myfile << 0 <<"\n";// TODO, save x, y, z
          myfile.close();
          cout << "File saved" <<endl;
          */

        }
        
        
      }
      //if the user enters 2, shutdown the program
      else if(save == 2){
        //myfile <<"}";
         //myfile.close();
        ros::shutdown();
      }
      //if the user enters anything else, continue
      else{
        save = 0;
      }
    } // if
}

int main(int argc, char **argv) {

  //myfile.open ("axis_coordinates.txt");
  //myfile << "{\n";
  ros::init(argc, argv, "arlistener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  cout <<"Enter: \n 1: log coordinates\n 2:  shutdown\n Anthing else to continue" <<endl;
  ros::spin();
  return 0;

}