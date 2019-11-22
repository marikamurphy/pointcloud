
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

void cb(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
      
      double x, y, z;
      x=req.markers[0].pose.pose.position.x;
      y=req.markers[0].pose.pose.position.y;
      z=req.markers[0].pose.pose.position.z;
      ROS_INFO("x=%1.2f  y=%1.2f  z=%1.2f", x, y, z);
     
      int save=0;
      cout <<"Please enter 1 if you would like to save these coordinates: ";
      cin >> save;
      if(save == 1){
        //open file and save coordinates to it
        ofstream myfile;
        myfile.open ("axis_coordinates.txt");
        myfile << x <<"\n";
        myfile << y <<"\n";
        myfile << z <<"\n";
        myfile.close();
        cout << "File saved" <<endl;
        ros::shutdown();
      }
      else{
        save = 0;
      }
    } // if
}

int main(int argc, char **argv) {

  
  ros::init(argc, argv, "arlistener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  ros::spin();
  return 0;

}