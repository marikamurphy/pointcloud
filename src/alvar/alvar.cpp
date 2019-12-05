#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
// basic file operations
#include <iostream>
#include <fstream>
#include <string>

#include <csignal>

#include <Eigen/Dense>
#include <Eigen/Geometry> //? do we need ?

using namespace std;
using namespace Eigen;
#define NUM_ROWS 6

ofstream myfile;
int numPoints = 0;
double z_avg; //??????????????????
MatrixXf points(NUM_ROWS,3);
string filename = "center_rotMat.txt";

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
  Vector3f oldCoord(0.0,0.0,1.0);
  //quaternion describes transformation b/w 2 vectors
  Quaternion<float> quad = Quaternion<float>::FromTwoVectors(newCoord,oldCoord);
  quad.normalize();
  //we transform the quaterion into a rotation matrix so we can easily multiply it by the points
  Matrix3f rotMat = quad.toRotationMatrix();
  cout << "rotMat" <<endl << rotMat<<endl;
  return rotMat;

}

/*Solution by method of least squares:
  A*c = b, c' = argmin(||A*c - b||^2)
  A = [x y 1], b = [x^2+y^2] */
Vector3f calc_center(Matrix3f rotMat){
  //we transpose all the points onto the plane
  MatrixXf rotatedPoints =  rotMat * points.transpose();
  cout <<"rotated points" <<endl<<rotatedPoints <<endl;
  float zCenter = rotatedPoints(2,0); //save this for later
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
  //cout <<"soln"<<endl<< center  <<endl;
  float xCenter = b_soln(0)/2;
  float yCenter = b_soln(1)/2;
  //cout << "X: " << xCenter << endl <<"Y: "<<yCenter<<endl <<"Z"<<zCenter<<endl;
  Vector3f centerMat(xCenter,yCenter,zCenter);
  cout <<"centerMat on plane: "<<centerMat<<endl<<"in our coords: " <<rotMat.transpose() * centerMat<<endl;
  return centerMat;
}

void saveMat(MatrixXf mat){
  myfile.open (filename);
  for(int r = 0; r < mat.rows(); r++){
    for(int c = 0; c < mat.cols(); c++){
      myfile << mat(r,c) << "\n";
    }
  }
  myfile.close();
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
      ROS_INFO("x=%1.2f  y=%1.2f  z=%1.2f", x, y, z);
     
      double save=0;
      cout <<"Please enter: ";
      cin >> save;

      //if the user enters 1, save the coordinates, or calculate the center
      if(save == 1){
        //need to log at least NUM_ROWS to points
        if(numPoints < NUM_ROWS){
          points(numPoints,0) = x;
          points(numPoints,1) = y;
          points(numPoints,2) = z;
         
          z_avg+=z; //????????????????
          numPoints+=1;
          cout << "num coords: " << numPoints << endl;
        }
        //once we have enough points, use least square to fit circle
        //and fine the center
        else{
          cout <<"points"<<endl<<points<<endl;
          Matrix3f rotMat = calc_rot_Mat();
          Vector3f centerMat = calc_center(rotMat);
          saveMat(rotMat);
          saveVec(centerMat);
          cout << "File saved" <<endl;
        } 
      }
      //if the user enters 2, shutdown the program
      else if(save == 2){
            ros::shutdown();
      }
      //if the user enters anything else, continue
      else{
        save = 0;
        raise(SIGINT); //exit process
      }
    }// end if statement
}

int main(int argc, char **argv) {

  
  ros::init(argc, argv, "arlistener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  cout <<"Enter: \n 1: log coordinates\n 2:  shutdown\n Anthing else to continue" <<endl;
  ros::spin();
  return 0;

}