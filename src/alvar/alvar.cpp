
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include <Eigen/Eigen>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::Vector3d;

using Eigen::Quaterniond;
using Eigen::AngleAxisd;

class AlvarMarker {
public:
  AlvarMarker(ros::NodeHandle &n, tf::TransformListener &tf_l
    , std::string fromFrame);

  void vis_cb(const visualization_msgs::Marker::ConstPtr& msg);

protected:
  tf::TransformListener &tfL;
  ros::Subscriber vis_sub;
  std::string _fromFrame;
};

AlvarMarker::AlvarMarker(ros::NodeHandle &n, tf::TransformListener &tf_l
    , std::string fromFrame) : tfL(tf_l), _fromFrame(fromFrame) {
  //subscriber to get tag information
  vis_sub = n.subscribe("/visualization_marker", 1, &AlvarMarker::vis_cb, this);
}

void AlvarMarker::vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
  std::cout<<"alvar"<<std::endl;
  geometry_msgs::PoseStamped tag_pose;
  geometry_msgs::PoseStamped tag_rel_pose;

  try{
    tag_pose.header = msg->header;
    tag_pose.pose = msg->pose;
    tfL.waitForTransform(_fromFrame, tag_pose.header.frame_id, ros::Time(0), ros::Duration(4));
    

    tfL.transformPose(_fromFrame, tag_pose, tag_rel_pose);

    //TODO: what to do with tag_rel_pose.pose?
 

  } catch (tf::TransformException ex) {}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alvar");
  ros::NodeHandle n;
  
  tf::TransformListener tf_l;

  AlvarMarker am(n, tf_l, "camera_rgb_optical_frame");  //Kinect on computer

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

//TODO: figure out how to save the pose of the alvar marker and pass along to pcd_write
  return 0;
}

