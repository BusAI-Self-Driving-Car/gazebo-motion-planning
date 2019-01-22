#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

geometry_msgs::PoseStamped g_mobot_pose; //this is the pose of the robot in the world, according to Gazebo
geometry_msgs::Quaternion g_quat;
ros::Publisher g_pose_publisher;
ros::Publisher g_odom_publisher;
ros::Publisher g_gps_publisher;
// std::normal_distribution<double> distribution(0.0, 1.0); //args: mean, std_dev
// std::default_random_engine generator;
std::string target_model;

void model_state_CB(const gazebo_msgs::ModelStates& model_states)
{
  int n_models = model_states.name.size();
  int imodel;
  //ROS_INFO("there are %d models in the transmission",n_models);
  bool found_name=false;
  for(imodel = 0; imodel < n_models; imodel++) {
    std::string model_name(model_states.name[imodel]);
    if (model_name.compare(target_model) == 0) {
      //ROS_INFO("found match: mobot is model %d",imodel);
      found_name = true;
      break;
    }
  }
  if(found_name) {
    g_mobot_pose.header.frame_id = target_model + "/odom";
    g_mobot_pose.pose = model_states.pose[imodel];
    g_pose_publisher.publish(g_mobot_pose);
  } else {
    // ROS_WARN("state of model not found");
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_model_state_publisher");
    ros::NodeHandle nh;

    ros::param::get("~target_model", target_model);

    g_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(target_model+"/current_pose", 1);

    ros::Subscriber state_sub = nh.subscribe("gazebo/model_states", 1, model_state_CB);

    //suppress the orientation output for noisy state; fill out a legal, constant quaternion
    g_quat.x = 0;
    g_quat.y = 0;
    g_quat.z = 0;
    g_quat.w = 1;

    ros::spin();
}
