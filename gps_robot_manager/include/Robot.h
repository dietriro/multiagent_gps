#ifndef ROBOT
#define ROBOT

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nodelet/NodeletLoad.h"
#include "ros/ros.h"

#include <string>
#include <sstream>
#include <math.h>

static const int NUM_SENSORS = 8;


using namespace std;

class Robot
{
public:
  Robot(const string &name, ros::NodeHandle* n);
  Robot(const string &name, const geometry_msgs::Pose2D &initPose, const string &yaml_description_path, ros::NodeHandle* n);
  ~Robot();

  void resetRobotPosition();
  void setCmdVelocity(const geometry_msgs::Twist &cmd_vel);
  void pubScanData();


private:
  void spawn();

  string name_;

  geometry_msgs::Pose2D init_pose_;
  geometry_msgs::Pose2D pose_;
  
  string yaml_description_path_;
  

  // Trajectory
  

  // Scanned Data
  vector<int> sensor_sonar_;
  vector<int> sensor_targets_;

  ros::NodeHandle* n_; 
  ros::Publisher pub_cmd_vel_;

  // Properties
  float sensor_threshold_;

};

#endif