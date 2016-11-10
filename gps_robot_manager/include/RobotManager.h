#ifndef ROBOT_MANAGER
#define ROBOT_MANAGER

#include "gps_robot_manager/Initialize.h"
#include "Robot.h"

#include "ros/ros.h"

#include <vector>
#include <string>

using namespace std;

class RobotManager
{
public:
  RobotManager(ros::NodeHandle* n);
  ~RobotManager();

  void update();

  // Services
  bool initializeRobots(gps_robot_manager::Initialize::Request  &req,
                         gps_robot_manager::Initialize::Response &res);
private:
  
  ros::NodeHandle* n_;                            // Creade node handle

  // ROS Services
  ros::ServiceServer service_init_;

  // ROS Publishers

  
  // ROS Subscribers
  ros::Subscriber sub_robot_init;

  vector<Robot*> robots;
  int trajectory_length_;

};

#endif