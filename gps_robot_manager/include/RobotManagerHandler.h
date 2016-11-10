#ifndef ROBOT_MANAGER
#define ROBOT_MANAGER

class RobotManagerHandler
{
public:
  RobotManagerHandler(ros::NodeHandle* n);

  bool initializeRobots(gps_robot_manager::InitializeRobotManagers::Request  &req,
                        gps_robot_manager::InitializeRobotManagers::Response &res);


private:
  ros::NodeHandle* n_;
  std::vector<RobotManager*> robot_managers_;

};

#endif