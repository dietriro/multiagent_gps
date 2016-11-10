#include "RobotManagerHandler.h"

RobotManagerHandler::RobotManagerHandler(ros::NodeHandle* n):
  n_(n)
{

}

bool RobotManagerHandler::initializeRobots(gps_robot_manager::InitializeRobotManagers::Request  &req,
                                           gps_robot_manager::InitializeRobotManagers::Response &res)
{
  RobotManager tempRM;

  for (int i=0;i<req.num_start_pos;i++)
  {
    for (int k=0;k<req.num_iters;k++)
    {
      tempRM
    }
  }
}