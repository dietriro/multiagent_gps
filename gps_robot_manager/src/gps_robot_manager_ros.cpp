#include "RobotManagerHandler.h"
#include "ros/ros.h"



// msg for robot: TrialParams.msg




int main(int argc, char **argv)
{

  // Initialize ROS
  ros::init(argc, argv, "gps_robot_manager");   // Initialize ROS
  ros::NodeHandle n;
  ros::Rate loop_rate(10);                      // Specify loop rate of node 

  // Create RobotManager
  RobotManagerHandler rmh(&n);

  while (ros::ok())
  {
    ros::spinOnce();            // Spin ROS and get all new messages

    rmh.update();

    loop_rate.sleep();          // Sleep for the loop_rate time
  }

  return 0;
}