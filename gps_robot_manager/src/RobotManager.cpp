#include "RobotManager.h"

RobotManager::RobotManager(ros::NodeHandle* n):
  n_(n)
{
  // ROS Services
  service_init_ = n_->advertiseService("initialize_robots", &RobotManager::initializeRobots, this);

  // ROS Publishers


  // ROS Subscribers


  // Misc

    int n_robots = 5;

  robots.reserve(n_robots);

  string robot_description_path = "/home/osu/ros_ws/src/stdr_simulator/stdr_resources/resources/robots/simple_robot.xml";

  vector<geometry_msgs::Pose2D> init_poses;
  init_poses.reserve(n_robots);

  geometry_msgs::Pose2D pose;
  pose.x = 1;
  pose.y = 1;
  pose.theta = 0;

  // Initialize all robots
  for (int i=0;i<n_robots;i++)
  {
    pose.x += 0;
    init_poses.push_back(pose); 
    Robot* newRobot = new Robot("robot"+std::to_string(i), init_poses.at(i), robot_description_path, n_);
    robots.push_back(newRobot);
  }

  // Load map for environment

  // Populate environment with robots

  ROS_INFO_STREAM("Initialization of " << n_robots << " successfully completed.");

}

RobotManager::~RobotManager()
{
  
}

void RobotManager::update()
{
  
}

bool RobotManager::initializeRobots(gps_robot_manager::Initialize::Request  &req,
                                    gps_robot_manager::Initialize::Response &res)
{
  int n_robots = 5;

  robots.reserve(n_robots);

  string robot_description_path = "/home/osu/ros_ws/src/stdr_simulator/stdr_resources/resources/robots/simple_robot.xml";

  vector<geometry_msgs::Pose2D> init_poses;
  init_poses.reserve(n_robots);

  geometry_msgs::Pose2D pose;
  pose.x = 1;
  pose.y = 1;
  pose.theta = 0;

  // Initialize all robots
  for (int i=0;i<n_robots;i++)
  {
    pose.x += i;
    init_poses.at(i) = pose; 
    Robot* newRobot = new Robot("robot"+std::to_string(i), init_poses.at(i), robot_description_path, n_);
    robots_.push_back(newRobot);
  }

  // Load map for environment


  ROS_INFO_STREAM("Initialization of " << n_robots << " successfully completed.");

  return true;
}

void RobotManager::simulate_robot_actions()
{
  vector< vector<int> > sensor_sonar, sensor_targets;

  sensor_sonar.reserve(robots_.size);
  sensor_targets.reserve(robots_.size);

  for (int r=0;r<robots_.size();r++) 
  {
    sensor_sonar.at(r).reserve(trajectory_length*8);
    sensor_targets.at(r).reserve(trajectory_length*8);
  }
  
  for (int t=0;t<trajectory_length_;t++)
  {
    for (int r=0;r<robots_.size();r++) 
    {
      robots_.at(r)->move();
      robots_.at(r)->get_sensor_data(&sensor_sonar, &sensor_targets);
    }
  }
}

// Subscriber callbacks

void RobotManager::callback_robot_info()
{
  // Set robot information


  // Start robot simulation
  if ()
}




















