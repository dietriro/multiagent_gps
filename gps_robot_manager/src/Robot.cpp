#include "Robot.h"


Robot::Robot(const string &name, ros::NodeHandle* n):
  name_(name),
  n_(n)
{
  pose_.x = pose.y = pose.theta = 0;

  ROS_INFO_STREAM("Created '" << this->name_ << "' at position " << pose_ << " successfully.");
}

Robot::Robot(const string &name, const geometry_msgs::Pose2D &init_pose, const string &yaml_description_path, ros::NodeHandle* n):
  name_(name),
  init_pose_(init_pose), 
  pose_(init_pose),
  yaml_description_path_(yaml_description_path),
  n_(n)
{
  // ROS Publisher
  pub_cmd_vel_ = n_->advertise<geometry_msgs::Twist>("/"+name_+"/cmd_vel", 1000);

  // stdr robot
  this->spawn();

  ROS_INFO_STREAM("Created '" << this->name_ << "' successfully.");
}

Robot::~Robot()
{

}

void Robot::resetRobotPosition()
{
  stringstream temp_cmd;
  temp_cmd << "rosrun stdr_robot robot_handler replace /" << name_ << " " << init_pose_.x << " " << init_pose_.y << " " << init_pose_.theta;
  system(temp_cmd.str().c_str());
}

void Robot::setCmdVelocity(const geometry_msgs::Twist &cmd_vel)
{

  pub_cmd_vel_.publish(cmd_vel);
}

void Robot::pubScanData()
{

}

void Robot::spawn()
{
  stringstream temp_cmd;
  temp_cmd << "rosrun stdr_robot robot_handler add " << yaml_description_path_ << " " << init_pose_.x << " " << init_pose_.y << " " << init_pose_.theta;
  system(temp_cmd.str().c_str());
}

void Robot::getNearbyObjects(vector<int> *num_hits, vector<geometry_msgs::Pose2D> obj)
{
  float alpha = 0;              // Angle between y-axis and vector to object
  float dist = 0;               // Distance between robot and object
  float dist_x = 0;             // The Distance in x-direction
  float dist_y = 0;             // The Distance in y-direction
  int sensor_index = 0;         // Index of the sensor that spotted object i

  for (int i=0;i<NUM_SENSORS;i++)
    num_hits->push_back(0);

  for (int obj_i=0;obj_i<objects.size();obj_i++)
  {
    dist_x = obj.at(obj_i).x - pose_.x;
    dist_y = obj.at(obj_i).y - pose_.y;
    dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    if (dist > sensor_range_)
      continue;

    alpha = acos(dist_y/dist);

    if (dist_y > 0) && (dist_x < 0)
      alpha = 2*M_PI - alpha;
    else if (dist_y > 0) && (dist_x > 0)
      alpha = M_PI + alpha;
    else if (dist_y > 0) && (dist_x > 0)
      alpha = M_PI - alpha;
    
    sensor_index = (alpha - pose_.theta) / (M_PI/4);
    if (sensor_index < 0)
      sensor_index = NUM_SENSORS + sensor_index;

    num_hits->at(sensor_index) += 1;
  }
}

void Robot::move()
{

}

void Robot::get_sensor_data()
{
  
}






