/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>
namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  linear_speed_(0.0),
  angular_speed_(0.0),
  angular_vel_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

      geometry_msgs::Twist zero_cmd_vel;
      zero_cmd_vel.linear.x = 0;
      zero_cmd_vel.angular.z = 0;
    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      linear_vel_=(double)0;
      angular_vel_=(double)0;
      cmd_vel_pub_.publish(zero_cmd_vel);

      ROS_WARN_THROTTLE(1.0, "Timeout\r");
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
     if (is_safe==1)
     {
     geometry_msgs::Twist pub;
     pub.linear.x=linear_vel_;
     pub.angular.z=angular_vel_; 
     cmd_vel_pub_.publish(pub);
     }
     else
     {   
     
      linear_vel_=(double)0;
      angular_vel_=(double)0;
      cmd_vel_pub_.publish(zero_cmd_vel); 
     }
     }
    ROS_INFO_THROTTLE(1.0,"Linear Vel:%f Angular Vel:%f\r",(double)linear_vel_,(double)angular_vel_);
    r.sleep();
  }
}

void SafeTeleop::moveForward()
{

   linear_vel_=(double)linear_speed_;
   angular_vel_=(double)0;
   last_command_timestamp_ = ros::Time::now().toSec();
}
void SafeTeleop::moveBackward()
{
  linear_vel_=-(double)linear_speed_;
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_=(double)0;
}

void SafeTeleop::rotateClockwise()
{
 
  angular_vel_=-(double)angular_speed_;
  linear_vel_=(double)0;
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateCounterClockwise()
{  
  angular_vel_=(double)angular_speed_;
  linear_vel_=(double)0;
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::stop()
{
  linear_vel_=(double)0;
  angular_vel_=(double)0;
  last_command_timestamp_ = ros::Time::now().toSec();
}


void SafeTeleop::increaseLinearSpeed()
{
  if (linear_speed_ + linear_vel_increment_> max_linear_vel_+0.0000001)
 {
  ROS_WARN("Maximum Linear velocity achieved\r");
  linear_speed_=(double)linear_speed_;
 }
 else
 {
  linear_speed_ = linear_speed_+ linear_vel_increment_;
 } 
  displayCurrentSpeeds();
  
  last_command_timestamp_ = ros::Time::now().toSec();
}    

void SafeTeleop::decreaseLinearSpeed()
{
  if (linear_speed_ - linear_vel_increment_<0)
 {
 ROS_WARN("Linear Velocity already 0\r");
 linear_speed_=(double)0;
 }
 else
 {
 linear_speed_ = linear_speed_- linear_vel_increment_;
 }
  displayCurrentSpeeds();
  
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::increaseAngularSpeed()
{
  if (angular_speed_ + angular_vel_increment_>max_angular_vel_+0.00000001)
{
  ROS_WARN("Maximum Angular velocity achieved\r");
 }
 else
 {
  angular_speed_ = angular_speed_+ angular_vel_increment_;
 } 
  displayCurrentSpeeds();
  
  last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::decreaseAngularSpeed()
{
  
  if (angular_speed_ - angular_vel_increment_<0)
 {
  ROS_WARN("Angular Velocity already 0\r");
 angular_speed_=(double)0;
 }
 else
 {
  angular_speed_ = angular_speed_-angular_vel_increment_;
 } 
  displayCurrentSpeeds();
  
  last_command_timestamp_ = ros::Time::now().toSec();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  auto laser_scan = getLaserScan();
  ROS_WARN_THROTTLE(1.0,"anglemin,max,inc%f,%f,%f\r",laser_scan.angle_min,laser_scan.angle_max,laser_scan.angle_increment);
  int len_scan;
  double min,max,start,end;
  int flag=1;  
  //int num_neg=laser_scan.angle_min/laser_scan.angle_increment;
  //int num_pos=laser_scan.angle_max/laser_scan.angle_increment;
  //len_scan=(laser_scan.angle_max-laser_scan.angle_min)/laser_scan.angle_increment;
  len_scan=laser_scan.ranges.size();
double rad=laser_scan.angle_min;
std::vector<int> arr;
ros::Rate loop_rate(1);
rad=SafeTeleop::normalizeTo360Angle(rad);
  if(linear_vel_>0)
  {
   //min=-0.2618;
   //max=0.2618;
   min=6.0213;
   max=0.2618;
   for(int i=0;i<len_scan;i++)  
   {
   if(laser_scan.ranges[i]==0.0)
    continue;
   //loop_rate.sleep();
  if(rad>=min || rad<=max)
  {
   if(laser_scan.ranges[i]<=min_safety_distance_)
   {
    ROS_WARN_THROTTLE(1.0,"Object detected in range 0.5");
    return 0;
   }
}
   rad=rad+laser_scan.angle_increment;
}
   }
  else if(linear_vel_<0)
  {
  // min=-2.88;
   //max=2.88;
  min=2.8800;
  max=3.4034;
  
for(int i=0;i<len_scan;i++)  
{

//loop_rate.sleep();
//int r;
/*if(rad>=0)
 r=rad;
else
 r=-rad;
*/
if(laser_scan.ranges[i]==0)
continue;
if(rad>=min && rad<=max)
{
if(laser_scan.ranges[i]<=min_safety_distance_)
   {
  ROS_WARN_THROTTLE(1.0,"Object detected in range 0.5");
  return 0;
   }
}
rad=rad+laser_scan.angle_increment;
} 
}
  return 1;
}

} // namespace safe_teleop_node


