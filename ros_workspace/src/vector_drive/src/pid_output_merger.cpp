#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

ros::Publisher pub;
geometry_msgs::Twist effort;

//callback defs
void longCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.linear.y = msg->data;
  pub.publish(effort);
}

void latCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.linear.x = msg->data;
  pub.publish(effort);
}

void vertCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.linear.z = msg->data;
  pub.publish(effort);
}

void pitchCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.angular.y = msg->data;
  pub.publish(effort);
}

void rollCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.angular.z = msg->data;
  pub.publish(effort);
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg)
{
  effort.angular.x = msg->data;
  pub.publish(effort);
}


//main loop
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_output_merger");

  ros::NodeHandle n;

  pub = n.advertise<geometry_msgs::Twist>("/rov/control_effort", 12);

  ros::Subscriber longSub = n.subscribe("/long_pid/control_effort", 3, longCallback);
  ros::Subscriber latSub = n.subscribe("/lat_pid/control_effort", 3, latCallback);
  ros::Subscriber vertSub = n.subscribe("/vert_pid/control_effort", 3, vertCallback);
  ros::Subscriber pitchSub = n.subscribe("/pitch_pid/control_effort", 3, pitchCallback);
  ros::Subscriber rollSub = n.subscribe("/roll_pid/control_effort", 3, rollCallback);
  ros::Subscriber yawSub = n.subscribe("/long_pid/control_effort", 3, yawCallback);

  ros::spin();

  return(0);
}
