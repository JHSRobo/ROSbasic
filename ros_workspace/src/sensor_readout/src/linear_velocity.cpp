#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

double x_vel = 0;
double y_vel = 0;
double z_vel = 0;
ros::Time prevTime;
geometry_msgs::Vector3 velocity;
ros::Publisher pub;

/**
  *  pi sense hat x, y, and z axes are on this website (and probably others):
  *  https://www.mathworks.com/help/supportpkg/raspberrypiio/examples/working-with-raspberry-pi-sense-hat.html
  *
  *
  *  In the ROV, the pi is oriented so that
  *
  *  its +x is the ROV's left which /rov/cmd_vel considers -x
  *  its +y is the ROV's bottom which /rov/cmd_vel considers +z
  *  is +z is the ROV's backside which /rov/cmd_vel considers -y
  */

  void imuCallback(const sensor_msgs::Imu::ConstPtr& accel)
  {
    ros::Time nowTime = ros::Time::now();
    double interval = (nowTime - prevTime).toSec();
    x_vel += -1 * accel->linear_acceleration.x * interval;
    y_vel += accel->linear_acceleration.z * interval;
    z_vel += -1 * accel->linear_acceleration.y * interval;
    prevTime = ros::Time::now();
    velocity.x = x_vel;
    velocity.y = y_vel;
    velocity.z = z_vel;
    pub.publish(velocity);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_velocity");

  ros::NodeHandle n;

  prevTime = ros::Time::now();

  pub = n.advertise<geometry_msgs::Vector3>("rov/linear_velocity", 1);
  ros::Subscriber sub = n.subscribe("/rov/imu", 3, imuCallback);

  ros::spin();
}
