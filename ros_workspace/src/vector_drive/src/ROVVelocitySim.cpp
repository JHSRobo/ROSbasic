#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

//custom message for holding 4 int32 thruster percents
#include "vector_drive/thrusterPercents.h"

//physical constants for some basic fluid dynamics
const double maxForwardThrust = 2.36; //max thrust of T100 (N); T200 is 5.1
const double maxReverseThrust = 1.85; //max thrust of T100 (N); T200 is 4.1
const double fluidDensity = 997; //density of water (kg/m^3)
const double refArea = 0.3251606; //ROV's reference area i.e. area facing direction of movement (m^2)
const double cD = 2.5; //drag coefficient for calculating drag force (best guesstimate)

double xVel = 0;
double yVel = 0;
double zVel = 0;

ros::Time vertTime;
ros::Time horizTime;
double timestep;

/*since at 0.1 m/s, our reynolds number is a million, we can consider the guesstimated cd a constant
For sharp-cornered bluff bodies, like square cylinders and plates held transverse
to the flow direction, this equation is applicable with the drag coefficient as a
constant value when the Reynolds number is greater than 1000. -Wikipedia*/

double calcDragForce(double velocity)
{
    return (0.5 * fluidDensity * velocity * velocity * cD * refArea);
}

double calcThrustForce(int thrustPercentage);


ros::publisher vert_vel_pub; //publishes calculated vertical velocity vector
ros::publisher lat_vel_pub; //publishes calculated lateral velocity vector
ros::publisher long_vel_pub; //publishes calculated longitudinal velocity vector

ros::subscriber horiz_sub;
ros::subscriber vert_sub;

void horizCallback(const vector_drive::thrusterpercents::ConstPtr &thrust);
void vertCallback(const vector_drive::thrusterpercents::ConstPtr &thrust);

int main(int argc, char **argv)
{

    //initialize node for rov sim
    ros::init(argc, argv, "rov_vel_sim");

    ros::NodeHandle n;

    //ROS velocity publishers for pids
    vert_vel_pub = n.advertise<std_msgs::Float64>("rovpid/vertical/state", 1);
    lat_vel_pub = n.advertise<std_msgs::Float64>("rovpid/leftright/state", 1);
    long_vel_pub = n.advertise<std_msgs::Float64>("rovpid/frontback/state", 1);

    vertTime = ros::Time::now();
    horizTime = vertTime;

    //ROS subscriber to get vectors from the joystick control input
    horiz_sub = n.subscribe("rov/cmd_horizontal_vdrive", 1, horizCallback);
    vert_sub = n.subscribe("rov/cmd_vertical_vdrive", 1, vertCallback);

    ros::spin();

    return 0;
}

double calcThrustForce(int thrustPercentage)
{
    //thrust percentages go from -1000 to 1000
    //pwm widths go from 1100 to 1900
    thrustPercentage = thrustPercentage * 0.4 + 1500; //convert thrust percent to pwm width so we can use the neat graphs bluerobotics has
    if (thrustPercentage <=1550 && thrustPercentage >= 1450) {return 0;} //t100 has a small deadzone
    //regressing some points from the bluerobotics t100 graph
    if (thrustPercentage < 1450) {return thrustPercentage * 0.00526491178571 - 7.462449664287;}  //r2 of 0.9865113971
    if (thrustPercentage > 1550) {return thrustPercentage * 0.00750371427428 - 11.760354953715;} //r2 of 0.996488047
}

void vertCallback(const vector_drive::thrusterpercents::ConstPtr &thrust)
{
    timestep = (ros::Time::now() - vertTime).toSec();
    vertTime = ros::Time::now();
    zVel += (2 * calcThrustForce(thrust->t1) + calcDragForce(zVel)) * timestep;
}
