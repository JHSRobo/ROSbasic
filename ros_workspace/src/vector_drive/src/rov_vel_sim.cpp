#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
//custom message for holding 4 int32 thruster percents
#include "vector_drive/thrusterPercents.h"

//physical constants for some basic fluid dynamics
const double maxForwardThrust = 2.36; //max thrust of T100 (N); T200 is 5.1
const double maxReverseThrust = 1.85; //max thrust of T100 (N); T200 is 4.1
const double fluidDensity = 997; //density of water (kg/m^3)
const double refArea = 0.3251606; //ROV's reference area i.e. area facing direction of movement (m^2)
const double cD = 2.5; //drag coefficient for calculating drag force (best guesstimate)
const double rovMass = 16; //kg

/*since at 0.1 m/s, our reynolds number is a million, we can consider the guesstimated cd a constant
For sharp-cornered bluff bodies, like square cylinders and plates held transverse
to the flow direction, this equation is applicable with the drag coefficient as a
constant value when the Reynolds number is greater than 1000. -Wikipedia*/

double xVel = 0;
double yVel = 0;
double zVel = 0;

ros::Time vertTime;
ros::Time horizTime;
double horizTimestep;
double vertTimestep;

double calcDragForce(double velocity)
{
    if(velocity == 0){return 0;}
    return (0.5 * fluidDensity * velocity * velocity * cD * refArea) * velocity / abs(velocity);
}

double calcThrustForce(int thrustPercentage);
double findLegLength(double hypotenuse);

ros::Publisher vert_vel_pub; //publishes calculated vertical velocity vector
ros::Publisher lat_vel_pub; //publishes calculated lateral velocity vector
ros::Publisher long_vel_pub; //publishes calculated longitudinal velocity vector

ros::Subscriber horiz_sub;
ros::Subscriber vert_sub;

void horizCalc();
void vertCalc();

void horizCallback(const vector_drive::thrusterPercents::ConstPtr &thrust);
void vertCallback(const vector_drive::thrusterPercents::ConstPtr &thrust);

int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;
int T5 = 0;
int T6 = 0;

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

    while(1){
        ros::spinOnce();
        horizCalc();
        vertCalc();
    }

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

void vertCallback(const vector_drive::thrusterPercents::ConstPtr &thrust)
{
    T5 = thrust->t1;
    T6 = T5;
    vertCalc();
}

void horizCallback(const vector_drive::thrusterPercents::ConstPtr &thrust)
{
    T1 = thrust->t1;
    T2 = thrust->t2;
    T3 = thrust->t3;
    T4 = thrust->t4;
    horizCalc();
}




void vertCalc()
{
    std_msgs::Float64 zVel_msg;
    vertTimestep = (ros::Time::now() - vertTime).toSec();
    vertTime = ros::Time::now();
    zVel += (2 * calcThrustForce(T5) - calcDragForce(zVel)) * vertTimestep;
    zVel_msg.data = zVel;
    vert_vel_pub.publish(zVel_msg);
}

void horizCalc()
{
    std_msgs::Float64 xVel_msg;
    std_msgs::Float64 yVel_msg;
    horizTimestep = (ros::Time::now() - horizTime).toSec();
    horizTime = ros::Time::now();

    /*
    * assuming all thruster are at 90 degree angles from each other and 45 from the linear axes of the rov
    * (thrusters form an x-shape with rov in the middle)
    *   \ /
    *   ROV
    *   / \
    * here's the thruster positions we're working with (I think):
    *
    *       front
    *       t3 t4
    *  left t1 t2 right
    *       back
    *
    * forward for a thruster is assumed to be a positive thrusterPercent and a >1500 pwm pulse width (bluerobotics t100 spec)
    *   t3 forward is upper-right (/)
    *     positive y, negative x
    *   t4 forward is upper-left  (\)
    *     positive y, positive x
    *   t1 forward is upper-left  (\)
    *     positive y, positive x
    *   t2 forward is upper-right (/)
    *     positive y, negative x
    *
    * decomposing/change of basis of a thruster thrust vector is finding the leg length
    * of a 45-45-90 triangle, which is just the sqrt of the hypotenuse divided by 2
    * we have to multiply the leg length by either -1 or 1 depending on the thruster and thruster percentage
    * to figure out what to multiply by, divide thruster percent by abs(thruster percent) to get + or -1
    * then check the forward direction of the thruster and multiply based off that
    *
    * in /rov/cmd_vel (and therefore, the pid setpoints), forward is positive and <<LEFT>> is positive,
    * so left has to mean a positive velocity and forward has to be a positive velocity
    * //TODO find out if up is positive or not (we really gotta find the throttle)
    * at this point, we've just gone off the assumption that up is positive
    *
    * also, y is longitudinal, and x is lateral
    */

    xVel += horizTimestep * (findLegLength(calcThrustForce(T1)) - findLegLength(calcThrustForce(T2)) - findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)) - calcDragForce(xVel));
    yVel += horizTimestep * (findLegLength(calcThrustForce(T1)) + findLegLength(calcThrustForce(T2)) + findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)) - calcDragForce(yVel));

    xVel_msg.data = xVel;
    yVel_msg.data = yVel;

    lat_vel_pub.publish(xVel_msg);
    long_vel_pub.publish(yVel_msg);
    std::cout << "yVel: " << yVel << "\n";
    std::cout << "horizTimestep: " << horizTimestep << "\n";
  }

double findLegLength(double hypotenuse)
{
    if (hypotenuse == 0) {return 0;}
    return sqrt(abs(hypotenuse)) * hypotenuse / abs(hypotenuse);
}
