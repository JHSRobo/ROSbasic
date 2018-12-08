#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <cmath> // for sqrt() function
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


/**
* @breif returns a number mapped proportioanlly from one range of numbers to another
* @param[in] input Value to be mapped
* @param[in] inMax The maximum value for the range of the input
* @param[in] inMin The minimum value for the range of the input
* @param[in] outMin The minimum value for the range of the output
* @param[in] outMax The maximum value for the range of the output
* @return The input trnslated proportionally from range in to range out
*/
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}


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
    //PLEASE KEEP LR/FB lat/long naming convention consistant!!!
    vert_vel_pub = n.advertise<std_msgs::Float64>("rovpid/vertical/state", 1);
    lat_vel_pub = n.advertise<std_msgs::Float64>("rovpid/leftright/state", 1);
    long_vel_pub = n.advertise<std_msgs::Float64>("rovpid/frontback/state", 1);

    vertTime = ros::Time::now();
    horizTime = vertTime;

    //ROS subscriber to get vectors from the joystick control input
    horiz_sub = n.subscribe("rov/cmd_horizontal_vdrive", 1, horizCallback);
    vert_sub = n.subscribe("rov/cmd_vertical_vdrive", 1, vertCallback);

    ros::Rate rate(100); //100Hz update rate to prevent divide by 0 induced by floating point precision errors

    while(ros::ok()){
        ros::spinOnce();
        horizCalc();
        vertCalc();
        rate.sleep();
    }

    return 0;
}

//Updated regressed formula
double calcThrustForce(int dutyCycle){
  //regressing some points from the bluerobotics t100 graph
  //https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t100-thruster/
  return(0.0000000297138*std::pow(dutyCycle, 3)-0.00012908*std::pow(dutyCycle, 2)+0.19387*dutyCycle-100.496198);
}

void vertCallback(const vector_drive::thrusterPercents::ConstPtr &thrust)
{
  T1 = map(thrust->t1, -1000, 1000, 1100, 1900);
  T2 = map(thrust->t2, -1000, 1000, 1100, 1900);
  vertCalc();
}

void horizCallback(const vector_drive::thrusterPercents::ConstPtr &thrust)
{
    T1 = map(thrust->t1, -1000, 1000, 1100, 1900);
    T2 = map(thrust->t2, -1000, 1000, 1100, 1900);
    T3 = map(thrust->t3, -1000, 1000, 1100, 1900);
    T4 = map(thrust->t4, -1000, 1000, 1100, 1900);
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
    *   t3 forward is bottom-right (/)
    *     positive y, negative x
    *   t4 forward is bottom-left  (\)
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

//    yVel += horizTimestep * (findLegLength(calcThrustForce(T1)) + findLegLength(calcThrustForce(T2)) + findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)) - calcDragForce(yVel));
//    xVel += horizTimestep * (findLegLength(calcThrustForce(T1)) - findLegLength(calcThrustForce(T2)) - findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)) - calcDragForce(xVel));

    yVel += horizTimestep * (findLegLength(calcThrustForce(T1)) + findLegLength(calcThrustForce(T2)) + findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)));
    xVel += horizTimestep * (findLegLength(calcThrustForce(T1)) - findLegLength(calcThrustForce(T2)) - findLegLength(calcThrustForce(T3)) + findLegLength(calcThrustForce(T4)));


    xVel_msg.data = xVel;
    yVel_msg.data = yVel;

    long_vel_pub.publish(xVel_msg); //fb
    lat_vel_pub.publish(yVel_msg);  //lr
    std::cout << "yVel: " << yVel << "\n";
    std::cout << "horizTimestep: " << horizTimestep << "\n";
  }

double findLegLength(double hypotenuse)
{
    return(hypotenuse/sqrt(2)); //45 45 90 triangle sides = 1 1 sqrt(2)
}
