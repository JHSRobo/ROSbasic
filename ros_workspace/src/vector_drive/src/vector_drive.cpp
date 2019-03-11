/**
* @author Michael Equi
* @version 0.1
* @date 8-11-2018
* @mainpage The drive_control node
* @section intro_sec Introduction
* This code contains implementations for converting horizontal and vertical control vectors into individual thruster percents (multiplied by 10 for more accuracy without needing to be stored as doubles) from -1000 to 1000
* @section compile_sec Compilation
* Compile using catkin_make in the ros_workspace directory.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//custom message for holding 4 int32 thruster percents
#include "vector_drive/thrusterPercents.h"
//custom message for holding 8 float64 thruster power usage in watts
#include "vector_drive/ThrusterPower.h"

//custom message from DRQ1250s
#include "drq1250/DRQ1250.h"


ros::Publisher horiz_pub;  //!< Publishes horizontal thrusterPercent (-1000 to 1000) message for thruster 1, 2, 3, and 4
ros::Publisher vert_pub;  //!< Publishes vertical thrusterPercent (-1000 to 1000) message for thruster 5, 6, 7, and 8
ros::Publisher power_pub;  //!< Publishes power (W) message for thrusters 1, 2, 3, 4, 5, 6, 7, and 8
ros::Subscriber sub; //!< Subscribes to rov/cmd_vel in order to get command/control vectors for vector drive algorithm
ros::Subscriber drq1_sub; //!< Subscribes to rov/drq1250_1/status in order to get P,V, and I information for active overload protection
ros::Subscriber drq2_sub; //!< Subscribes to rov/drq1250_2/status in order to get P,V, and I information for active overload protection

//global vars for storing the drq1250 data
drq1250::DRQ1250 drq1; //!< Should provide power for thrusters 1, 5, 4
drq1250::DRQ1250 drq2; //!< Should provide power for thrusters 2, 6, 3
double drq1_idle; //!< How much power drq1 consumes when thrusters are a 0%
double drq2_idle; //!< How much power drq2 consumes when thrusters are a 0%

//template classes for simple functions

/**
* @breif constrians value between min and max inclusive. Value is returned by reference.
* @param[in,out] value input to be constrianed
* @param[in] min The minimum value that "value" should be able to be
* @param[in] max The maximum value that "value" should be able to be
*/
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

/**
* @breif returns the absolute value of value
* @param[in] value
* @return the absolute value of value input
*/
template <class T>
T abs(T value){
    if(value < 0)
        value*=-1;
    return value;
}

/**
* @breif returns the larger number between vlaue1 and value2
* @param[in] value1
* @param[in] value2
* @return The larger of the two values
*/
template <class T>
T max(T value1, T value2){
    if(value1 > value2)
        return value1;
    return value2;
}

/**
* @breif returns the smaller number between value1 and value2
* @param[in] value1
* @param[in] value2
* @return The smaller of the two values
*/
template <class T>
T min(T value1, T value2){
    if(value1 < value2)
        return value1;
    return value2;
}

/**
* @breif returns the normalized array
* @param[in] array to normalize
* @return void as the func is return by reference
*/
void normalize(const double *arr, double *normArr, int size){
  //arr and normArr should be the same size
  //This is a very powerful function using primitive arrays so be carefull
  double max = 0;
  double min = 99999999; //large number
  for(int i=0; i<size; i++){
    if(arr[i] > max){
      max = arr[i];
    }
    if(arr[i] < min){
      min = arr[i];
    }
  }
  if(max-min == 0){
    for(int i=0; i<size;i++){
      normArr[i] = 0;
    }
  } else {
    for(int i=0; i<size;i++){
      normArr[i] = (arr[i]-min)/(max-min);
    }
  }
}

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

/**
* @breif Takes in linearX, linearY, and anguarX percents (-1 to 1) and translates tham to indvidual thrusters percents from -1000 to 1000. These percents are for thrusters T1, T2, T3, and T4.
* @param[in] linearX Equivalent to the left-right axis of the joystick
* @param[in] linearY Equivalent to the front-back axis of the joystick
* @param[in] angularX Equivalent to the rotational/angular axis of the joystick
* @return Const vector_drive/thrusterPercents message ready to be published to the rov/cmd_horizontal_vdrive topic
*/
vector_drive::thrusterPercents vectorMath(const double &linearX, const double &linearY, const double &angularX){
    //if values out of range flag an error
    if(abs(linearX) > 1 || abs(linearY) > 1 || abs(angularX) > 1){
        //ROS_ERROR("cmd_vel value out of range!\nEntering safe mode and disabling thrusters... ");
        ROS_ERROR_STREAM("linearX: " << linearX << "  linearY: " << linearY << "  angularX: " << angularX);
    }

    //inversion, sensitivity, bi-linear threshold are handled in rov_control_interface (drive_control node)
    //deadzone handled by joy package

    //Motor calculations
    //See: https://drive.google.com/file/d/11VF0o0OYVaFGKFvbYtnrmOS0e6yM6IxH/view
    //TODO prevent thrustpercents from going out of 1000 to -1000 range
    double T1 = linearX + linearY + angularX;
    double T2 = -linearX + linearY - angularX;
    double T3 = -linearX - linearY + angularX;
    double T4 = linearX - linearY - angularX;

    //Normalize the values so that no motor outputs over 100% thrust
    double maxMotor = max(max(max(abs(T1), abs(T2)), abs(T3)), abs(T4));
    double maxInput = max(max(abs(linearX), abs(linearY)), abs(angularX));

    if(maxMotor == 0)
        maxMotor = 1;

    T1 *= maxInput / maxMotor;
    T2 *= maxInput / maxMotor;
    T3 *= maxInput / maxMotor;
    T4 *= maxInput / maxMotor;

    ROS_DEBUG_STREAM("T1: " << T1 << "  T2: " << T2 << "  T3: " << T3 << "  T4: " << T4);

    //load thruster values into custom int32 ROS message
    vector_drive::thrusterPercents thrustPercents;
    thrustPercents.t1 = T1*1000;
    thrustPercents.t2 = T2*1000;
    thrustPercents.t3 = T3*1000;
    thrustPercents.t4 = T4*1000;

    return thrustPercents;
}

/*
* @brief predict the amount of power a thruster will consume given the 5th order appox and the percent
* @param[in] percent the percent -100 - 100 to compute power consumption at
*/
double predictPower(double percent){
  //source = https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t100-thruster/
  double power = -1.102300134426306e-9*pow(percent, 5)+4.283627297272621e-7*pow(percent, 4)+
          0.000010392999307079629*pow(percent, 3)+0.01158098074829852*pow(percent, 2)+
          0.0057226220370483576*percent-0.1928628358148441;
  if(power < 0){
    power = 0;
  }
  return power;
}

void activeOverloadProtection(vector_drive::thrusterPercents &horizontals, vector_drive::thrusterPercents &verticals){
  //--------------------------------------------------- Calculate thruster power values
  vector_drive::ThrusterPower thuster_power;

  double horizontal_power_arr[4];
  double vertical_power_arr[4];

  horizontal_power_arr[0] = predictPower(horizontals.t1/10);
  horizontal_power_arr[1] = predictPower(horizontals.t2/10);
  horizontal_power_arr[2] = predictPower(horizontals.t3/10);
  horizontal_power_arr[3] = predictPower(horizontals.t4/10);

  vertical_power_arr[0] = predictPower(verticals.t1/10);
  vertical_power_arr[1] = predictPower(verticals.t2/10);
  vertical_power_arr[2] = predictPower(0);
  vertical_power_arr[3] = predictPower(0);

  //Correct prediction value if DRQ data has come in
  double normArray_horiz[4];
  normalize(horizontal_power_arr, normArray_horiz, 4);
  double normArray_vert[4];
  normalize(vertical_power_arr, normArray_vert, 4);

  double totalPower;
  for(int i=0;i<4;i++){
    totalPower += horizontal_power_arr[i] + vertical_power_arr[i];
  }

  if(ros::Time::now() - drq1.header.stamp < ros::Duration(0.6)){
    if(totalPower == 0){
      drq1_idle = drq1.Pout*drq1_idle/2;
    } else {
      horizontal_power_arr[0] = 0.2*horizontal_power_arr[0] + 0.8*normArray_horiz[0]*(drq1.Pout-drq1_idle); //T1
      horizontal_power_arr[3] = 0.2*horizontal_power_arr[3] + 0.8*normArray_horiz[3]*(drq1.Pout-drq1_idle); //T4
      vertical_power_arr[0] = 0.2*vertical_power_arr[0] + 0.8*normArray_vert[0]*(drq1.Pout-drq1_idle); //T5
      vertical_power_arr[3] = 0.2*vertical_power_arr[3] + 0.8*normArray_vert[3]*(drq1.Pout-drq1_idle); //T8
    }
  }
  if(ros::Time::now() - drq2.header.stamp < ros::Duration(0.6)){
    if(totalPower == 0){
      drq2_idle = drq2.Pout*drq2_idle/2;
    } else {
      horizontal_power_arr[1] = 0.2*horizontal_power_arr[1] + 0.8*normArray_horiz[1]*(drq1.Pout-drq2_idle); //T2
      horizontal_power_arr[2] = 0.2*horizontal_power_arr[2] + 0.8*normArray_horiz[2]*(drq1.Pout-drq2_idle); //T3
      vertical_power_arr[1] = 0.2*vertical_power_arr[1] + 0.8*normArray_vert[1]*(drq1.Pout-drq2_idle); //T6
      vertical_power_arr[2] = 0.2*vertical_power_arr[2] + 0.8*normArray_vert[2]*(drq1.Pout-drq2_idle); //T7
    }
  }

  thuster_power.t1 = horizontal_power_arr[0];
  thuster_power.t2 = horizontal_power_arr[1];
  thuster_power.t3 = horizontal_power_arr[2];
  thuster_power.t4 = horizontal_power_arr[3];
  thuster_power.t5 = vertical_power_arr[0];
  thuster_power.t6 = vertical_power_arr[1];
  thuster_power.t7 = vertical_power_arr[2];
  thuster_power.t8 = vertical_power_arr[3];

  power_pub.publish(thuster_power);

  //--------------------------------------------------- Apply the Active Overload Protection (AOP)
  // di/dt protection not implemented yet

  double powerBudget = 900.0 //Watts (less than 1200 due to votlage drop over tether and the IIn limit) -> eventually make a dynamic reconfigure
  // total power for horizontals
  double totalPowerHorizontal;
  for(int i=0;i<4;i++){
    totalPowerHorizontal += horizontal_power_arr[i];
  }
  // total power for verticals
  double totalPowerVertical;
  for(int i=0;i<4;i++){
    totalPowerVertical += vertical_power_arr[i];
  }



}

/**
* @breif updates control percents, runs vectorMath, updates thruster percents, and publishes the updates thruster percents to the rov/cmd_horizontal_vdrive topic
* @param[in] vel Input from the joystick, ros_control_interface and ROS Control PID algorithms
*/
void commandVectorCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    //linear (L-R)
    double linearX = vel->linear.x;

    //linear (F-B)
    double linearY = vel->linear.y;

    //angular
    double angularX = vel->angular.x;

    vector_drive::thrusterPercents horizThrustPercents;
    horizThrustPercents = vectorMath(linearX, linearY, angularX);

    //Handle verticals
    double linearZ = vel->linear.z;

    double T5 = linearZ*1000;
    double T6 = linearZ*1000;
    double T7 = linearZ*1000;
    double T8 = linearZ*1000;

    vector_drive::thrusterPercents vertThrustPercents;
    vertThrustPercents.t1 = T5;
    vertThrustPercents.t2 = T6;
    vertThrustPercents.t3 = T7;
    vertThrustPercents.t4 = T8;

    activeOverloadProtection(horizThrustPercents, vertThrustPercents);

    //publish messages
    vert_pub.publish(vertThrustPercents);
    horiz_pub.publish(horizThrustPercents);
}

void drq1_cb(const drq1250::DRQ1250 data){
  drq1.header = data.header;
  drq1.Vin = data.Vin;
  drq1.Vout = data.Vout;
  drq1.Iout = data.Iout;
  drq1.Pout = data.Pout;
}

void drq2_cb(const drq1250::DRQ1250 data){
  drq2.header = data.header;
  drq2.Vin = data.Vin;
  drq2.Vout = data.Vout;
  drq2.Iout = data.Iout;
  drq2.Pout = data.Pout;
}

int main(int argc, char **argv)
{
    //initialize node for horizontal vector drive
    ros::init(argc, argv, "horiz_drive");

    ros::NodeHandle n;

    //ROS subscriber to get vectors from the joystick control input
    sub = n.subscribe("/rov/cmd_vel", 1, commandVectorCallback);

    drq1_sub = n.subscribe("drq1250_1/status", 1, drq1_cb);
    drq2_sub = n.subscribe("drq1250_2/status", 1, drq2_cb);

    //ROS publisher to send thruster percent to hardware control node for CAN transmission
    horiz_pub = n.advertise<vector_drive::thrusterPercents>("cmd_horizontal_vdrive", 1);
    //ROS publisher to send thruster percent to hardware control node for CAN transmission
    vert_pub = n.advertise<vector_drive::thrusterPercents>("cmd_vertical_vdrive", 1);
    //ROS publisher to send thruster power
    power_pub = n.advertise<vector_drive::ThrusterPower>("thruster_power", 1);

    ros::spin();

    return 0;
}
