
#include <ros.h>
//#include <sensor_msgs/Joy.h>
//#include <detect_box/angle.h>
//#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
ros::NodeHandle nh;

//sensor_msgs::Joy joy_msg;
//detect_box::angle joy_msgo ;
//std_msgs::Float32MultiArray joy_msg;
geometry_msgs::Point joy_msg;
ros::Publisher joy_data("/arduino/joystick", &joy_msg);

#define joyX A0
#define joyY A1

void setup() {
 //Serial.begin(57600);
  nh.initNode();
  nh.advertise(joy_data);
}

void loop() {
  float xValue = -analogRead(joyX);
  float yValue = analogRead(joyY);
  Serial.print(xValue);
  Serial.print("\t");
  Serial.println(yValue);

  //joy_msg.angle_Start = xValue;
  //joy_msg.angle_End = yValue;
 float xMax = -971;
 float xMin = -20;
 float xZero = -495;

 float yMax = 993;
 float yMin = 17;
 float yZero = 506;

 if(-xValue > -xMax)
 {
  joy_msg.x = 1;
 }
 else
 {
  if (-xValue > -xZero)
  {
    joy_msg.x = (-xValue + xZero) / (-xMax+xZero);
  }
  else
  {
    if(-xValue > -xMin)
    {
      joy_msg.x = (-xValue + xZero) / (-xZero + xMin);
    }
    else
    {
      joy_msg.x = -1;
    }
  }
 }

 if(yValue > yMax)
 {
  joy_msg.y = 1;
 }
 else
 {
  if (yValue > yZero)
  {
    joy_msg.y = (yValue - yZero) / (yMax-yZero);
  }
  else
  {
    if(yValue > yMin)
    {
      joy_msg.y = (yValue - yZero) / (yZero - yMin);
    }
    else
    {
      joy_msg.y = -1;
    }
  }
 }


  //if (yValue > 355)
  //{
   // joy_msg.data[1] = (yValue - 355) / 140;
  //}
  //else
  //{
  //  joy_msg.data[1] = (yValue - 355) / 115;
 // }
 // joy_msg.data[0] = xValue;//-512)/354
 // joy_msg.data[1] = yValue;//-512)/354
  //joy_msg.axes[0] = xValue;
  //joy_msg.axes[1] = yValue;
  //char hello[16] = "hello my world!";
//joy_msg.data = hello;
//joy_msg.x = xValue;
//joy_msg.y = yValue;
//  if(abs(joy_msg.linear.x) < 0.2)
//    joy_msg.linear.x = 0;
//
//  if(abs(joy_msg.angular.z) < 0.2)
//    joy_msg.angular.z = 0;
//
  joy_data.publish( &joy_msg );
  nh.spinOnce();


  delay(10);
}
