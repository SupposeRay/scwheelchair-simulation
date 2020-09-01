#include <ros.h>
//#include <sensor_msgs/Joy.h>
//#include <detect_box/angle.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

//sensor_msgs::Joy joy_msg;
//detect_box::angle joy_msg;
control_joystick::joymsg joy_msg;
ros::Publisher joy_data("/joy_data", &joy_msg);

#define joyX A1
#define joyY A0

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(joy_data);
}
 
void loop() {
  float xValue = analogRead(joyX);
  float yValue = analogRead(joyY);
//  Serial.print(xValue);
//  Serial.print("\t");
//  Serial.println(yValue);

  //joy_msg.angle_Start = xValue;
  //joy_msg.angle_End = yValue;
  if (xValue > 334)
  {
    joy_msg.x = (xValue - 334) / 126;    
  }
  else
  {
    joy_msg.x = (xValue - 334) / 119;
  }
  if (yValue > 355)
  {
    joy_msg.y = (yValue - 355) / 140;    
  }
  else
  {
    joy_msg.y = (yValue - 355) / 115;
  }
//  joy_msg.x = xValue;//-512)/354
//  joy_msg.y = yValue;//-512)/354
  //joy_msg.axes[0] = xValue;
  //joy_msg.axes[1] = yValue;

  joy_data.publish( &joy_msg );
  nh.spinOnce();
  
  
  delay(50);
}
