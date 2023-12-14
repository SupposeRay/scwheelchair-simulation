#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float32MultiArray joy_msg;
ros::Publisher joy_data("/joy_data/remote", &joy_msg);

#define joyX A0
#define joyY A1

void setup() {
  Serial.begin(57600);
  float joy_values[] = {0,0};
  joy_msg.data = joy_values;
  joy_msg.data_length = 2;
  nh.initNode();
  nh.advertise(joy_data);
}
 
void loop() {

  float xFinal, yFinal;
  float xOffset = 480;
  float yOffset = 490;
  float xValue = analogRead(joyX) - xOffset;
  float yValue = analogRead(joyY) - yOffset;

  xFinal = xValue / 500;
  yFinal = yValue / 500;
//  if (xValue >= 0)
//  {
//    xFinal = -xValue / (455 - 332) / 2.0;
//  }
//  else
//  {
//    xFinal = -xValue / (322 - 210) / 2.0;
//  }
//
//  if (yValue >= 0)
//  {
//    yFinal = -yValue / (490 - 353) / 1.7;
//  }
//  else
//  {
//    yFinal = -yValue / (353 - 240) / 2.0;
//  }
//  Serial.print(xFinal);
//  Serial.print("\t");
//  Serial.println(yFinal);
  if(abs(xFinal) < 0.1)
    xFinal = 0;

  if(abs(yFinal) < 0.1)
    yFinal = 0;
    
  joy_msg.data[0] = xFinal;
  joy_msg.data[1] = yFinal;

  joy_data.publish( &joy_msg );
  nh.spinOnce();
  
  delay(10);
}
