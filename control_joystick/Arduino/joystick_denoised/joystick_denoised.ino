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
//  Serial.print(xValue);
//  Serial.print("\t");
//  Serial.println(yValue);
  float xFinal, yFinal;
  float xOffset = 332;
  float yOffset = 353;
  float xValue = analogRead(joyX) - xOffset;
  float yValue = analogRead(joyY) - yOffset;
  if (xValue >= 0)
  {
    xFinal = xValue / (455 - 332);
  }
  else
  {
    xFinal = xValue / (322 - 210);
  }

  if (yValue >= 0)
  {
    yFinal = yValue / (490 - 353);
  }
  else
  {
    yFinal = yValue / (353 - 240);
  }
  
  joy_msg.data[0] = xFinal;
  joy_msg.data[1] = yFinal;

  joy_data.publish( &joy_msg );
  nh.spinOnce();
  
  delay(10);
}
