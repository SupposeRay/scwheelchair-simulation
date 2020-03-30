#include <ros/ros.h>
#include <geometry_msgs/Twist.h>	// outgoing message type
#include <math.h>

double v_ = 0.25; 
double w_ = 1*M_PI/10;
double publish_frequency_ =  10;
double duration_ = 5;

int main(int argc, char **argv)
{
	///// Initialize Node	
  ros::init(argc, argv, "auto_cmd");
  ros::NodeHandle node_handle("~");

  if (!(
          node_handle.getParam("v", v_) &&
          node_handle.getParam("w", w_) &&
          node_handle.getParam("publish_frequency", publish_frequency_) &&
          node_handle.getParam("duration", duration_)
    )){
      ROS_ERROR("Could not load parameters.");
      ros::requestShutdown();
    }
  else ROS_INFO("parameters succesfully read");

  ///// Publishers /////
  // ros::Publisher usercmd_publisher = node_handle.advertise<geometry_msgs::Twist>("/user/cmd_vel", 1);
  ros::Publisher cmd_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  geometry_msgs::Twist cmd_vel;		

  ros::Rate loop_rate(publish_frequency_);
  ros::spinOnce();

  double t = 0;
  while ((ros::ok()) && (t < duration_))
  {
    cmd_vel.linear.x = v_;
    cmd_vel.angular.z = w_;

    cmd_publisher.publish(cmd_vel);
    t += 1/publish_frequency_;

    ROS_INFO_STREAM("time step: " << t << " V: " << cmd_vel.linear.x << " W: " << cmd_vel.angular.z );
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ROS_INFO_STREAM("Loop finished, publishing zero velocity");

  int i = 0;
  while (i<30)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_publisher.publish(cmd_vel);
    loop_rate.sleep();
    i++;
  }
    
  ROS_INFO_STREAM("Closing node");

  return 0;
}
