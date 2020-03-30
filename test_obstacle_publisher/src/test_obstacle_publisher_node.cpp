#include "test_obstacle_publisher/test_obstacle_publisher_node.h"

namespace test_obstacle_publisher {

//! Constructor
ObstaclePublisher::ObstaclePublisher(ros::NodeHandle& node_handle) 
: node_handle_(node_handle)
{
  ROS_DEBUG("Launching Test Obstacle Publisher Node");


  if (!readParameters()) 
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }

  //! Subscribers & Publishers
  obst_marker_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/test_obst_markers", 1);
  obst_msg_publisher_ = node_handle_.advertise<scat_msgs::EnvObjectList>("/test_obst_msg", 1);
  timer_ = node_handle_.createTimer(ros::Duration(1/publish_frequency_), &ObstaclePublisher::timerCallback, this);

  ROS_DEBUG("Succesfully launched Test Obstacle Publisher Node");

}

ObstaclePublisher::~ObstaclePublisher(){}

/*!
 * \brief function which reads parameters from parameterserver
 */
bool ObstaclePublisher::readParameters()
{
  if (!node_handle_.getParam("publish_frequency", publish_frequency_))
    ROS_WARN_STREAM("Parameter publish_frequency not set for test_obstacle_publisher. Using default setting: " << publish_frequency_); 
  if (!node_handle_.getParam("frame_id", frame_id_))
    ROS_WARN_STREAM("Parameter frame_id not set for test_obstacle_publisher. Using default setting: " << frame_id_); 

  float inflation_factor = 0.0;
  int coord_dim = 3;
  std::vector<float> object_params;
  scat_msgs::EnvObject object_msg;
  object_msg_list_.header.frame_id = frame_id_;
  int n_pars; // number of parameters per object
  int marker_ID = 1;

  if (node_handle_.getParam("objectID_1", object_params))
  {
    ROS_INFO_STREAM("Publishing Walls: " << object_params.size()/6);
    // Walls
    object_msg.ID = 1;
    // Shape: Line. Consists of two 3D points
    n_pars = 6;    
    // create objects
    for (int i = 0; i < object_params.size(); i += n_pars)
    {
      std::vector<float> object_i( &object_params[i], &object_params[i]+n_pars );
      object_msg.params = object_i;
      object_msg_list_.objects.push_back(object_msg);

      // create marker
      visualization_msgs::Marker object_marker;
      object_marker.header.frame_id = frame_id_;      
      object_marker.type = visualization_msgs::Marker::LINE_LIST;
      object_marker.id = marker_ID;
      marker_ID += 1;
      object_marker.color.r = 1.0;
      object_marker.color.a = 1.0;
      object_marker.scale.x = 0.1;    
      object_marker.points.push_back(rosmsg::makePoint(object_i[0],object_i[1]));
      object_marker.points.push_back(rosmsg::makePoint(object_i[3],object_i[4]));
      object_marker_list_.markers.push_back(object_marker);
    }    
  }

  if (node_handle_.getParam("objectID_2", object_params))
  {
    ROS_INFO_STREAM("Publishing Steps: " << object_params.size()/6);
    // Steps / Curbs
    object_msg.ID = 2;
    // Shape: Line. Consists of two 3D points
    n_pars = 6;        
    // create objects
    for (int i = 0; i < object_params.size(); i += n_pars)
    {
      std::vector<float> object_i( &object_params[i], &object_params[i]+n_pars );
      object_msg.params = object_i;
      object_msg_list_.objects.push_back(object_msg);

      // create marker
      visualization_msgs::Marker object_marker;
      object_marker.header.frame_id = frame_id_;  
      object_marker.type = visualization_msgs::Marker::LINE_LIST;
      object_marker.id = marker_ID;
      marker_ID += 1;
      object_marker.color.r = 0.5;
      object_marker.color.g = 0.5;
      object_marker.color.a = 1.0;
      object_marker.scale.x = 0.1; 
      object_marker.points.push_back(rosmsg::makePoint(object_i[0],object_i[1]));
      object_marker.points.push_back(rosmsg::makePoint(object_i[3],object_i[4]));
      object_marker_list_.markers.push_back(object_marker);
    } 
  }

  if (node_handle_.getParam("objectID_7", object_params))
  {
    ROS_INFO_STREAM("Publishing Tables: " << object_params.size()/12);
    // Tables / Desks
    object_msg.ID = 7;
    // Shape: Polygon(Rectangle). Consists of four 3D points
    n_pars = 12;   
    // create objects
    for (int i = 0; i < object_params.size(); i += n_pars)
    {
      std::vector<float> object_i( &object_params[i], &object_params[i]+n_pars );
      object_msg.params = object_i;
      object_msg_list_.objects.push_back(object_msg);

      // create marker
      visualization_msgs::Marker object_marker;
      object_marker.header.frame_id = frame_id_;  
      object_marker.type = visualization_msgs::Marker::LINE_LIST;
      object_marker.id = marker_ID;
      marker_ID += 1;
      object_marker.color.g = 1.0;
      object_marker.color.a = 1.0;
      object_marker.scale.x = 0.1;  
      object_marker.points = rosmsg::makeMarkerPoints(nav_utils::createPolygon2D(object_i,inflation_factor,coord_dim)); 
      object_marker_list_.markers.push_back(object_marker);
    } 
  }

  if (node_handle_.getParam("objectID_8", object_params))
  {
    // humans
    object_msg.ID = 8;
    // Shape: Circle. consists of one 3D point + radius
    n_pars = 4;   
    ROS_INFO_STREAM("Publishing humans: " << object_params.size()/n_pars);
    // create objects
    for (int i = 0; i < object_params.size(); i += n_pars)
    {
      std::vector<float> object_i( &object_params[i], &object_params[i]+n_pars );
      object_msg.params = object_i;
      object_msg_list_.objects.push_back(object_msg);

      // create marker
      visualization_msgs::Marker object_marker;
      object_marker.header.frame_id = frame_id_;  
      object_marker.type = visualization_msgs::Marker::CYLINDER;
      object_marker.id = marker_ID;
      marker_ID += 1;
      object_marker.color.g = 1.0;
      object_marker.color.a = 1.0;
      object_marker.scale.x = object_i[3];  
      object_marker.pose = rosmsg::makePose(object_i[0], object_i[1], 0); 
      object_marker_list_.markers.push_back(object_marker);
    } 
  }  
  
  ROS_DEBUG("Parameters read.");
  return true;
}

/*!
 * \brief callback for timer
 */
void ObstaclePublisher::timerCallback(const ros::TimerEvent&)
{
  object_msg_list_.header.stamp = ros::Time::now();
  obst_msg_publisher_.publish(object_msg_list_);
  obst_marker_publisher_.publish(object_marker_list_);
}

} /* namespace */
