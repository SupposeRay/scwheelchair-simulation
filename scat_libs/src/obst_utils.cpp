#include <scat_libs/obst_utils.h>

namespace obst_utils
{

// // Constructor
// ObstUtils::ObstUtils() {}

// // Destructor
// ObstUtils::~ObstUtils() {}

// tf::TransformListener tf_listener;

// This method converts a list of objects (of type EnvObjectList), to vectors of Points in the target_frame.
std::vector<std::vector<geometry_msgs::Point>> convertObjectListToPoints(const scat_msgs::EnvObjectList &object_list, const std::string &target_frame, const tf::TransformListener &tf_listener)
{
  std::vector<geometry_msgs::Point> obstacle;
  std::vector<std::vector<geometry_msgs::Point>> obstacle_list;  

  if (object_list.header.frame_id == target_frame) // not using transform
  {
    bool object_ok;
    for (auto& object : object_list.objects)
    {
      // Check for NaNs
      object_ok = true;
      for (auto& param : object.params)
      {
        if (std::isnan(param))
        {
          ROS_ERROR_STREAM("Nan value encountered in cam obstacle message");
          object_ok = false;
        }
      }        
      if (object_ok)
      {    
        // Convert Object to Point type, and add to obstacle list
        obstacle.clear();
        // Add circles
        if (object.ID == 7 || object.ID == 8)
        {
          obstacle.push_back(rosmsg::makePoint(object.params[0], object.params[1], object.params[2]));
        }       
        // Add lines
        else if ( object.ID == 1 || object.ID == 2 || object.ID == 3 || object.ID == 6)
        {
          for (int i = 0; i < 6; i += 3)
            obstacle.push_back(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]));
        }
        // Add polygons
        else if (object.ID == 4 || object.ID == 5)
        {
          int polygon_size = static_cast<int>(object.params[0]);
          for (int i = 1; i < 3*polygon_size; i += 3)
            obstacle.push_back(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]));
        }  
        obstacle_list.push_back(obstacle);   
      }
    }
  }
  else // Using transform to transform all Objects into target_frame.
  {
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform(target_frame, object_list.header.frame_id, object_list.header.stamp, ros::Duration(0.1));
      tf_listener.lookupTransform(target_frame, object_list.header.frame_id, object_list.header.stamp, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR_STREAM(ex.what());
      return obstacle_list;
    }  

    bool object_ok;
    for (auto& object : object_list.objects)
    {
      // Check for NaNs
      object_ok = true;
      for (auto& param : object.params)
      {
        if (std::isnan(param))
        {
          ROS_ERROR_STREAM("Nan value encountered in cam obstacle message");
          object_ok = false;
        }
      }        
      if (object_ok)
      {    
        // Convert Object to Point type, transform to target_frame and add to obstacle list
        obstacle.clear();
        // Add circles
        if (object.ID == 7 || object.ID == 8)
        {
          obstacle.push_back(tf_utils::transformPoint(rosmsg::makePoint(object.params[0], object.params[1], object.params[2]), transform));
        }       
        // Add lines
        else if ( object.ID == 1 || object.ID == 2 || object.ID == 3 || object.ID == 6)
        {
          for (int i = 0; i < 6; i += 3)
            obstacle.push_back(tf_utils::transformPoint(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]), transform));
        }
        // Add polygons
        else if (object.ID == 4 || object.ID == 5)
        {
          int polygon_size = static_cast<int>(object.params[0]);
          for (int i = 1; i < 3*polygon_size; i += 3)
            obstacle.push_back(tf_utils::transformPoint(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]), transform));
        }  
        obstacle_list.push_back(obstacle);   
      }
    }
  }

  return obstacle_list;  
}

// This method converts a list of objects (of type EnvObjectList) into the target_frame.
scat_msgs::EnvObjectList transformObjectList(const scat_msgs::EnvObjectList &object_list, const std::string &target_frame, const tf::TransformListener &tf_listener)
{
  if (object_list.header.frame_id == target_frame)
  {
    return object_list;
  }
  else
  {
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform(target_frame, object_list.header.frame_id, object_list.header.stamp, ros::Duration(0.1));
      tf_listener.lookupTransform(target_frame, object_list.header.frame_id, object_list.header.stamp, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_INFO_STREAM(ex.what());
    }  

    scat_msgs::EnvObjectList new_object_list;
    new_object_list.header.stamp = object_list.header.stamp;
    new_object_list.header.frame_id = target_frame;
    bool object_ok;
    for (auto& object : object_list.objects)
    {
      // Check for NaNs
      object_ok = true;
      for (auto& param : object.params)
      {
        if (std::isnan(param))
        {
          ROS_ERROR_STREAM("Nan value encountered in cam obstacle message");
          object_ok = false;
        }
      }        
      if (object_ok)
      {
        // Convert Object to Point type, transform to target_frame and add to obstacle list
        scat_msgs::EnvObject new_object;
        new_object.ID = object.ID;

        // Add circles
        if (object.ID == 7 || object.ID == 8)
        {
          // transform point
          geometry_msgs::Point new_point = tf_utils::transformPoint(rosmsg::makePoint(object.params[0], object.params[1], object.params[2]), transform);
          new_object.params.push_back(new_point.x);
          new_object.params.push_back(new_point.y);
          new_object.params.push_back(new_point.z);      
          // Put in remaining parameters
          for (int i = 3; i < object.params.size(); i++)
            new_object.params.push_back(object.params[i]);
        }    
        // Add lines
        else if ( object.ID == 1 || object.ID == 2 || object.ID == 3 || object.ID == 6)
        {
          // transform points (2)
          for (int i = 0; i < 6; i += 3)
          {
            geometry_msgs::Point new_point = tf_utils::transformPoint(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]), transform);
            new_object.params.push_back(new_point.x);
            new_object.params.push_back(new_point.y);
            new_object.params.push_back(new_point.z);        
          }
          // Put in remaining parameters
          for (int i = 6; i < object.params.size(); i++)
            new_object.params.push_back(object.params[i]);
          }
        // Add polygons
        else if (object.ID == 4 || object.ID == 5)
        {
          int polygon_size = static_cast<int>(object.params[0]);
          for (int i = 1; i < 3*polygon_size; i += 3)
          {
            geometry_msgs::Point new_point = tf_utils::transformPoint(rosmsg::makePoint(object.params[i], object.params[i+1], object.params[i+2]), transform);
            new_object.params.push_back(new_point.x);
            new_object.params.push_back(new_point.y);
            new_object.params.push_back(new_point.z);  
          }
          // Put in remaining parameters
          for (int i = 3*polygon_size; i < object.params.size(); i++)
            new_object.params.push_back(object.params[i]);
        }  
        new_object_list.objects.push_back(new_object);   
      }
    }

    return new_object_list;
  }
}

} // namespace obst_utils
