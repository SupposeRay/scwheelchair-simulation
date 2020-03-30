#include <scat_libs/nav_utils.h>

namespace nav_utils
{

std::vector<geometry_msgs::Point> moveFootprint(std::vector<geometry_msgs::Point> footprint,
                                                const double &x, const double &y, const double &theta)
{
  for (geometry_msgs::Point &corner : footprint)
  {
    corner = geom_utils::localToGlobal(corner, x, y, theta);
  }
  return footprint;
}

/*!
 * \details Abstract method which creates a vector of ROS points describing a polygon (each point once), from a list of coordinates. 
 * coord_dim gives the dimension of input coordinates. 
 * If the input coords are given as [x1, y1, z1, ..], coord_dim should be 3. 
 * If the input coords are given as [x1, y1, ..], coord_dim should be 2.
 * All dimensions can be increase by a factor given by 'inflation'
 */
std::vector<geometry_msgs::Point> createPolygon2D(const std::vector<double> &coords,
                                                  const double &inflation,
                                                  const int &coord_dim)
{
  std::vector<geometry_msgs::Point> polygon;
  std::vector<double> x_coords, y_coords;
  for (int i = 0; i < coords.size(); i += coord_dim)
  {
    x_coords.push_back(coords[i]);
    y_coords.push_back(coords[i + 1]);
  }
  double x_centroid = std::accumulate(x_coords.begin(), x_coords.end(), 0.0) / x_coords.size();
  double y_centroid = std::accumulate(y_coords.begin(), y_coords.end(), 0.0) / y_coords.size();

  for (int i = 0; i < x_coords.size(); i += 1)
  {
    geometry_msgs::Point corner;
    corner.x = x_coords[i] + (x_coords[i] - x_centroid) * inflation;
    corner.y = y_coords[i] + (y_coords[i] - y_centroid) * inflation;
    polygon.push_back(corner);
  }
  return polygon;
}

/*!
 * \details Abstract method which creates a vector of ROS points describing a polygon (each point once), from a list of coordinates. 
 * coord_dim gives the dimension of input coordinates. 
 * If the input coords are given as [x1, y1, z1, ..], coord_dim should be 3. 
 * If the input coords are given as [x1, y1, ..], coord_dim should be 2.
 * All dimensions can be increase by a factor given by 'inflation'
 */
std::vector<geometry_msgs::Point> createPolygon2D(const std::vector<float> &coords,
                                                  const float &inflation,
                                                  const int &coord_dim)
{
  std::vector<geometry_msgs::Point> polygon;
  std::vector<float> x_coords, y_coords;
  for (int i = 0; i < coords.size(); i += coord_dim)
  {
    x_coords.push_back(coords[i]);
    y_coords.push_back(coords[i + 1]);
  }

  float x_centroid = std::accumulate(x_coords.begin(), x_coords.end(), 0.0) / x_coords.size();
  float y_centroid = std::accumulate(y_coords.begin(), y_coords.end(), 0.0) / y_coords.size();

  for (int i = 0; i < x_coords.size(); i += 1)
  {
    geometry_msgs::Point corner;
    corner.x = x_coords[i] + (x_coords[i] - x_centroid) * inflation;
    corner.y = y_coords[i] + (y_coords[i] - y_centroid) * inflation;
    polygon.push_back(corner);
  }
  return polygon;
}

// Checks whether any of the points in the pointcloud is 'inside' the footprint when the footprint is at coordinates x,y,theta. 
// If so, returns true, corresponding to a detected collision. 
bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const sensor_msgs::PointCloud &pointcloud,
                                const double &x, const double &y, const double &theta)
{
  bool collision = false;
  footprint = nav_utils::moveFootprint(footprint, x, y, theta);
  for (auto point : pointcloud.points)
  {
    if (geom_utils::isInsidePolygon(footprint, point))
    {
      collision = true;
    }
  }

  return collision;
}

// Checks whether any of the points in the pointcloud is 'inside' the footprint when the footprint is at coordinates x,y,theta. 
// If so, returns true, corresponding to a detected collision. This method also returns which points are in collision, through the vector coll_points. 
bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const sensor_msgs::PointCloud &pointcloud,
                                const double &x, const double &y, const double &theta,
                                std::vector<geometry_msgs::Point32> &coll_points)
{
  bool collision = false;
  coll_points.clear();
  footprint = nav_utils::moveFootprint(footprint, x, y, theta);
  for (auto point : pointcloud.points)
  {
    if (geom_utils::isInsidePolygon(footprint, point))
    {
      collision = true;
      coll_points.push_back(point);
    }
  }
  return collision;
}

// Checks whether any of obstacles collides with the footprint at coordinates x,y,theta. Obstacles are given as a vector of points. 
// If so, returns true, corresponding to a detected collision. 
bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const std::vector<std::vector<geometry_msgs::Point>> &obstacles,
                                const double &x, const double &y, const double &theta)
{
  bool collision = false;
  footprint = nav_utils::moveFootprint(footprint, x, y, theta);
  // Iterate through the obstacle map
  for (auto& obstacle : obstacles)
  {
    
    // Step 1: Check if any point of the obstacle is inside the footprint or not. Reports collision if true.
    for (int i = 0; i < obstacle.size(); i++)
    {    
      if (geom_utils::isInsidePolygon(footprint, obstacle[i]))
      {
        collision = true;
        break;
      }
    }
    if (collision) break;

    // Step 2: Check if there are any intersections between footprint edges and obstacle edges
    if (obstacles.size() > 1)
    {
      for (int i = 0; i < footprint.size(); i++)
      {
        // Iterate through the sides of the robot footprint polygon
        auto& point1 = footprint[i];
        auto& point2 = footprint[(i+1)%footprint.size()];
        // Iterate through the sides of the obstacle polygon. 
        // Report collision if any of the sides on the obstacle collide with any of the sides of the footprint. 
        for (int i = 0; i < obstacle.size(); i++)
        {
          if (geom_utils::doIntersect(point1, point2, obstacle[i], obstacle[(i+1)%obstacle.size()]))
          {
            collision = true;
            break;
          }
        }
      }
    }
    if (collision) break;
  }
  return collision;
}

// Checks whether any of obstacles collides with the footprint at coordinates x,y,theta. Obstacles are given as a vector of points. 
// If so, returns true, corresponding to a detected collision. 
bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const std::vector<std::vector<geometry_msgs::Point>> &obstacles,
                                const double &x, const double &y, const double &theta,
                                std::vector<geometry_msgs::Point32> &coll_points)
{
  bool collision = false;
  footprint = nav_utils::moveFootprint(footprint, x, y, theta);
  // Iterate through the obstacle map
  for (auto& obstacle : obstacles)
  {
    
    // Step 1: Check if any point of the obstacle is inside the footprint or not. Reports collision if true.
    for (int i = 0; i < obstacle.size(); i++)
    {    
      if (geom_utils::isInsidePolygon(footprint, obstacle[i]))
      {
        collision = true;
        coll_points.push_back(rosmsg::makePoint32(obstacle[i].x,obstacle[i].y));
      }
    }
    if (collision) break;

    // Step 2: Check if there are any intersections between footprint edges and obstacle edges
    if (obstacles.size() > 1)
    {
      for (int i = 0; i < footprint.size(); i++)
      {
        // Iterate through the sides of the robot footprint polygon
        auto& point1 = footprint[i];
        auto& point2 = footprint[(i+1)%footprint.size()];
        // Iterate through the sides of the obstacle polygon. 
        // Report collision if any of the sides on the obstacle collide with any of the sides of the footprint. 
        for (int i = 0; i < obstacle.size(); i++)
        {
          if (geom_utils::doIntersect(point1, point2, obstacle[i], obstacle[(i+1)%obstacle.size()]))
          {
            collision = true;
            coll_points.push_back(rosmsg::makePoint32(obstacle[i].x,obstacle[i].y));
            coll_points.push_back(rosmsg::makePoint32(obstacle[(i+1)%obstacle.size()].x,obstacle[(i+1)%obstacle.size()].y));
          }
        }
      }
    }
    if (collision) break;
  }
  return collision;
}


/*!
 * \details returns true if collision detected, based on radial distance from the points in the laserscan message, 
 */
bool checkRadiusForCollision(const double &robot_radius,
                             const sensor_msgs::LaserScan &scan,
                             const double &x, const double &y, const double &theta,
                             const double &angle_res, const double &obst_inflation)
{
  bool collision = false;
  double angle = atan2(y, x);
  double dist = base_utils::euclideanDistance(x, y);
  int range_min = static_cast<int>(floor((angle - angle_res - scan.angle_min) / scan.angle_increment));
  int range_max = static_cast<int>(ceil((angle + angle_res - scan.angle_min) / scan.angle_increment));
  for (int i = range_min; i <= range_max; i++)
  {
    if (scan.ranges[i] < dist + robot_radius + obst_inflation)
    {
      double theta = static_cast<double>(i) * scan.angle_increment + scan.angle_min;
      double range = scan.ranges[i];
      collision = true;
    }
  }
  return collision;
}

/*!
 * \details returns true if collision detected, based on distance from the points in the point cloud.
 *  This method also returns which points are in collision, through the vector coll_points. 
 */
bool checkRadiusForCollision(const double &robot_radius,
                             const sensor_msgs::LaserScan &scan,
                             const double &x, const double &y, const double &theta,
                             const double &angle_res, const double &obst_inflation,
                             std::vector<double> &coll_angles)
{
  bool collision = false;
  coll_angles.clear();
  double angle = atan2(y, x);
  double dist = base_utils::euclideanDistance(x, y);
  int range_min = static_cast<int>(floor((angle - angle_res - scan.angle_min) / scan.angle_increment));
  int range_max = static_cast<int>(ceil((angle + angle_res - scan.angle_min) / scan.angle_increment));
  for (int i = range_min; i <= range_max; i++)
  {
    if (scan.ranges[i] < dist + robot_radius + obst_inflation)
    {
      double theta = static_cast<double>(i) * scan.angle_increment + scan.angle_min;
      double range = scan.ranges[i];
      collision = true;
      coll_angles.push_back(theta);
    }
  }
  return collision;
}

} // namespace nav_utils
