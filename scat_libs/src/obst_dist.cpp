#include "scat_libs/obst_dist.h"

namespace obst_dist
{

// Constructor
ObstacleDistance::ObstacleDistance() {}

// Destructor
ObstacleDistance::~ObstacleDistance() {}

float ObstacleDistance::get_min_distance(float point_x, float point_y)
{
  // ROS_DEBUG("get_min_distance");
  if (tree_init)
  {
    flann::Matrix<float> query(new float[2], 1, 2);
    query[0][0] = point_x;
    query[0][1] = point_y;

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;

    flann::SearchParams params;
    params.checks = 128;
    params.sorted = true;

    obs_tree->knnSearch(query, indices, dists, 1, params);
    float distance = static_cast<float>(dists[0][0]);

    delete[] query.ptr();
    indices.clear();
    dists.clear();

    return distance;
  }
  else
  {
    ROS_WARN("Obstacle distance requested before tree constructed.");
    return 0.0;
  }
    
}

// Update global list of obstacles, based on map. 
void ObstacleDistance::updateObsgrid(const nav_msgs::OccupancyGrid& costmap)
{
  ROS_DEBUG("updating map");

  map_resolution_ = costmap.info.resolution;
  map_origin_x_ = costmap.info.origin.position.x;
  map_origin_y_ = costmap.info.origin.position.y;
  map_rows_ = costmap.info.height;
  map_cols_ = costmap.info.width;
  map_.header = costmap.header;
  map_.info = costmap. info;
  map_.data = costmap.data;

  map_received_ = true;
}

// Update local list of obstacles, based on laserscan pointcloud
void ObstacleDistance::updateObsgrid(const sensor_msgs::PointCloud& scan_cloud)
{
  // ROS_DEBUG("updating scan");
  obsgrid_scan_.cells.clear();
  obsgrid_scan_.header = scan_cloud.header;
  for (int i = 0; i < scan_cloud.points.size(); i++)
  {
    if (base_utils::euclideanDistance(scan_cloud.points[i]) < max_scan_radius_)
    {
        geometry_msgs::Point point64;
        point64.x = scan_cloud.points[i].x;
        point64.y = scan_cloud.points[i].y;      
        obsgrid_scan_.cells.push_back(point64);
    }
  }
  scan_received_ = true;
  updateObstree();
}

void ObstacleDistance::updatePositionMap()
{
  if  (map_received_)
  {
    // ROS_DEBUG("updating position in map");
    geometry_msgs::PointStamped bot_frame, map_frame;
    bot_frame.header.frame_id = base_frame_id_;
    // bot_frame.header.stamp = timestamp;
    bot_frame.point.x = 0;
    bot_frame.point.y = 0;
    try{
      tf_listener_.transformPoint(fixed_frame_id_, bot_frame, map_frame);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    int bot_col = ceil((map_frame.point.x - map_origin_x_) / map_resolution_) - 1;
    int bot_row = ceil((map_frame.point.y - map_origin_y_) / map_resolution_) - 1;  
    int map_gridsize = local_map_radius_/map_resolution_;
    int row_min = std::max(0,         bot_col-map_gridsize);
    int row_max = std::min(map_rows_, bot_col+map_gridsize);
    int col_min = std::max(0,         bot_row-map_gridsize);
    int col_max = std::min(map_cols_, bot_row+map_gridsize);
    obsgrid_map_.cells.clear();
    obsgrid_map_.header = map_.header;
    for (int i = row_min; i < row_max; i++)
    {
      for (int j = col_min; j < col_max; j++)
      {
        if (map_.data[i * map_cols_ + j] >= occupancy_threshold_)
        {
          geometry_msgs::Point obstacle_coordinates;
          obstacle_coordinates.x = (j * map_resolution_) + map_origin_x_ + (map_resolution_ / 2.0) - map_frame.point.x;
          obstacle_coordinates.y = (i * map_resolution_) + map_origin_y_ + (map_resolution_ / 2.0) - map_frame.point.y;
          obstacle_coordinates.z = 0;
          obsgrid_map_.cells.push_back(obstacle_coordinates);
        }
      }
    }
    position_received_ = true;
    updateObstree();
  }
}

// Place new list of occupied cells in to a tree structure that supports fast
// nearest neighbor searching
void ObstacleDistance::updateObstree()
{
  obsgrid_.cells.clear();
  obsgrid_.header.frame_id = base_frame_id_;
  obsgrid_.header.stamp = ros::Time::now();  
  if (scan_received_)
  {
    for (auto cell : obsgrid_scan_.cells)
    {
        obsgrid_.cells.push_back(cell);
    }
  }
  if (map_received_ && position_received_)
  {
    for (auto cell : obsgrid_map_.cells)
    {
        obsgrid_.cells.push_back(cell);
    }
  }
  // ROS_DEBUG_STREAM("local obsgrid size: " << obsgrid_.cells.size());
  if (obsgrid_.cells.size() > 0)
  {
    if ( tree_init)
    {
      delete obs_tree;
      delete data;
    }
    data = new flann::Matrix<float>(new float[obsgrid_.cells.size() * 2], obsgrid_.cells.size(), 2);

    for (size_t i = 0; i < data->rows; ++i)
    {
      for (size_t j = 0; j < data->cols; ++j)
      {
        if (j == 0)
          (*data)[i][j] = obsgrid_.cells[i].x;
        else
          (*data)[i][j] = obsgrid_.cells[i].y;
      }
    }
    // Obstacle index for fast nearest neighbor search
    obs_tree = new flann::Index<flann::L2<float>>(*data, flann::KDTreeIndexParams(4));
    obs_tree->buildIndex();
    tree_init = true;
  }
}

} // namespace shared_dwa
