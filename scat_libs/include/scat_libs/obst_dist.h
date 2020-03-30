#ifndef SCAT_OBST_DIST
#define SCAT_OBST_DIST
// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <nlopt.hpp>
#include <flann/flann.hpp>
// package
#include <scat_libs/base_utils.h>

namespace obst_dist
{

struct PointStruct
{
   float x;
   float y;
   int member;
   int p_idx;

   PointStruct() : x(0), y(0), member(-1), p_idx(0) {}                 // simple constructor with default values
   PointStruct(float a, float b) : x(a), y(b), member(-1), p_idx(0) {} // constructor with x & y value setting

   inline bool operator==(PointStruct position)
   {
      if (position.x == x && position.y == y)
         return true;
      else
         return false;
   }
};

class ObstacleDistance
{
public:
   // Constructor
   ObstacleDistance();
   // Destructor
   virtual ~ObstacleDistance();

   // Public Methods
   void updateObsgrid(const nav_msgs::OccupancyGrid& map);
   void updateObsgrid(const sensor_msgs::PointCloud& scan);
   void updatePositionMap();
   float get_min_distance(float point_x, float point_y);

   // Public Parameters
   float local_map_radius_; 
   float max_scan_radius_;
   float occupancy_threshold_;
   std::string base_frame_id_,fixed_frame_id_;
   ros::Time timestamp;

private:

   // Methods
   void updateObstree();

   // Private Attributes`
   // booleans for message reception
   bool map_received_ = false;
   bool scan_received_ = false;
   bool position_received_ = false;
   bool tree_init = false;   
   // Nearest Neighbour objects
   flann::Index<flann::L2<float>> *obs_tree;
   flann::Matrix<float> *data;
   // cost map gridcells
   nav_msgs::GridCells obsgrid_map_;
   nav_msgs::GridCells obsgrid_scan_;
   nav_msgs::GridCells obsgrid_;   
   // map data
   float map_resolution_, map_origin_x_, map_origin_y_;
   int map_rows_, map_cols_;
   nav_msgs::OccupancyGrid map_;

   tf::TransformListener tf_listener_;

};

} // namespace shared_dwa
#endif