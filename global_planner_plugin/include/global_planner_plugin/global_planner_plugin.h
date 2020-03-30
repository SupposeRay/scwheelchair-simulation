#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.hpp>
// ROS general
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
// Messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseGoal.h>

#ifndef PROBABILISTIC_PLANNER_H_
#define PROBABILISTIC_PLANNER_H_

namespace global_planner_plugin {
  class GlobalPlannerPlugin : public nav_core::BaseGlobalPlanner {
    public:


        /**
         * @brief  Default constructor for the NavFnROS object
         */
      GlobalPlannerPlugin();

        /**
         * @brief  Constructor 
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
         */
      GlobalPlannerPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      
        /**
        * @brief Default destructor
        */
      ~GlobalPlannerPlugin();

        /**
         * @brief  Initialization function 
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
         */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief Compute a plan
       * @param start_pose The starting pose of the robot
       * @param goal The goal pose
       * @param plan The plan filled by the planner
       * @return True if a valid plan was found, false otherwise
       */      
      bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan);
      

    private:
      bool initialized_, trajectory_received_;
      costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
      std::string name_;
      ros::Subscriber sub_trajectory_;
      ros::Publisher pub_plan_;
      ros::Publisher pub_goal_;
      
      void trajectoryCallback(const nav_msgs::PathConstPtr& trajectory);

      // containers
      nav_msgs::Path path_;
      move_base_msgs::MoveBaseGoal trajectoryGoal_;
  };
};

#endif
