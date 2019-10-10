// The MIT License (MIT)

// Copyright (c) 2019 Pranav

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

/* code */
#ifndef RRT_TURTLEBOT_H
#define RRT_TURTLEBOT_H
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <ctime>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace rrt_planner {

class RrtPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RrtPlanner();
  RrtPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
  bool checkGoal(std::pair<double, double> current);
  bool isSafe(std::pair<double, double> nearest_point,
              std::pair<double, double> new_point);
  std::vector<geometry_msgs::PoseStamped> planner();
  std::pair<double, double> randomPoint();
  std::pair<std::pair<double, double>, bool> moveTo(
      std::pair<double, double> from_, std::pair<double, double> towards_);
  /**
   * @brief calculates the euclidean distance
   *
   * @param point1
   * @param point2
   * @return double the euclidean distance
   */
  inline double distance(std::pair<double, double> point1,
                         std::pair<double, double> point2) {
    return sqrt(
        (point1.first - point2.first) * (point1.first - point2.first)
            + (point1.second - point2.second) * (point1.second - point2.second));
  }

 private:
  ros::NodeHandle nh_;
  std::vector<std::vector<bool>> obstacle_map_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  bool initialized_;
  double goal_radius_ = 1;
  double epsilon_ = 1;
  unsigned int map_width_;
  unsigned int map_height_;
  std::pair<double, double> start_;
  std::pair<double, double> goal_;
  int max_iterations_ = 500000;
  double robot_radius_ = 0.4;
  base_local_planner::WorldModel *world_model_;
  visualization_msgs::Marker marker;
  ros::Publisher marker_pub;
};

}  // namespace rrt_planner
#endif
