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

#include <pluginlib/class_list_macros.h>
#include <rrt_turtlebot/kdTree.h>
#include <rrt_turtlebot/rrt.h>

PLUGINLIB_EXPORT_CLASS(rrt_planner::RrtPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {
  /**
   * @brief Constructor
   */
  RrtPlanner::RrtPlanner() : costmap_ros_(nullptr), initialized_(false) {}
  /**
   * @brief Constructor
   * @param name
   * @param cost_map ros
   */
  RrtPlanner::RrtPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  : costmap_ros_(costmap_ros) {
    initialize(name, costmap_ros);
  }
  /** @brief overridden class from nav_core::BaseGlobalPlanner
   * @param name
   * @param cost_map ros
   */
  void RrtPlanner::initialize(std::string name,
      costmap_2d::Costmap2DROS *costmap_ros) {
    if (!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      ros::NodeHandle nh("~/" + name);
      nh_ = nh;
      map_width_ = costmap_->getSizeInCellsX();
      map_height_ = costmap_->getSizeInCellsY();
      obstacle_map_.resize(map_height_, std::vector<bool>(map_width_));
      for (auto j = 0; j < map_height_; j++) {
        for (auto i = 0; i < map_width_; i++) {
          auto cost = static_cast<int>(costmap_->getCost(i, j));
          if (cost > 100) {
            obstacle_map_.at(j).at(i) = true;
          } else {
            obstacle_map_.at(j).at(i) = false;
          }
        }
      }

      std::cout << "Planner initialized successfully.\n";
      initialized_ = true;
    } else {
      std::cout << "Planner has already been initialized.\n";
    }
  }
  /**
   * @brief the virtual method of the base class
   * @param start
   * @param goal
   * @param path
   * @return bool
   */
  bool RrtPlanner::makePlan(const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      std::vector<geometry_msgs::PoseStamped> &plan) {
    if (!initialized_) {
      std::cout << "initialize to use the planner\n";
      return false;
    }
    plan.clear();
    start_.first = start.pose.position.x;
    start_.second = start.pose.position.y;
    goal_.first = goal.pose.position.x;
    goal_.second = goal.pose.position.y;
    marker_pub =
    nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "rrt_planner";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal.pose.position.x;
    marker.pose.position.y = goal.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 0.3f;
    marker.color.g = 0.3f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      std::cout << "This planner will only accept goals in the "
      << costmap_ros_->getGlobalFrameID().c_str()
      << "frame, "
      "the goal was sent to the "
      << goal.header.frame_id.c_str() << " frame.\n";
      return false;
    }
    plan = planner();

    if (plan.size() > 1) {
      std::cout << "A path was found.\n";
      marker_pub.publish(marker);
      return true;
    } else {
      std::cout << "No path was found.\n";
      return false;
    }
  }
  /**
   * @brief checks if the point is within the radius of the goal
   *
   * @param current
   * @return true
   * @return false
   */
  bool RrtPlanner::checkGoal(std::pair<double, double> current) {
    if (distance(current, goal_) <= goal_radius_) {
      return true;
    }
    return false;
  }
  /**
   * @brief checks if the chosen point is safe
   *
   * @return bool
   */
  bool RrtPlanner::isSafe(std::pair<double, double> nearest_point,
      std::pair<double, double> new_point) {
    std::vector<std::pair<double, double>> neighbours = {
      std::make_pair(-robot_radius_, -robot_radius_),
      std::make_pair(robot_radius_, -robot_radius_),
      std::make_pair(-robot_radius_, robot_radius_),
      std::make_pair(robot_radius_, robot_radius_),
      std::make_pair(0.0, -robot_radius_),
      std::make_pair(-robot_radius_, 0.0),
      std::make_pair(0.0, robot_radius_),
      std::make_pair(robot_radius_, 0.0)};
    for (auto n : neighbours) {
      std::pair<double, double> temp = std::make_pair(new_point.first + n.first,new_point.second + n.second);
      unsigned int x_, y_;
      costmap_->worldToMap(temp.first, temp.second, x_, y_);
      if (obstacle_map_.at(y_).at(x_)) {
        return false;
      }
    }
    return true;
  }
  /**
   * @brief RRT planning algorithm
   *
   * @return std::vector<geometry_msgs::PoseStamped>
   */
  std::vector<geometry_msgs::PoseStamped> RrtPlanner::planner() {
    std::vector<geometry_msgs::PoseStamped> plan;
    std::shared_ptr<Node> root = std::make_shared<Node>(start_, nullptr);
    std::unique_ptr<KdTree> tree=std::make_unique<KdTree>(start_, 0, nullptr, nullptr, root);
    std::pair<double, double> current = start_;
    while (!checkGoal(current) && (max_iterations_)) {
      std::pair<double, double> random_ = randomPoint();
      Nns res = tree->search(random_, nullptr);
      std::pair<double, double> nearest = res.point_;
      std::pair<double, double> new_point = moveTo(nearest, random_).first;
      bool flag = moveTo(nearest, random_).second;
      if (flag) {
        std::shared_ptr<Node> temp = std::make_shared<Node>(new_point, res.node_);
        tree->insert(new_point, temp);
        current = new_point;
        max_iterations_--;
      } else {
        max_iterations_--;
        continue;
      }
    }
    Nns res = tree->search(current, nullptr);
    std::shared_ptr<Node> node = res.node_;
    geometry_msgs::PoseStamped pos;
    pos.header.frame_id = "map";
    pos.pose.position.x = goal_.first;
    pos.pose.position.y = goal_.second;
    pos.pose.position.z = 0;
    pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    plan.push_back(pos);
    while (node->parent_) {
      pos.header.frame_id = "map";
      pos.pose.position.x = node->point_.first;
      pos.pose.position.y = node->point_.second;
      pos.pose.position.z = 0;
      pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      plan.push_back(pos);
      node = node->parent_;
    }
    std::reverse(plan.begin(), plan.end());
    return plan;
  }
  /**
   * @brief generates a random point
   *
   * @return std::pair<double, double> the random point
   */
  std::pair<double, double> RrtPlanner::randomPoint() {
    srand(time(NULL));
    std::random_device rd;
    std::mt19937 rand_x(rd());
    std::mt19937 rand_y(rd());
    std::uniform_int_distribution<> dist_x(-costmap_->getSizeInMetersX(),
        costmap_->getSizeInMetersX());
    std::uniform_int_distribution<> dist_y(-costmap_->getSizeInMetersY(),
        costmap_->getSizeInMetersY());
    return std::make_pair(dist_x(rand_x), dist_y(rand_y));
  }
  /**
   * @brief returns a point at an epsilon distance in the direction of the second
   * point
   *
   * @param from_
   * @param towards_
   * @return std::pair<std::pair<double, double>, bool>
   */
  std::pair<std::pair<double, double>, bool> RrtPlanner::moveTo(
      std::pair<double, double> from_, std::pair<double, double> towards_) {
    double angle =
    atan2(towards_.second - from_.second, towards_.first - from_.first);
    std::pair<double, double> next_point =
    std::make_pair((from_.first + epsilon_ * cos(angle)),
        (from_.second + epsilon_ * sin(angle)));
    if (isSafe(from_, next_point)) {
      return std::make_pair(next_point, true);
    }
    return std::make_pair(next_point, false);
  }

}  // namespace rrt_planner
