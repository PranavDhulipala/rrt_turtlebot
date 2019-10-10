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

#include <gtest/gtest.h>
#include "rrt_turtlebot/kdTree.h"
#include "rrt_turtlebot/rrt.h"

class KdTreeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    for (auto point : points) {
      if (!root) {
        root = std::make_unique<rrt_planner::KdTree>(point, 0, nullptr, nullptr, nullptr);
      } else {
        root->insert(point, std::make_shared<rrt_planner::Node>(point, nullptr));
      }
    }
  }
  std::unique_ptr<rrt_planner::KdTree> root = nullptr;
  std::vector<std::pair<double, double>> points = { std::make_pair(3.0, 6.0),
      std::make_pair(17.0, 15.0), std::make_pair(13.0, 15.0), std::make_pair(
          6.0, 12.0), std::make_pair(9.0, 1.0), std::make_pair(2.0, 7.0),
      std::make_pair(10.0, 19.0) };
  std::pair<double, double> test_var1 = std::make_pair(10.0, 19.0);
  std::pair<double, double> test_var2 = std::make_pair(1.0, 7.0);
};

TEST_F(KdTreeTest, CheckInsertion) {
  ASSERT_EQ(root->right_->right_->left_->point_, test_var1);
}

TEST_F(KdTreeTest, CheckSearch) {
  EXPECT_EQ(root->search(test_var2, nullptr).dist_, 1.0);
  EXPECT_EQ(root->search(test_var2, nullptr).point_, std::make_pair(2.0, 7.0));
  EXPECT_EQ(root->search(test_var2, nullptr).node_->point_,
      std::make_pair(2.0, 7.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, " planner_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
