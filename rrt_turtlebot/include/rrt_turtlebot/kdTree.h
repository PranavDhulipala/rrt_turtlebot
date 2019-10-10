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
#ifndef KDTREE_H
#define KDTREE_H
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace rrt_planner {
struct Node {
  std::pair<double, double> point_ = std::make_pair(std::nan("1"),
                                                    std::nan("1"));
  std::shared_ptr<Node> parent_ = nullptr;
  Node(std::pair<double, double> point, std::shared_ptr<Node> parent)
      : point_(point) {
    parent_ = parent;
  }
  Node(std::pair<double, double> point)
      : point_(point) {
  }
  Node() {
  }
  ~Node() {
  }
};

struct Nns {
  std::pair<double, double> point_;
  double dist_;
  std::shared_ptr<Node> node_ = nullptr;
  Nns(std::pair<double, double> point, double dist, std::shared_ptr<Node> node);
  Nns() {
  }
  ;
  ~Nns() {
  }
  ;
};

struct KdTree {
  std::pair<double, double> point_;
  int dim_, axis_;
  std::shared_ptr<Node> node_ = nullptr;
  std::unique_ptr<KdTree> left_ = nullptr;
  std::unique_ptr<KdTree> right_ = nullptr;
  KdTree(std::pair<double, double> point, int axis, std::unique_ptr<KdTree> left, std::unique_ptr<KdTree> right,
         std::shared_ptr<Node> node);
  void insert(std::pair<double, double>& point, std::shared_ptr<Node> node);
  inline double distance(std::pair<double, double> point) {
    return sqrt(
        (this->point_.first - point.first) * (this->point_.first - point.first)
            + (this->point_.second - point.second)
                * (this->point_.second - point.second));
  }
  Nns search(
      std::pair<double, double>& point,
      std::shared_ptr<Node> node,
      std::pair<double, double> ref_point = std::make_pair(std::nan("1"),
                                                           std::nan("1")),
      double distance = std::numeric_limits<double>::infinity());
  ~KdTree() {
  }
};
}  // namespace rrt_planner
#endif
