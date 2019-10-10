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
#include "rrt_turtlebot/kdTree.h"

namespace rrt_planner {

Nns::Nns(std::pair<double, double> point, double dist,std::shared_ptr<Node> node) {
  this->point_ = point;
  this->dist_ = dist;
  this->node_ = node;
}

KdTree::KdTree(std::pair<double, double> point, int axis, std::unique_ptr<KdTree> left,
               std::unique_ptr<KdTree> right,std::shared_ptr<Node> node) {
  this->point_ = point;
  this->axis_ = axis;
  this->left_ = std::move(left);
  this->right_ = std::move(right);
  this->node_ = node;
}

void KdTree::insert(std::pair<double, double>& point,std::shared_ptr<Node> node) {
  if (!this->axis_) {
    if (this->point_.first > point.first) {
      if (!this->left_) {
        this->left_ = std::make_unique<KdTree>(point, (this->axis_ + 1) % 2, nullptr, nullptr,
                                 node);
      } else {
        this->left_->insert(point, node);
      }
    } else {
      if (!this->right_) {
        this->right_ = std::make_unique<KdTree>(point, (this->axis_ + 1) % 2, nullptr,
                                  nullptr, node);
      } else {
        this->right_->insert(point, node);
      }
    }
  } else {
    if (this->point_.second > point.second) {
      if (!this->left_) {
        this->left_ = std::make_unique<KdTree>(point, (this->axis_ + 1) % 2, nullptr, nullptr,
                                 node);
      } else {
        this->left_->insert(point, node);
      }
    } else {
      if (!this->right_) {
        this->right_ = std::make_unique<KdTree>(point, (this->axis_ + 1) % 2, nullptr,
                                  nullptr, node);
      } else {
        this->right_->insert(point, node);
      }
    }
  }
}

Nns KdTree::search(std::pair<double, double>& point,std::shared_ptr<Node> node,
                   std::pair<double, double> ref_point, double dist) {
  if (!this->left_ && !this->right_) {
    double w = this->distance(point);
    if (w < dist) {
      return Nns(this->point_, w, this->node_);
    } else {
      return Nns(ref_point, dist, node);
    }
  } else {
    double d = this->distance(point);
    if (d < dist) {
      dist = d;
      ref_point = this->point_;
      node = this->node_;
    }
    if (!this->axis_) {
      if (point.first <= this->point_.first) {
        if (point.first - dist <= this->point_.first) {
          if (this->left_) {
            Nns res = this->left_->search(point, node, ref_point, dist);
            ref_point = res.point_;
            node = res.node_;
            dist = res.dist_;
          }
        }
        if (point.first + dist > this->point_.first) {
          if (this->right_) {
            return this->right_->search(point, node, ref_point, dist);
          }
        }
      } else {
        if (point.first + dist > this->point_.first) {
          if (this->right_) {
            Nns res = this->right_->search(point, node, ref_point, dist);
            ref_point = res.point_;
            node = res.node_;
            dist = res.dist_;
          }
        }
        if (point.first - dist <= this->point_.first) {
          if (this->left_) {
            return this->left_->search(point, node, ref_point, dist);
          }
        }
      }
    } else {
      if (point.second <= this->point_.second) {
        if (point.second - dist <= this->point_.second) {
          if (this->left_) {
            Nns res = this->left_->search(point, node, ref_point, dist);
            ref_point = res.point_;
            node = res.node_;
            dist = res.dist_;
          }
        }
        if (point.second + dist > this->point_.second) {
          if (this->right_) {
            return this->right_->search(point, node, ref_point, dist);
          }
        }
      } else {
        if (point.second + dist > this->point_.second) {
          if (this->right_) {
            Nns res = this->right_->search(point, node, ref_point, dist);
            ref_point = res.point_;
            node = res.node_;
            dist = res.dist_;
          }
        }
        if (point.second - dist <= this->point_.second) {
          if (this->left_) {
            return this->left_->search(point, node, ref_point, dist);
          }
        }
      }
    }
    return Nns(ref_point, dist, node);
  }
}
}  // namespace rrt_planner
