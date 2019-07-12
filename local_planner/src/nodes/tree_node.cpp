#include "local_planner/tree_node.h"

namespace avoidance {

TreeNode::TreeNode()
    : total_cost_{0.0f},
      heuristic_{0.0f},
      origin_{0},
      depth_{0},
      closed_{false} {
  position_ = Eigen::Vector3f::Zero();
  velocity_ = Eigen::Vector3f::Zero();
}

TreeNode::TreeNode(int from, int d, const Eigen::Vector3f& pos, const Eigen::Vector3f& vel)
    : total_cost_{0.0f},
      heuristic_{0.0f},
      origin_{from},
      depth_{d},
      closed_{false} {
  position_ = pos;
  velocity_ = vel;
}

void TreeNode::setCosts(float h, float c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() const { return position_; }
Eigen::Vector3f TreeNode::getVelocity() const { return velocity_; }
}
