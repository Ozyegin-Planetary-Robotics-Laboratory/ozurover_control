#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct Point {
  double x, y;
  Point(double x, double y) : x(x), y(y) {}
};

class RRT {
public:
  RRT(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, double minX, double minY, double maxX, double maxY, double stepSize);
  void generateRRT(int iterations);
  nav_msgs::Path getPath();
  geometry_msgs::PoseStamped randomPose();
  int nearestNeighborIndex(geometry_msgs::PoseStamped target);
  geometry_msgs::PoseStamped newPose(geometry_msgs::PoseStamped nearest, geometry_msgs::PoseStamped target);
  double distance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);
  std::vector<geometry_msgs::PoseStamped> nodes;
  std::vector<int> parentIndices;
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  double minX, minY, maxX, maxY;
  double stepSize;
};

RRT::RRT(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, double minX, double minY, double maxX, double maxY, double stepSize)
  : start(start), goal(goal), minX(std::min(start.pose.position.x, goal.pose.position.x) - 1.0), 
    minY(std::min(start.pose.position.y, goal.pose.position.y) - 1.0), 
    maxX(std::max(start.pose.position.x, goal.pose.position.x) + 1.0), 
    maxY(std::max(start.pose.position.y, goal.pose.position.y) + 1.0), 
    stepSize(stepSize)
    {
    nodes.push_back(start);
    parentIndices.push_back(0);
    srand(time(0));
}

geometry_msgs::PoseStamped RRT::randomPose() {
  double x = minX + static_cast<double>(rand()) / RAND_MAX * (maxX - minX);
  double y = minY + static_cast<double>(rand()) / RAND_MAX * (maxY - minY);

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

int RRT::nearestNeighborIndex(geometry_msgs::PoseStamped target) {
  double minDist = distance(nodes[0], target);
  int minIndex = 0;

  for (int i = 1; i < nodes.size(); ++i) {
    double dist = distance(nodes[i], target);
    if (dist < minDist) {
      minDist = dist;
      minIndex = i;
    }
  }
  return minIndex;
}

geometry_msgs::PoseStamped RRT::newPose(geometry_msgs::PoseStamped nearest, geometry_msgs::PoseStamped target) {
  double angle = atan2(target.pose.position.y - nearest.pose.position.y, target.pose.position.x - nearest.pose.position.x);

  // Interpolate the direction to bias towards the current direction
  double parentAngle = atan2(nearest.pose.position.y - nodes[parentIndices.back()].pose.position.y, nearest.pose.position.x - nodes[parentIndices.back()].pose.position.x);
  angle = 0.8 * angle + 0.2 * parentAngle;
  double x = nearest.pose.position.x + stepSize * cos(angle);
  double y = nearest.pose.position.y + stepSize * sin(angle);

  geometry_msgs::PoseStamped newPose;
  newPose.pose.position.x = x;
  newPose.pose.position.y = y;
  return newPose;
}

double RRT::distance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
  return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
}

nav_msgs::Path RRT::getPath() {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  int currentIndex = nodes.size() - 1;
  path.poses.push_back(goal);
  // Iterate from the goal back to the start
  while (currentIndex != 0) {
    geometry_msgs::PoseStamped current_pose = nodes[currentIndex];
    // Calculates the direction from the current pose to the next pose
    geometry_msgs::PoseStamped next_pose = nodes[parentIndices[currentIndex]];
    double dx = current_pose.pose.position.x - next_pose.pose.position.x;
    double dy = current_pose.pose.position.y - next_pose.pose.position.y;
    double yaw = atan2(dy, dx);
    // Convert yaw to a quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    // Set the orientation of the current pose
    next_pose.pose.orientation.x = quaternion.getX();
    next_pose.pose.orientation.y = quaternion.getY();
    next_pose.pose.orientation.z = quaternion.getZ();
    next_pose.pose.orientation.w = quaternion.getW();
    // Add the current pose to the path
    path.poses.push_back(next_pose);
    currentIndex = parentIndices[currentIndex];
  }
  // Reverse the path to make it start from the beginning
  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}