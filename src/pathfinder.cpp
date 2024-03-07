#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ozurover_messages/Pathfind.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../include/rrt.h"


class PathfindingNode {
private:
  ros::NodeHandle nh;
  ros::Subscriber obstacle_map_sub_;
  ros::ServiceServer pathfinding_service;
  nav_msgs::OccupancyGrid obstacle_map_;
  nav_msgs::OccupancyGrid::ConstPtr obstacle_grid;

  void obstacleGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg) {
    obstacle_grid = grid_msg;
  }

  bool isCollisionDetected(const geometry_msgs::PoseStamped& pose) {   
    // Convert continuous world coordinates to grid indices
    double map_x = (pose.pose.position.x - obstacle_grid->info.origin.position.x) / obstacle_grid->info.resolution;
    double map_y = (pose.pose.position.y - obstacle_grid->info.origin.position.y) / obstacle_grid->info.resolution;

    int grid_x = static_cast<int>(map_x);
    int grid_y = static_cast<int>(map_y);

    if (grid_x >= 0 && grid_x < obstacle_grid->info.width && grid_y >= 0 && grid_y < obstacle_grid->info.height) {
      int index = grid_y * obstacle_grid->info.width + grid_x;
      if (obstacle_grid->data[index] > 0) {
        // Obstacle detected
        ROS_WARN("Collision detected! X: %f, Y: %f, GridX: %d, GridY: %d, Index: %d, Value: %d",
        pose.pose.position.x, pose.pose.position.y, grid_x, grid_y, index, obstacle_grid->data[index]);
        return true;
      }
    }
    // No collision
    return false;
  }
    
  bool pathfindServiceCallback(ozurover_messages::Pathfind::Request& req, ozurover_messages::Pathfind::Response& res) {
    RRT rrt(req.rover, req.goal, -30, -30, 30, 30, 0.3);
    for (int i = 0; i < 30000; ++i) {
      geometry_msgs::PoseStamped randomPoseResult = rrt.randomPose();
      int nearestIndex = rrt.nearestNeighborIndex(randomPoseResult);
      geometry_msgs::PoseStamped nearestNode = rrt.nodes[nearestIndex];
      geometry_msgs::PoseStamped newNode = rrt.newPose(nearestNode, randomPoseResult);
      if (!isCollisionDetected(newNode)) {   
        // Check if the new node is close to the goal 
        if (rrt.distance(newNode, rrt.goal) < rrt.stepSize) {
          int nearestIndex = rrt.nearestNeighborIndex(req.goal);
          rrt.nodes.push_back(req.goal);
          rrt.parentIndices.push_back(nearestIndex);

          // If close to the goal, break the loop
          break;
        }
        rrt.nodes.push_back(newNode);
        rrt.parentIndices.push_back(nearestIndex);      
      }
    }
    res.path = rrt.getPath();
    return true;
  }

public:
  PathfindingNode() {
    obstacle_map_sub_ = nh.subscribe("ares/map/obstacles", 1, &PathfindingNode::obstacleGridCallback, this);
    pathfinding_service = nh.advertiseService("ares/pathfind", &PathfindingNode::pathfindServiceCallback, this);
  }

  void init() {
      ros::spin();
  } 
}; // class PathfindingNode

int main(int argc, char** argv) {
  ros::init(argc, argv, "pathfinding_node");
  PathfindingNode pathfinding_node;
  pathfinding_node.init();
  return 0;
}