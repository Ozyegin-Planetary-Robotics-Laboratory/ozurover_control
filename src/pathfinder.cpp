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
    double radius = 0.3;
    // Convert continuous world coordinates to grid indices
    double map_x = (pose.pose.position.x - obstacle_grid->info.origin.position.x) / obstacle_grid->info.resolution;
    double map_y = (pose.pose.position.y - obstacle_grid->info.origin.position.y) / obstacle_grid->info.resolution;
    int grid_x = static_cast<int>(map_x);
    int grid_y = static_cast<int>(map_y);
    // Check each cell in a square around the pose within the given radius
    int radius_cells = static_cast<int>(radius / obstacle_grid->info.resolution);
    int min_x = std::max(0, grid_x - radius_cells);
    int max_x = std::min(static_cast<int>(obstacle_grid->info.width) - 1, grid_x + radius_cells);
    int min_y = std::max(0, grid_y - radius_cells);
    int max_y = std::min(static_cast<int>(obstacle_grid->info.height) - 1, grid_y + radius_cells);
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        int index = y * obstacle_grid->info.width + x;
        if (obstacle_grid->data[index] > 0) {
          // Obstacle detected within the radius
          ROS_WARN("Collision detected within radius! X: %f, Y: %f", pose.pose.position.x, pose.pose.position.y);
          return true;
        }
      }
  }
  // No collision within the radius
  return false;
}

geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::PoseStamped& next_pose) {
  // Calculate the direction from the current pose to the next pose
  double dx = next_pose.pose.position.x - current_pose.pose.position.x;
  double dy = next_pose.pose.position.y - current_pose.pose.position.y;
  double yaw = atan2(dy, dx);
  // Convert yaw to a quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

nav_msgs::Path smoothPath(const nav_msgs::Path& original_path, int num_iterations) {
  nav_msgs::Path smoothed_path;
  smoothed_path.header = original_path.header;
  smoothed_path.poses = original_path.poses;

  int num_poses = original_path.poses.size();    
  for (int iter = 0; iter < num_iterations; ++iter) {
    // Temporary copy of the smoothed path
    nav_msgs::Path temp_path = smoothed_path;
    // Iterate through each pose except the first and last
    for (int i = 0; i < num_poses - 1; ++i) {
      if(i == 0) {
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.orientation = calculateOrientation(temp_path.poses[i], temp_path.poses[i + 1]);
        smoothed_path.poses[i] = new_pose;
      } 
      else {
       // Average the positions of neighboring waypoints
       geometry_msgs::PoseStamped new_pose;
       new_pose.pose.position.x = (temp_path.poses[i-1].pose.position.x+temp_path.poses[i].pose.position.x + temp_path.poses[i + 1].pose.position.x) / 3.0;
       new_pose.pose.position.y = (temp_path.poses[i-1].pose.position.y+temp_path.poses[i].pose.position.y+temp_path.poses[i + 1].pose.position.y) / 30;
       // Calculate orientation based on the direction to the next pose
       new_pose.pose.orientation = calculateOrientation(temp_path.poses[i], temp_path.poses[i + 1]);
       // Check collision at the new position
       if (!isCollisionDetected(new_pose)) {
        // If no collision, update the position and orientation
        smoothed_path.poses[i] = new_pose;
       }
        // Else, leave the pose unchanged
        }
      }    
    }
    return smoothed_path;
  }

  bool pathfindServiceCallback(ozurover_messages::Pathfind::Request& req, ozurover_messages::Pathfind::Response& res) {
    RRT rrt(req.rover, req.goal, -30, -30, 30, 30, 0.5);
    for (int i = 0; i < 15000; ++i) {
      geometry_msgs::PoseStamped randomPoseResult = rrt.randomPose();
      int nearestIndex = rrt.nearestNeighborIndex(randomPoseResult);
      geometry_msgs::PoseStamped nearestNode = rrt.nodes[nearestIndex];
      geometry_msgs::PoseStamped newNode = rrt.newPose(nearestNode, randomPoseResult);

      double angle = atan2(newNode.pose.position.y - nearestNode.pose.position.y,newNode.pose.position.x - nearestNode.pose.position.x);
      // Check if the angle is within acceptable bounds (e.g., less than 60 degrees)
      const double maxAngle = 60.0 * M_PI / 180.0; // Convert 60 degrees to radians
      if (!isCollisionDetected(newNode)) {   
        // Check if the new node is close to the goal 
        if (rrt.distance(newNode, rrt.goal) < 0.3) {
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
    nav_msgs::Path smoothPathh;
    smoothPathh.header.stamp = ros::Time::now();
    smoothPathh.header.frame_id = "map";
    smoothPathh = smoothPath(rrt.getPath(), 6000);
    res.path = smoothPathh;
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