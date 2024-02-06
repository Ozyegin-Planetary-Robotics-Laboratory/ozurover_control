#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

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
    // void RRT::generateRRT(int iterations);
    double distance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);


    vector<geometry_msgs::PoseStamped> nodes;
    vector<int> parentIndices;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;
    double minX, minY, maxX, maxY;
    double stepSize;
    
private:

};

RRT::RRT(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, double minX, double minY, double maxX, double maxY, double stepSize)
    : start(start), goal(goal), minX(minX), minY(minY), maxX(maxX), maxY(maxY), stepSize(stepSize) {
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
    double x = nearest.pose.position.x + stepSize * cos(angle);
    double y = nearest.pose.position.y + stepSize * sin(angle);

    geometry_msgs::PoseStamped newPose;
    newPose.header.stamp = ros::Time::now();
    newPose.header.frame_id = "map";
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

    while (currentIndex != 0) {
        path.poses.push_back(nodes[currentIndex]);
        currentIndex = parentIndices[currentIndex];
    }

    path.poses.push_back(start);
    reverse(path.poses.begin(), path.poses.end());
    return path;
}


/* void RRT::generateRRT(int iterations) {
    for (int i = 0; i < iterations; ++i) {

        geometry_msgs::PoseStamped randomPoseResult = randomPose();
        int nearestIndex = nearestNeighborIndex(randomPoseResult);
        
        geometry_msgs::PoseStamped nearestNode = nodes[nearestIndex];
        geometry_msgs::PoseStamped newNode = newPose(nearestNode, randomPoseResult);

        if (isCollisionFree(nearestNode, newNode)) {
            nodes.push_back(newNode);
            parentIndices.push_back(nearestIndex);

            // Check if the new node is close to the goal
            if (distance(newNode, goal) < stepSize) {
                // If close to the goal, break the loop
                break;
            }
        }
    }
}
*/
