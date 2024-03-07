#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <array>


class OccupancyUtils
{
private:
  typedef std::array<int, 2> Point;
  typedef std::vector<Point> Line;
  nav_msgs::OccupancyGrid::Ptr obstacle_grid;
  size_t getIndex_(const geometry_msgs::PoseStamped &pose);
  size_t getIndex_(const int &i1, const int &i2);
  Point getIndices_(const geometry_msgs::PoseStamped &pose);
  Line getLine_(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);
  Line getLine_(const Point &p1, const Point &p2);
  bool isBlocked_(const Line &line);
public:
  OccupancyUtils(/* args */);
  ~OccupancyUtils();
  void openOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);
  bool isCollisionCourse(const nav_msgs::Path &path);
};

OccupancyUtils::OccupancyUtils(/* args */)
{
}

OccupancyUtils::~OccupancyUtils()
{
}

void OccupancyUtils::openOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  obstacle_grid = grid_msg;
}

bool OccupancyUtils::isCollisionCourse(const nav_msgs::Path &path) {
  for (size_t i = 0; i < path.poses.size() - 1; i++) {
    if (isBlocked_(getLine_(path.poses[i], path.poses[i + 1]))) {
      return true;
    }
  }
  return false;
}

size_t OccupancyUtils::getIndex_(const geometry_msgs::PoseStamped &pose) {
  Point p = getIndices_(pose);
  return getIndex_(p[0], p[1]);
}

size_t OccupancyUtils::getIndex_(const int &i1, const int &i2) {
  return i2 * obstacle_grid->info.width + i1;
}

OccupancyUtils::Point OccupancyUtils::getIndices_(const geometry_msgs::PoseStamped &pose) {
  Point p;
  p[0] = (pose.pose.position.x - obstacle_grid->info.origin.position.x) / obstacle_grid->info.resolution;
  p[1] = (pose.pose.position.y - obstacle_grid->info.origin.position.y) / obstacle_grid->info.resolution;
  return p;
}

OccupancyUtils::Line OccupancyUtils::getLine_(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2) {
  Line line;
  Point p1 = getIndices_(pose1);
  Point p2 = getIndices_(pose2);
  return getLine_(p1, p2);
}

OccupancyUtils::Line OccupancyUtils::getLine_(const Point &p1, const Point &p2) {
  Line line;
  int x1 = p1[0];
  int y1 = p1[1];
  int x2 = p2[0];
  int y2 = p2[1];
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = x1 < x2 ? 1 : -1;
  int sy = y1 < y2 ? 1 : -1;
  int err = dx - dy;
  while (true) {
    line.push_back({x1, y1});
    if (x1 == x2 && y1 == y2) {
      break;
    }
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y1 += sy;
    }
  }
  return line;
}

bool OccupancyUtils::isBlocked_(const Line &line) {
  for (const auto &p : line) {
    if (obstacle_grid->data[getIndex_(p[0], p[1])] > 0) {
      return true;
    }
  }
  return false;
}

