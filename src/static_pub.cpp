#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

int main(int argc, char **argv) {
  nav_msgs::OccupancyGrid::Ptr map = boost::make_shared<nav_msgs::OccupancyGrid>();
  map->header.frame_id = "map";
  map->info.width = 50;
  map->info.height = 50;
  map->info.resolution = 0.1;
  map->info.origin.position.x = 0;
  map->info.origin.position.y = 0;
  map->info.origin.position.z = 0;
  map->info.origin.orientation.x = 0;
  map->info.origin.orientation.y = 0;
  map->info.origin.orientation.z = 0;
  map->info.origin.orientation.w = 1;
  map->data.resize(map->info.width * map->info.height);
  for (int i = 0; i < map->info.width * map->info.height; i++) {
    map->data[i] = 0;
  }

  ros::init(argc, argv, "static_map_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("ares/map/obstacles", 10);
  ros::Rate loop_rate(0.5);

  while (ros::ok()) {
    map->header.stamp = ros::Time::now();
    pub.publish(map);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
