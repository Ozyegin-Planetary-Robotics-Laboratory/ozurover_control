#include <ros/ros.h>
#include <cstdlib> 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <ozurover_messages/Marker.h>
#include <ozurover_messages/Pathfind.h>
#include <ozurover_messages/Abort.h>
#include <ozurover_messages/GetMarker.h>
#include <ozurover_messages/AddMarker.h>
#include <ozurover_messages/FollowPathAction.h>
#include <ozurover_messages/Steer.h>
#include <ozurover_messages/GPSLocalize.h>
#include "occupancy.hpp"

#include <thread>

#define CONTROL_FREQ 20.0f

enum State {
  IDLE,
  VISITING,
  EXPLORING,
  CONVERGING,
  ABORTING
};

enum LEDState {
  MANUAL,     // BLUE
  AUTONOMOUS, // RED
  SUCCESS     // FLASHING GREEN
};

class StateMachineNode {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<ozurover_messages::FollowPathAction> ac_;
  ros::ServiceClient pc_; // Pathfind client.
  ros::ServiceClient gc_; // Queue client.
  ros::ServiceClient lc_; // GNSS to local coords client.
  ros::ServiceServer as_; // Abort server.
  ros::Subscriber mapSub_;
  ros::Subscriber gnssSub_;
  ros::Subscriber markerSub_;
  ros::Publisher RGBPub_;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  ros::AsyncSpinner _spinner;

  State state_;
  nav_msgs::Path path_;
  ozurover_messages::Marker goal_;
  ozurover_messages::GPS rover_gps_;
  geometry_msgs::PoseStamped rover_pose_;
  std::map<uint16_t, geometry_msgs::PoseStamped> markers_;  
  nav_msgs::OccupancyGrid::ConstPtr obstacle_grid;
  OccupancyUtils occupancyUtils_;

  void setRGB(LEDState s) {
    std_msgs::UInt8MultiArray msg;
    switch (s)
    {
    case AUTONOMOUS:
      msg.data = {0, 1};
      RGBPub_.publish(msg);
      msg.data = {1, 0};
      RGBPub_.publish(msg);
      msg.data = {2, 0};
      RGBPub_.publish(msg);
      break;
    case MANUAL:
      msg.data = {0, 0};
      RGBPub_.publish(msg);
      msg.data = {1, 1};
      RGBPub_.publish(msg);
      msg.data = {2, 0};
      RGBPub_.publish(msg);
      break;
    case SUCCESS:
      msg.data = {0, 0};
      RGBPub_.publish(msg);
      msg.data = {1, 0};
      RGBPub_.publish(msg);
      msg.data = {2, 1};
      RGBPub_.publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      msg.data = {2, 0};
      RGBPub_.publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      msg.data = {2, 1};
      RGBPub_.publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      msg.data = {2, 0};
      RGBPub_.publish(msg);
      break;
    default:
      break;
    }
  }

  void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    rover_gps_.latitude = msg->latitude;
    rover_gps_.longitude = msg->longitude;
    return;
  }

  geometry_msgs::PoseStamped processMarkerPoses(const geometry_msgs::PoseStamped &msg_prev, const geometry_msgs::PoseStamped &msg_new) {
    geometry_msgs::PoseStamped avg;
    avg.pose.position.x = (msg_prev.pose.position.x + msg_new.pose.position.x) / 2.0f;
    avg.pose.position.y = (msg_prev.pose.position.y + msg_new.pose.position.y) / 2.0f;
    avg.pose.position.z = (msg_prev.pose.position.z + msg_new.pose.position.z) / 2.0f;
    return avg;
  }

  void markerCallback(const ozurover_messages::Marker::ConstPtr& msg) {
    markers_[msg->type] = processMarkerPoses(markers_[msg->type], msg->pose);
  }

  bool checkNewTask() {
    ozurover_messages::GetMarker goalN_;
    if (gc_.call(goalN_)) {
      ozurover_messages::GPSLocalize lreq;
      lreq.request.gps.latitude = goalN_.response.gps.latitude;
      lreq.request.gps.longitude = goalN_.response.gps.longitude;
      if (lc_.call(lreq)) {
        goal_.type = goalN_.response.type;
        goal_.pose = lreq.response.pose;  
        return true;
      }
    }
    return false;
  }

  bool checkPathCollisions(const nav_msgs::Path &path) {
    occupancyUtils_.openOccupancyGrid(obstacle_grid);
    return occupancyUtils_.isCollisionCourse(path);
  }

  bool getPathFor(const geometry_msgs::PoseStamped &goal) {
    ozurover_messages::Pathfind pathN;
    pathN.request.rover = rover_pose_;
    pathN.request.goal = goal;
    if (pc_.call(pathN)) {
      path_ = pathN.response.path;
      return true;
    }
    return false;
  }

  bool abortCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ac_.cancelAllGoals();
    state_ = ABORTING;
    return true;
  }

  void visitCurrentGoal() {
    ros::Rate rate(1.0);
    while (!getPathFor(goal_.pose)) rate.sleep();
    ozurover_messages::FollowPathGoal goal;
    goal.path = path_;
    state_ = VISITING;
    ac_.sendGoal(goal);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    obstacle_grid = msg;
  }

  void updateInternalData() {
    /* Update latest rover position by mapping 0, 0, 0 base link to map frame*/
    ros::Rate rate(CONTROL_FREQ);
    while (ros::ok()) {
      rate.sleep();
      try {
        geometry_msgs::TransformStamped transform = _tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        rover_pose_.pose.position.x = transform.transform.translation.x;
        rover_pose_.pose.position.y = transform.transform.translation.y;
        rover_pose_.pose.position.z = transform.transform.translation.z;
        rover_pose_.pose.orientation = transform.transform.rotation;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
    }
    return;
  }

public:
  StateMachineNode() : 
    _tfListener(_tfBuffer),
    ac_("ares/follow_path", true),
    as_(nh_.advertiseService("ares/abort", &StateMachineNode::abortCallback, this)),
    _spinner(2)
  {
    pc_ = nh_.serviceClient<ozurover_messages::Pathfind>("ares/pathfind");
    gc_ = nh_.serviceClient<ozurover_messages::GetMarker>("ares/goal/dequeue");
    lc_ = nh_.serviceClient<ozurover_messages::Abort>("ares/localize");
    RGBPub_ = nh_.advertise<std_msgs::UInt8MultiArray>("ares/led", 1);
    mapSub_ = nh_.subscribe("ares/map/obstacles", 1, &StateMachineNode::mapCallback, this);
    gnssSub_ = nh_.subscribe("ares/gps", 1, &StateMachineNode::gnssCallback, this);
    markerSub_ = nh_.subscribe("ares/goal/marker", 1, &StateMachineNode::markerCallback, this);
    ac_.waitForServer();
    state_ = IDLE;
  }

  void run() {
    _spinner.start();
    std::thread updateThread(&StateMachineNode::updateInternalData, this);
    updateThread.detach();
    while (ros::ok()) {
      switch (state_) {
        case IDLE:
          if (checkNewTask()) {
            state_ = VISITING;
            setRGB(AUTONOMOUS);
            visitCurrentGoal();
          }
          continue;
        case VISITING:
          {
            setRGB(AUTONOMOUS);
            actionlib::SimpleClientGoalState tracerState = ac_.getState();
            switch (tracerState.state_) {
              case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
                if (goal_.type == 1002) { 
                  state_ = IDLE;
                  setRGB(SUCCESS);
                } else {
                  state_ = EXPLORING;
                }
                break;
              case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
                visitCurrentGoal();
                break;
              case actionlib::SimpleClientGoalState::StateEnum::ACTIVE:
                if (checkPathCollisions(path_)) {
                  visitCurrentGoal();
                }
                break;
              default:
                break;
            }
          }
          break;
        case EXPLORING:
          {
            setRGB(AUTONOMOUS);
            actionlib::SimpleClientGoalState tracerState = ac_.getState();
            ros::Rate rate(1.0f);
            while (markers_.find(goal_.type) == markers_.end()) {
              /* Implement exploration protocol here. */
              rate.sleep();
            }
            state_ = CONVERGING;
          }
        case CONVERGING:
          /* Implement convergence protocol. */
          {
            setRGB(AUTONOMOUS);
            actionlib::SimpleClientGoalState tracerState = ac_.getState();
            geometry_msgs::PoseStamped markerGoal = markers_.find(goal_.type)->second;
            ros::Rate rate(5.0f);
          }
          break;
        case ABORTING:
          /* Implement abort protocol. */
          ac_.cancelAllGoals();
          state_ = IDLE;
          break;
        default:
          break;
      }
    }
    return;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_machine");
  StateMachineNode node;
  node.run();
  return 0;
}