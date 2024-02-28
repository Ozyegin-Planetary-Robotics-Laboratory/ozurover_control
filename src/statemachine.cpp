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

#include <thread>

#define CONTROL_FREQ 20.0f

enum State {
  IDLE,
  VISITING,
  EXPLORING,
  CONVERGING,
  ABORTING
};

enum LED {
  RED = 398,
  GREEN = 298,
  BLUE = 389
};

class LightController {
protected:
  bool redState_;
  bool greenState_;
  bool blueState_;
public:
  LightController() :
    redState_(false),
    greenState_(false),
    blueState_(false)
  {
    /* Echo out direction to the gpio pins in /sys/class/gpio/ after export*/
    system("echo 398 > /sys/class/gpio/export");
    system("echo 298 > /sys/class/gpio/export");
    system("echo 389 > /sys/class/gpio/export");

    /* Set direction to out */
    system("echo out > /sys/class/gpio/gpio398/direction");
    system("echo out > /sys/class/gpio/gpio298/direction");
    system("echo out > /sys/class/gpio/gpio389/direction");

    /* Set initial state to off */
    closeLight(RED);
    closeLight(GREEN);
    closeLight(BLUE);
  }

  void setLight(LED id) {
    switch (id) {
      case RED:
        system("echo 1 > /sys/class/gpio/gpio398/value");
        redState_ = true;
        break;
      case GREEN:
        system("echo 1 > /sys/class/gpio/gpio298/value");
        greenState_ = true;
        break;
      case BLUE:
        system("echo 1 > /sys/class/gpio/gpio389/value");
        blueState_ = true;
        break;
      default:
        break;
    }
  }

  void closeLight(LED id) {
    switch (id) {
      case RED:
        system("echo 0 > /sys/class/gpio/gpio398/value");
        redState_ = false;
        break;
      case GREEN:
        system("echo 0 > /sys/class/gpio/gpio298/value");
        greenState_ = false;
        break;
      case BLUE:
        system("echo 0 > /sys/class/gpio/gpio389/value");
        blueState_ = false;
        break;
      default:
        break;
    }
  }
};

class StateMachineNode {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<ozurover_messages::FollowPathAction> ac_;
  ros::ServiceClient pc_; // Pathfind client.
  ros::ServiceClient gc_; // Queue client.
  ros::ServiceClient lc_; // GNSS to local coords client.
  ros::ServiceServer as_; // Abort server.
  ros::Subscriber gnssSub_;
  ros::Subscriber markerSub_;
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  ros::AsyncSpinner _spinner;

  State state_;
  nav_msgs::Path path_;
  ozurover_messages::Marker goal_;
  ozurover_messages::GPS rover_gps_;
  geometry_messages::PoseStamped rover_pose_;
  std::map<uint16_t, geometry_msgs::PoseStamped> markers_;  

  void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    rover_gps_.latitude = msg->latitude;
    rover_gps_.longitude = msg->longitude;
    return;
  }

  geometry_msgs::PoseStamped processMarkerPoses(const geometry_msgs::PoseStamped &msg_prev, const geometry_msgs::PoseStamped &msg_new) {
    if (markers_.find(msg->id) != markers_.end()) {
      geometry_msgs::PoseStamped avg;
      avg.pose.position.x = (msg_prev.pose.position.x + msg_new.pose.position.x) / 2.0f;
      avg.pose.position.y = (msg_prev.pose.position.y + msg_new.pose.position.y) / 2.0f;
      avg.pose.position.z = (msg_prev.pose.position.z + msg_new.pose.position.z) / 2.0f;
      markers_[msg->id] = avg;
    }
    return markers_[msg->id] = msg->pose;
  }

  void markerCallback(const ozurover_messages::Marker::ConstPtr& msg) {
    markers_[msg->id] = processMarkerPoses(markers_[msg->id], msg->pose);
  }

  bool checkNewTask() {
    ozurover_messages::GetMarker goalN;
    if (gc_.call(goalN_)) {
      ozurover_messages::GPSLocalize lreq;
      lreq.reuest.gps.latitude = goalN.response.gps.latitude;
      lreq.reuest.gps.longitude = goalN.response.gps.longitude;
      if (lc_.call(lreq);) {
        goal_ = lreq.response.marker;  
        return true;
      }
    }
    return false;
  }

  bool checkPathCollisions(const nav_msgs::Path &path) {
    /* Implement path collision checking here. */

    return false;
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
    while (!getPathFor(goal_)) rate.sleep();
    ozurover_messages::FollowPathGoal goal;
    goal.path = path_;
    state_ = VISITING;
    ac_.sendGoal(goal);
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
  {
    pc_ = nh_.serviceClient<ozurover_messages::Pathfind>("ares/pathfind");
    gc_ = nh_.serviceClient<ozurover_messages::GetMarker>("ares/goal/dequeue");
    lc_ = nh_.serviceClient<ozurover_messages::Abort>("ares/localize");
    gnssSub_ = nh_.subscribe("ares/gps", 1, &StateMachineNode::gnssCallback, this);
    markerSub_ = nh_.subscribe("ares/goal/marker", 1, &StateMachineNode::markerCallback, this);
    ac_.waitForServer();
    state_ = IDLE;
  }

  void run() {
    /* Initialize other threads.*/
    _spinner.start();
    std::thread updateThread(&StateMachineNode::updateInternalData, this);
    updateThread.detach();
    while (ros::ok()) {
      switch (state_) {
        case IDLE:
          if (checkNewTask()) {
            state_ = VISITING;
            visitCurrentGoal();
          }
          continue;
        case VISITING: // Moving towards the given coordinates
          actionlib::SimpleClientGoalState::StateEnum tracerState = ac_.getState();
          switch (tracerState){
            case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
              if (goal.id == 1002) { 
                state_ = IDLE; // If goal has no visual indicator, wrap up.
              } else {
                state_ = EXPLORING; // If goal has visual indicator, start exploring.
              }
              break;
            case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
              visitCurrentGoal(); // Retry visiting
              break;
            case: actionlib::SimpleClientGoalState::StateEnum::ACTIVE:
              if ();
              break;
            default:
              break;
          }
          break;
        case EXPLORING: // Roaming around visited area until marker is found.
          actionlib::SimpleClientGoalState::StateEnum tracerState = ac_.getState();
          ros::Rate rate(1.0f);
          while (markers_.find(goal_.id) == markers_.end()) {
            /* Implement protocol here. */
            rate.sleep();
          }
          state_ = CONVERGING;
        case CONVERGING:
          /* Implement convergence protocol. */
          actionlib::SimpleClientGoalState::StateEnum tracerState = ac_.getState();
          geometry_msgs::PoseStamped markerGoal = markers_[goal_.id].pose;
          ros::Rate rate(5.0f);
          while (ros::ok()) {
            tracerState = ac_.getState();
            if (ac_.isPreemptRequested()) {
              ac_.cancelAllGoals();
              state_ = ABORTING;
              break;
            }
            if (state_ == ABORTING) break;
            if (getPathFor(markerGoal)) {
              ozurover_messages::FollowPathGoal goal;
              goal.path = path_;
              ac_.sendGoal(goal);
              break;
            }
            rate.sleep();
          }
          break;
        case ABORTING:
          /* Implement abort protocol. */
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