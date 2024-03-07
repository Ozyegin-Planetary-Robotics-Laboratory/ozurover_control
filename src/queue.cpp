#include <ros/ros.h>
#include <ozurover_messages/AddMarker.h>
#include <ozurover_messages/GetMarker.h>
#include <ozurover_messages/Abort.h>
#include <ozurover_messages/GPS.h>
#include <vector>
#include <deque>

struct MarkerElement {
  uint16_t type;
  ozurover_messages::GPS gps;
};

class QueueNode {
protected:
  std::deque<MarkerElement> queue;
  std::vector<MarkerElement> history;
  ros::NodeHandle nh;
  ros::ServiceServer abort_service;
  ros::ServiceServer queue_service;
  ros::ServiceServer dequeue_service;
  ros::ServiceServer h_push_service;
  ros::ServiceServer h_pop_service;

  bool abort(ozurover_messages::Abort::Request &req, ozurover_messages::Abort::Response &res) {
    queue.clear();
    if (!history.empty()) {
      // Revert to the last position in history
      MarkerElement last_position = history.back();
      queue.push_back(last_position);
      // ?Remove the last position from history?
      history.pop_back();
    }
    return true;
  }

  bool enqueue(ozurover_messages::AddMarker::Request &req, ozurover_messages::AddMarker::Response &res) {
    MarkerElement new_element;
    new_element.type = req.type;
    new_element.gps.latitude = req.gps.latitude;
    new_element.gps.longitude = req.gps.longitude;
    queue.push_back(new_element);
    return true;
  }

  bool dequeue(ozurover_messages::GetMarker::Request &req, ozurover_messages::GetMarker::Response &res) {
    if (!queue.empty()) {
      res.type = queue.front().type;
      res.gps = queue.front().gps;
      queue.pop_front();
      return true;
    } else {
      return false;
    }
  }

  bool h_push(ozurover_messages::AddMarker::Request &req, ozurover_messages::AddMarker::Response &res) {
      MarkerElement new_element;
      new_element.type = req.type;
      new_element.gps.latitude = req.gps.latitude;
      new_element.gps.longitude = req.gps.longitude;

      history.push_back(new_element);

      return true;
  }

  bool h_pop(ozurover_messages::GetMarker::Request &req, ozurover_messages::GetMarker::Response &res) {
    if (!history.empty()) {
      res.type = history.back().type;
      res.gps = history.back().gps;
      
      history.pop_back(); //Because we want the last element which succesfully achieved by rover.
      return true;
    } else {
      return false;
    }
  }

public:
  QueueNode() {
    abort_service = nh.advertiseService("ares/goal/abort", &QueueNode::abort, this);
    queue_service = nh.advertiseService("ares/goal/enqueue", &QueueNode::enqueue, this);
    dequeue_service = nh.advertiseService("ares/goal/dequeue", &QueueNode::dequeue, this);
    h_push_service = nh.advertiseService("ares/goal/history_push", &QueueNode::h_push, this);
    h_pop_service = nh.advertiseService("ares/goal/history_pop", &QueueNode::h_pop, this);
  }

  void init() {
    ros::spin();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "queue_node");
  QueueNode queue_node;
  queue_node.init();
  return 0;
}