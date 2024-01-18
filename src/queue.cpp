#include <ros/ros.h>
#include <ozurover_messages/AddMarker.h>
#include <ozurover_messages/GetMarker.h>
#include <ozurover_messages/Abort.h>
#include <vector>
#include <deque>

struct MarkerElement {
    /* Fill this place up with the relevant data*/
};

class QueueNode {
private:
    std::deque<MarkerElement> queue;
    std::vector<MarkerElement> history;
    ros::NodeHandle nh;
    ros::ServiceServer abort_service;
    ros::ServiceServer queue_service;
    ros::ServiceServer dequeue_service;
    ros::ServiceServer h_push_service;
    ros::ServiceServer h_pop_service;
    
    bool abort(ozurover_messages::Abort::Request &req, ozurover_messages::Abort::Response &res) {
        return true;
    }

    bool enqueue(ozurover_messages::AddMarker::Request &req, ozurover_messages::AddMarker::Response &res) {
        return true;
    }
    bool dequeue(ozurover_messages::GetMarker::Request &req, ozurover_messages::GetMarker::Response &res) {
        return true;
    }

    bool h_push(ozurover_messages::AddMarker::Request &req, ozurover_messages::AddMarker::Response &res) {
        return true;
    }

    bool h_pop(ozurover_messages::GetMarker::Request &req, ozurover_messages::GetMarker::Response &res) {
        return true;
    }


public:
    QueueNode() {
        /* Initialize services*/
    }

    void init() {
        /* Initialize the queue and do ros::spin()*/
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "queue_node");
    QueueNode queue_node;
    queue_node.init();
    return 0;
}