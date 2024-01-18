#include <ros/ros.h>
#include <ozurover_messages/Enqueue.h>
#include <ozurover_messages/Dequeue.h>
#include <ozurover_messages/Abort.h>
#include <deque>

struct QueueElement {
    /* Fill this place up with the relevant data*/
};

class QueueNode {
private:
    std::deque<QueueElement> queue;
    std::deque<QueueElement> history;
    ros::NodeHandle nh;
    ros::ServiceServer abort_service;
    ros::ServiceServer queue_service;
    ros::ServiceServer dequeue_service;
    ros::ServiceServer h_queue_service;
    ros::ServiceServer h_dequeue_service;
    
    bool abort(ozurover_messages::Abort::Request &req, ozurover_messages::Abort::Response &res) {
        return true;
    }

    bool enqueue(ozurover_messages::Enqueue::Request &req, ozurover_messages::Enqueue::Response &res) {
        return true;
    }
    bool dequeue(ozurover_messages::Dequeue::Request &req, ozurover_messages::Dequeue::Response &res) {
        return true;
    }

    bool h_queue_service(ozurover_messages::Dequeue::Request &req, ozurover_messages::Dequeue::Response &res) {
        return true;
    }

    bool h_dequeue_service(ozurover_messages::Dequeue::Request &req, ozurover_messages::Dequeue::Response &res) {
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