#include <ros/ros.h>
#include <ozurover_messages/Enqueue.h>
#include <ozurover_messages/Dequeue.h>
#include <deque>

struct QueueNode {
    /* Fill this place up with the relevant data*/
}

class QueueNode {
private:
    std::deque<QueueNode> queue;
    ros::NodeHandle nh;
    ros::ServiceServer abort_service;
    ros::ServiceServer queue_service;
    ros::ServiceServer dequeue_service;
    
    bool abort(ozurover_messages::Abort::Request &req, ozurover_messages::Abort::Response &res) {
        return true;
    }

    bool enqueue(ozurover_messages::Enqueue::Request &req, ozurover_messages::Enqueue::Response &res) {
        return true;
    }
    bool dequeue(ozurover_messages::Dequeue::Request &req, ozurover_messages::Dequeue::Response &res) {
        return true;
    }
public:
    QueueNode() {
        /* Initialize services*/
    }

    init() {
        /* Initialize the queue and do ros::spin()*/
    }

}

int main(int argc, char **argv) {
    ros.init(argc, argv, "queue_node");
    QueueNode queue_node;
    queue_node.init();
    return 0;
}