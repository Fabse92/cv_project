#ifndef LISTENER_H
#define LISTENER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "object_candidates/ArrayImages.h"

class LISTENER
{

public:
    // constructor
    LISTENER();

private:

    // all the handlers for interfacing
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber sub;

    // callback that is called when a new image is published to the topic
    void imageCb(const object_candidates::ArrayImages msg);
};

  #endif // LISTENER_H
