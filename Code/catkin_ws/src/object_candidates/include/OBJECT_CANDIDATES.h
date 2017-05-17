#ifndef OBJECT_CANDIDATES_H
#define OBJECT_CANDIDATES_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "VOCUS2.h"


class OBJECT_CANDIDATES
{

public:
    // constructor
    OBJECT_CANDIDATES();

private:
    // the VOCUS2 object
    VOCUS2 vocus_;

    // all the handlers for interfacing
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher image_pub_;

    // callback that is called when a new image is published to the topic
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

  #endif // OBJECT_CANDIDATES_H
