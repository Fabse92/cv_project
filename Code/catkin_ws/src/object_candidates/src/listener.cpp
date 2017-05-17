#include "listener.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "object_candidates/ArrayImages.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <sstream>

LISTENER::LISTENER() : it_(nh_)
{
    image_pub_ = it_.advertise("/candidate",1000);
    sub = nh_.subscribe("/object_candidates", 1000, &LISTENER::imageCb, this);
}

void LISTENER::imageCb(const object_candidates::ArrayImages msg)
{      
    //std::cout << "hi";
    
    //for (unsigned i = 0; i < msg.data.size(); ++i){
    //  std::cout << "hi";
    //}
    ros::Rate loop_rate(5);
    while (nh_.ok()) {
      image_pub_.publish(msg.data[0]);
      ros::spinOnce();
      loop_rate.sleep();
    }
}



