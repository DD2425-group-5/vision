#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>


void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    ROS_INFO("width,height: [%d,%d]", msg->width, msg->height);
    ROS_INFO("point_step: [%d]", msg->point_step);
    ROS_INFO("row_step: [%d]", msg->row_step);
    //ROS_INFO("enc: [%d]", msg->encoding.c_str());
    ROS_INFO("FIELDs[]:");
    int i = 0;
    ROS_INFO("  name: %s", msg->fields[i].name.c_str());
    ROS_INFO("  offset: %d", msg->fields[i].offset);
    ROS_INFO("  datatype: %d", msg->fields[i].datatype);
    ROS_INFO("  count: %d", msg->fields[i].count);
    
    //std::stringstream ss;
    //for(int j = 12; j < 16; ++j) {
    float f = msg->data[0];
    ROS_INFO("depth: %f", f);
        //ss << (float) msg->data[8] << " ";
    //}
    //ROS_INFO("data: %s",ss.str().c_str());
    
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("width,height: [%d,%d]", msg->width, msg->height);
    ROS_INFO("encoding: [%s]", msg->encoding.c_str());
    ROS_INFO("step: [%d]", msg->step);
    ROS_INFO("data 0,1,2,4: [%d,%d,%d,%d]", msg->data[0],msg->data[1],msg->data[2],msg->data[3]);
    //ROS_INFO("data len: [%d]", sizeof(msg->data)
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "formattester");


  ros::NodeHandle n;


  //ros::Subscriber subd = n.subscribe("/camera/depth/image_rect", 1000, depthCallback);
  ros::Subscriber subi = n.subscribe("/camera/rgb/image_rect_color", 1000, imageCallback);

  ros::spin();

  return 0;
}

