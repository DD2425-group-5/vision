#ifndef EVIDENCENODE_HPP
#define EVIDENCENODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <ras_msgs/RAS_Evidence.h>

//#include <r


//std
#include <vector>
#include <string>
#include <sstream>



class EvidenceNode
{
public:
    EvidenceNode(int argc, char* argv[]);
    ros::Subscriber object_subscriber;
    ros::Subscriber rgb_subscriber;
    ros::Publisher sound_publisher;
    ros::Publisher evidence_publisher;
    
private:

    //ros stuff
    void objectCallback(const std_msgs::String::ConstPtr &msg);
    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    sensor_msgs::Image::ConstPtr camera_img_raw;
    ros::Time t_rgb;
    
    //speak
    void speak(std::string object_id);
    


};

#endif // EVIDENCENODE_HPP
