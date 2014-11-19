#ifndef VISIONMASTERNODE_HPP
#define VISIONMASTERNODE_HPP

#include <ros/ros.h>

//messages
#include <color_detection/colors_detected.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


//std
#include <vector>
#include <string>



class VisionMasterNode
{
public:
    VisionMasterNode(int argc, char* argv[]);

    ros::Subscriber color_subscriber;
    ros::Subscriber depth_subscriber;
    ros::Publisher master_publisher;

    enum object_ids { Red_Cube,
                   Blue_Cube,
                   Green_Cube,
                   Yellow_Cube,
                   Yellow_Ball,
                   Red_Ball,
                   Green_Cylinder,
                   Blue_Triangle,
                   Purple_Cross,
                   Patric };


    /*
    enum objects { "Red Cube",
                   "Blue Cube",
                   "Green Cube",
                   "Yellow Cube",
                   "Yellow Ball",
                   "Red Ball",
                   "Green Cylinder",
                   "Blue Triangle",
                   "Purple Cross",
                   "Patric" };*/

private:
    //messages
    ros::Time t_color;
    ros::Time t_depth;
    bool cube_found;
    std::vector<bool> colors_found;

    //ros stuff
    void colorCallback(const color_detection::colors_detected::ConstPtr &msg);
    void depthCallback(const std_msgs::Bool::ConstPtr &msg);
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
    void update();

    //objects found
    std::vector<bool> objects_found;
    void objectDetected(int id);
    std::vector<int> occurances_in_a_row;
    std::vector<std::string> object_names;

    //parameters
    int occurances_thresh;

};

#endif // VISIONMASTERNODE_HPP
