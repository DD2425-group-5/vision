#ifndef VISIONMASTERNODE_HPP
#define VISIONMASTERNODE_HPP

#include <ros/ros.h>

//messages
#include <color_detection/colors_detected.h>
#include <color_detection/color_status.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vision_master/object_found.h>

//#include <visionutil/geometry.hpp>
#include <rosutil/rosutil.hpp>

#include <opencv2/core/core.hpp>


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
    ros::Publisher hint_publisher;

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
    std::vector<color_detection::color_status> colors_found;

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
    std::vector<int> color_mappings;

    std::vector<float> getRelativePosition(int camera_res_h,
                                         int camera_res_w,
                                         int pixel_row, int pixel_col, float depth);

    //parameters
    int occurances_thresh;

    void readParams(ros::NodeHandle);
    float camera_offset_x;
    float camera_offset_y;
    float camera_offset_z;
    float camera_rotation_x;
    float camera_fov_h;
    float camera_fov_w;

    //refinement
    float max_detection_distance;
    int edge_close_w;
    int edge_close_h;

};

#endif // VISIONMASTERNODE_HPP
