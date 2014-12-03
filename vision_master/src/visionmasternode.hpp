#ifndef VISIONMASTERNODE_HPP
#define VISIONMASTERNODE_HPP

#include <ros/ros.h>

//messages
//#include <vision_msgs/colors_detected.h>
#include <vision_msgs/color_status.h>
#include <vision_msgs/colors_with_shape_info.h>
#include <vision_msgs/color_and_cube.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vision_msgs/object_found.h>

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

    //ros::Subscriber color_subscriber;
    //ros::Subscriber depth_subscriber;
    ros::Subscriber all_subscriber;
    ros::Publisher master_publisher;
    ros::Publisher hint_publisher;
    //ros::NodeHandle hand;
    
    void runNode(ros::NodeHandle handle);

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
    //ros::Time t_color;
    //ros::Time t_depth;
    //bool cube_found;
    //std::vector<vision_msgs::color_status> colors_found;
    ros::Time t_all;
    vision_msgs::colors_with_shape_info::Ptr p_all;

    //ros stuff
    //void colorCallback(const vision_msgs::colors_detected::ConstPtr &msg);
    //void depthCallback(const std_msgs::Bool::ConstPtr &msg);
    void allmightyCallback(const vision_msgs::colors_with_shape_info::Ptr& msg);
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    
    void update();

    //objects found
    std::vector<bool> objects_found;
    void objectDetected(int id);
    std::vector<int> occurances_in_a_row;
    std::vector<std::string> object_names;
    std::vector<int> color_mappings;
    std::vector<std::pair<int,int> > color_to_id;

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
