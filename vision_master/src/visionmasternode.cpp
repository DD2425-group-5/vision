#include "visionmasternode.hpp"
/*
void VisionMasterNode::colorCallback(const vision_msgs::colors_detected::ConstPtr &msg) {
    t_color = ros::Time::now();
    colors_found[0] = msg->blue;
    colors_found[1] = msg->green;
    colors_found[2] = msg->red;
    colors_found[3] = msg->yellow;
    colors_found[4] = msg->orange;
    colors_found[5] = msg->purple;
}

void VisionMasterNode::depthCallback(const std_msgs::Bool::ConstPtr &msg) {
    t_depth = ros::Time::now();
    cube_found = msg->data;
}
*/
void VisionMasterNode::allmightyCallback(const vision_msgs::colors_with_shape_info::Ptr& msg) {
    t_all = ros::Time::now();
    p_all = msg;
}

void VisionMasterNode::update() {
    //object ids:
    /*
0 Red Cube
1 Blue Cube
2 Green Cube
3 Yellow Cube
4 Yellow Ball
5 Red Ball
6 Green Cylinder
7 Blue Triangle
8 Purple Cross
9 Patric
      */
    //std::vector<bool> found_this_round(10,false);

    if((ros::Time::now()-t_all).toSec()>0.3) {
        //cube_publisher.publish(msg);
        return;
    }

    //basically go through each color and check if we see something.
    //begin with the colors that have a cube (blue,green,red,yellow)
    for(int i = 0; i < 4; ++i) {
        if(p_all->data[i].color.found) {
            if(p_all->data[i].cube) {
                occurances_in_a_row[color_to_id[i].first] += 1;
                occurances_in_a_row[color_to_id[i].second] -= 5;
            } else {
                occurances_in_a_row[color_to_id[i].second] += 1;
            }
        } else {
            //no such color seen
            occurances_in_a_row[color_to_id[i].first] -= 1;
            occurances_in_a_row[color_to_id[i].second] -= 1;
        }
    }

    //check patric
    if(p_all->data[4].color.found) {
        if(!(p_all->data[4].cube)) {
            occurances_in_a_row[9] += 1;
        }
    } else {
        occurances_in_a_row[9] -= 1;
    }

    //check purple
    if(p_all->data[5].color.found) {
        occurances_in_a_row[8] += 1;
    } else {
        occurances_in_a_row[8] -= 1;
    }

    for(int i = 0; i < occurances_in_a_row.size(); ++i) {
        occurances_in_a_row[i] = std::max(0,occurances_in_a_row[i]);
    }
    
    //ROS_INFO_STREAM("TOPIC NAME: " << master_publisher.getTopic() 
    //                << " Num Subscribers: " << master_publisher.getNumSubscribers());

    for(int i = 0; i < 10; ++i) {
        if(occurances_in_a_row[i] >= occurances_thresh && !objects_found[i]) {
            //a new object was detected.
            int pixel_row = p_all->data[color_mappings[i]].color.row;
            int pixel_col = p_all->data[color_mappings[i]].color.col;
            float depth = p_all->data[color_mappings[i]].color.depth;

            ros::Publisher& p = master_publisher;
            bool hint = false;
            if(i < 8 &&
                    (depth > max_detection_distance ||
                     pixel_row < edge_close_h ||
                     pixel_row > 480 - edge_close_h ||
                     pixel_col < edge_close_w ||
                     pixel_col > 640 - edge_close_w
                ) ) {
                //p = hint_publisher;
                hint = true;
            } else {
                objects_found[i] = true;
                
            }

            vision_msgs::object_found msg;
            msg.id = object_names[i];

            //its position
            std::vector<float> offsets;

            offsets = getRelativePosition(480,640,pixel_row, pixel_col,depth);

            //send message
            msg.offset_x = offsets[0];
            msg.offset_y = offsets[1];
            if(!hint)
            p.publish(msg);
            if(!hint)
            ROS_INFO_STREAM("FOUND OBJECT: " << object_names[i] << " Offsets: ("
                            << offsets[0] << "," << offsets[1] << ")");
            else
            ROS_INFO_STREAM("HINT: " << object_names[i] << " Offsets: ("
                                << offsets[0] << "," << offsets[1] << ")");
        }
    }
}

/**
Input the pixel coordinates where the obejct was found and the distance to it,
and it gives the relative offset in x-y coordinates where the object is in
a 2D space relative to the robot. Used for the mapping part.

Note: For some reason the x axis is positive to the left and negative to the right
in the mapping, so that is how this outputs. I don't know why they did that and
it doesn't make any sense. So don't blame me for that /Dmitrij.

Geometrical explanation for what happens:
Depth to object + camera height gives the ground distance to the object, which
is called d_g in the function (using pythagora's theorem).

Then we calculate w_omega, which is the angle between where the robot's middle front is facing
and the object. Using w_omega and d_g we can use trivial trigonometry do the the distances in
the x and y direction to the object.

*/
std::vector<float> VisionMasterNode::getRelativePosition(
        int camera_res_h,
        int camera_res_w,
        int pixel_row, int pixel_col, float depth) {

    std::vector<float> p(2);
    ROS_INFO_STREAM("pixel_row: " << pixel_row << " pixel_col: " << pixel_col << 
                    "depth: " << depth);
    
    if(depth < 0) {
        depth = 0.4;
    }

    //float h_omega = camera_rotation_x - camera_res_h/2 + ((float)pixel_row)*(camera_fov_h/camera_res_h);
    float w_omega = ((float) pixel_col)*(camera_fov_w/camera_res_w) - camera_fov_w/2;
    //h_omega = std::abs(h_omega);
    //float d_g = std::cos((h_omega*M_PI) / 180)*depth;
    float d_g = std::sqrt(std::pow(depth,2)-std::pow(camera_offset_z,2));
    
    ROS_INFO_STREAM("w_omega: " << w_omega << " d_g: " << d_g);

    w_omega = w_omega*(M_PI/180); //convert to radians

    float epsilon = 1e-5;
    if(std::abs(w_omega) < epsilon) {
        //object straight infront
        p[0] = camera_offset_x;
        p[1] = d_g + camera_offset_y;
        return p;
    }

    bool left = false;
    if(w_omega < 0) {
        left = true;
        w_omega = std::abs(w_omega);
    }

    float x = std::sin(w_omega)*d_g;
    float y = std::cos(w_omega)*d_g;

    if(!left) {
        x = -x;
    }

    x += camera_offset_x;
    y += camera_offset_y;
    p[0] = x;
    p[1] = y;
    return p;
}

void VisionMasterNode::runNode(ros::NodeHandle handle) {
    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
    for(int i = 0; i < 10; ++i)
        loop_rate.sleep();


    while(ros::ok()) {
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void VisionMasterNode::readParams(ros::NodeHandle n) {
    ROSUtil::getParam(n, "robot_info/camera_offset_x", camera_offset_x);
    ROSUtil::getParam(n, "robot_info/camera_offset_y", camera_offset_y);
    ROSUtil::getParam(n, "robot_info/camera_offset_z", camera_offset_z);
    ROSUtil::getParam(n, "robot_info/camera_rotation_x", camera_rotation_x);
    //note below: camera fov v is vertical field of view, and h for horizontal
    //that is from the config file.
    //we save it as fov_w and fov_h for width and height, so its kinda flipped
    //its not a mistake that it is set this way.
    ROSUtil::getParam(n, "robot_info/camera_fov_v", camera_fov_h);
    ROSUtil::getParam(n, "robot_info/camera_fov_h", camera_fov_w);

}

ros::NodeHandle VisionMasterNode::nodeSetup(int argc, char *argv[]) {
    ros::init(argc, argv, "VisionMaster");
    ros::NodeHandle handle;

    //cube_found = false;
    //colors_found = std::vector<bool>(6,false);
    //colors_found = std::vector<vision_msgs::color_status>(6);
    objects_found = std::vector<bool>(10,false);
    occurances_in_a_row = std::vector<int>(10,0);

    occurances_thresh = 10;
    readParams(handle);
    /*
    0 Red Cube
    1 Blue Cube
    2 Green Cube
    3 Yellow Cube
    4 Yellow Ball
    5 Red Ball
    6 Green Cylinder
    7 Blue Triangle
    8 Purple Cross
    9 Patric
*/
    object_names = std::vector<std::string>(10);
    object_names[0] = "Red Cube";
    object_names[1] = "Blue Cube";
    object_names[2] = "Green Cube";
    object_names[3] = "Yellow Cube";
    object_names[4] = "Yellow Ball";
    object_names[5] = "Red Ball";
    object_names[6] = "Green Cylinder";
    object_names[7] = "Blue Triangle";
    object_names[8] = "Purple Cross";
    object_names[9] = "Patric";

    color_mappings = std::vector<int>(10);
    color_mappings[0] = 2;
    color_mappings[1] = 0;
    color_mappings[2] = 1;
    color_mappings[3] = 3;
    color_mappings[4] = 3;
    color_mappings[5] = 2;
    color_mappings[6] = 1;
    color_mappings[7] = 0;
    color_mappings[8] = 5;
    color_mappings[9] = 4;

    //color to id contains mappings from color->IDs of {cube,non-cube} pairs.
    color_to_id = std::vector<std::pair<int,int> >(4);
    color_to_id[0] = std::pair<int,int>(1,7); //blue
    color_to_id[1] = std::pair<int,int>(2,6); //green
    color_to_id[2] = std::pair<int,int>(0,5); //red
    color_to_id[3] = std::pair<int,int>(3,4); //yellow

    max_detection_distance = 0.49;
    edge_close_w = 40;
    edge_close_h = 40;


    //general ros setup
    //t_color = ros::Time::now();
    //t_depth = ros::Time::now();
    //color_subscriber = handle.subscribe("/vision/color_classifier", 1, &VisionMasterNode::colorCallback, this);
    //depth_subscriber = handle.subscribe("/vision/cube_identifier", 1, &VisionMasterNode::depthCallback, this);
    all_subscriber = handle.subscribe("/vision/cube_identifier", 10, &VisionMasterNode::allmightyCallback, this);
    master_publisher = handle.advertise<vision_msgs::object_found>("/vision/detection", 100);
    hint_publisher = handle.advertise<vision_msgs::object_found>("/vision/detection_hint", 100);
    return handle;
}

VisionMasterNode::VisionMasterNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    //hand = nodeSetup(argc, argv);
    runNode(handle);
}

//void ruuun(VisionMasterNode& vmn) {
 //   vmn.runNode(vmn.hand);
//}

int main(int argc, char* argv[]) {
    VisionMasterNode vmn(argc, argv);
    //ruuun(vmn);
}
