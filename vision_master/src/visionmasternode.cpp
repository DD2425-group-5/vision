#include "visionmasternode.hpp"

void VisionMasterNode::colorCallback(const color_detection::colors_detected::ConstPtr &msg) {
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

    std::vector<bool> found_this_round(10,false);
    std::vector<float> offsets_x(10);
    std::vector<float> offsets_y(10);

    if(colors_found[0].found) {
        //sees something blue
        //do we see a cube?
        if(cube_found) {
            occurances_in_a_row[1] += 1;
            if(occurances_in_a_row[1] >= occurances_thresh)
                found_this_round[1] = true;
        } else {
            //blue triangle
            occurances_in_a_row[7] += 1;
            if(occurances_in_a_row[7] >= occurances_thresh)
                found_this_round[7] = true;
        }
    } else {
        occurances_in_a_row[1] = 0;
        occurances_in_a_row[7] = 0;
    }

    if(colors_found[1].found) {
        //sees something green
        //do we see a cube?
        if(cube_found) {
            occurances_in_a_row[2] += 1;
            if(occurances_in_a_row[2] >= occurances_thresh)
                found_this_round[2] = true;
        } else {
            //cylinder
            occurances_in_a_row[6] += 1;
            if(occurances_in_a_row[6] >= occurances_thresh)
                found_this_round[6] = true;
        }
    } else {
        occurances_in_a_row[2] = 0;
        occurances_in_a_row[6] = 0;
    }

    if(colors_found[2].found) {
        //sees something red
        //do we see a cube?
        if(cube_found) {
            occurances_in_a_row[0] += 1;
            if(occurances_in_a_row[0] >= occurances_thresh)
                found_this_round[0] = true;
        } else {
            //sphere
            occurances_in_a_row[5] += 1;
            if(occurances_in_a_row[5] >= occurances_thresh)
                found_this_round[5] = true;
        }
    }  else {
        occurances_in_a_row[0] = 0;
        occurances_in_a_row[5] = 0;
    }

    if(colors_found[3].found) {
        //sees something yellow
        //do we see a cube?
        if(cube_found) {
            occurances_in_a_row[3] += 1;
            if(occurances_in_a_row[3] >= occurances_thresh)
                found_this_round[3] = true;
        } else {
            //sphere
            occurances_in_a_row[4] += 1;
            if(occurances_in_a_row[4] >= occurances_thresh)
                found_this_round[4] = true;
        }
    } else {
        occurances_in_a_row[3] = 0;
        occurances_in_a_row[4] = 0;
    }

    if(colors_found[4].found) {
        if(!cube_found) {
            occurances_in_a_row[9] += 1;
            if(occurances_in_a_row[9] >= occurances_thresh)
                found_this_round[9] = true;
        }
    } else {
        occurances_in_a_row[9] = 0;
    }

    if(colors_found[5].found) {
        occurances_in_a_row[8] += 1;
        if(occurances_in_a_row[8] >= occurances_thresh)
            found_this_round[8] = true;
    } else {
        occurances_in_a_row[8] = 0;
    }

    for(int i = 0; i < found_this_round.size(); ++i) {
        if(found_this_round[i] && !objects_found[i]) {
            objects_found[i] = true;
            vision_master::object_found msg;
            msg.id = object_names[i];
            std::vector<float> offsets;
            int pixel_row = colors_found[color_mappings[i]].row;
            int pixel_col = colors_found[color_mappings[i]].col;
            float depth = colors_found[color_mappings[i]].depth;
            //getRelativePosition()
            //getRelativePosition()

            offsets = getRelativePosition(480,640,pixel_row, pixel_col,depth);
            msg.offset_x = offsets[0];
            msg.offset_y = offsets[1];
            master_publisher.publish(msg);
            ROS_INFO_STREAM("FOUND OBJECT: " << object_names[i] << " Offsets: ("
                            << offsets[0] << "," << offsets[1] << ")");
        }
    }
}

std::vector<float> VisionMasterNode::getRelativePosition(
        int camera_res_h,
        int camera_res_w,
        int pixel_row, int pixel_col, float depth) {

    std::vector<float> p(2);

    //float h_omega = camera_rotation_x - camera_res_h/2 + ((float)pixel_row)*(camera_fov_h/camera_res_h);
    float w_omega = ((float) pixel_col)*(camera_fov_w/camera_res_w) - camera_res_w/2;
    //h_omega = std::abs(h_omega);
    //float d_g = std::cos((h_omega*M_PI) / 180)*depth;
    float d_g = std::sqrt(std::pow(depth,2)-std::pow(camera_offset_z,2));

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

    cube_found = false;
    //colors_found = std::vector<bool>(6,false);
    colors_found = std::vector<color_detection::color_status>(6);
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


    //general ros setup
    t_color = ros::Time::now();
    t_depth = ros::Time::now();
    color_subscriber = handle.subscribe("/vision/color_classifier", 1, &VisionMasterNode::colorCallback, this);
    depth_subscriber = handle.subscribe("/vision/cube_identifier", 1, &VisionMasterNode::depthCallback, this);
    master_publisher = handle.advertise<vision_master::object_found>("/vision/detection", 100);
    return handle;
}

VisionMasterNode::VisionMasterNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[]) {
    VisionMasterNode vmn(argc, argv);
}
