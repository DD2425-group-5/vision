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

    if(colors_found[0]) {
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

    if(colors_found[1]) {
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

    if(colors_found[2]) {
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

    if(colors_found[3]) {
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

    if(colors_found[4]) {
        if(!cube_found) {
            occurances_in_a_row[9] += 1;
            if(occurances_in_a_row[9] >= occurances_thresh)
                found_this_round[9] = true;
        }
    } else {
        occurances_in_a_row[9] = 0;
    }

    if(colors_found[5]) {
        occurances_in_a_row[8] += 1;
        if(occurances_in_a_row[8] >= occurances_thresh)
            found_this_round[8] = true;
    } else {
        occurances_in_a_row[8] = 0;
    }

    for(int i = 0; i < found_this_round.size(); ++i) {
        if(found_this_round[i] && !objects_found[i]) {
            objects_found[i] = true;
            std_msgs::String msg;
            msg.data = object_names[i];
            master_publisher.publish(msg);
            ROS_INFO_STREAM("FOUND OBJECT: " << object_names[i]);
        }
    }





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

ros::NodeHandle VisionMasterNode::nodeSetup(int argc, char *argv[]) {
    ros::init(argc, argv, "VisionMaster");
    ros::NodeHandle handle;

    cube_found = false;
    colors_found = std::vector<bool>(6,false);
    objects_found = std::vector<bool>(10,false);
    occurances_in_a_row = std::vector<int>(10,0);

    occurances_thresh = 10;
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

    //general ros setup
    t_color = ros::Time::now();
    t_depth = ros::Time::now();
    color_subscriber = handle.subscribe("/vision/color_classifier", 1, &VisionMasterNode::colorCallback, this);
    depth_subscriber = handle.subscribe("/vision/cube_identifier", 1, &VisionMasterNode::depthCallback, this);
    master_publisher = handle.advertise<std_msgs::String>("/vision/detection", 100);
    return handle;
}

VisionMasterNode::VisionMasterNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[]) {
    VisionMasterNode vmn(argc, argv);
}
