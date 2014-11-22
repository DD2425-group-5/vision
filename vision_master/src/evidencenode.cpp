#include "evidencenode.hpp"

void EvidenceNode::objectCallback(const std_msgs::String::ConstPtr &msg) {
    speak(msg->data);
    
    //send message to /evidence
    ras_msgs::RAS_Evidence ev_msg;
    ev_msg.stamp = ros::Time::now();
    ev_msg.group_number = 5;
    ev_msg.image_evidence = *camera_img_raw;
    ev_msg.object_id = msg->data;
    evidence_publisher.publish(ev_msg);
    
}

void ColorDetectionNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_rgb = ros::Time::now();
    camera_img_raw = msg;
}

void EvidenceNode::speak(std::string object_id) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "I see a " << object_id;
    msg.data = ss.str();
    sound_publisher.publish(msg);
}

EvidenceNode::EvidenceNode(int argc, char* argv[]) {
	ros::init(argc, argv, "Evidence");
	ros::NodeHandle handle;
    object_subscriber = handle.subscribe("/vision/detection", 10, &EvidenceNode::objectCallback, this);
    rgb_subscriber = handle.subscribe("/camera/rgb/image_rect_color", 1, &EvidenceNode::rgbCallback, this);
    sound_publisher = handle.advertise<std_msgs::String>("/espeak/string", 10);
    evidence_publisher = handle.advertise<ras_msgs::RAS_Evidence>("/evidence", 10);
    
    double control_frequency = 10.0;
    
    ros::Rate loop_rate(control_frequency);
    for(int i = 0; i < 10; ++i)
        loop_rate.sleep();


    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[]) {
    EvidenceNode en(argc, argv);
}