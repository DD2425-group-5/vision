#include "cubeidentifiernode.hpp"

void CubeIdentifierNode::coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg) {
    t_coeff = ros::Time::now();
    p_coeff = msg;
}

void CubeIdentifierNode::pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    t_pc = ros::Time::now();
    p_pc = msg;
}

void CubeIdentifierNode::update() {

}

ros::NodeHandle CubeIdentifierNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "CubeIdentifier");
    ros::NodeHandle handle;

    pc_subscriber = handle.subscribe("/plane_extraction/plane_removed", 1,
                                         &CubeIdentifierNode::pcCallback, this);
    coeffs_subscriber = handle.subscribe("/plane_extraction/plane_coefficients", 1,
                                         &CubeIdentifierNode::coeffsCallback, this);

    return handle;
}

void CubeIdentifierNode::runNode(ros::NodeHandle handle) {
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


CubeIdentifierNode::CubeIdentifierNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[])
{
    CubeIdentifierNode cin(argc, argv);
}
