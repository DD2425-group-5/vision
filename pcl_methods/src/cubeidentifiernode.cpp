#include "cubeidentifiernode.hpp"

void CubeIdentifierNode::coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg) {
    t_coeff = ros::Time::now();
    p_coeff = msg;
}

void CubeIdentifierNode::pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg) {
    t_pc = ros::Time::now();
    p_pc = msg;
}

void CubeIdentifierNode::readParams(ros::NodeHandle& n) {

    ROSUtil::getParam(n, "cube_identifier_params/theta", theta);
    ROSUtil::getParam(n, "cube_identifier_params/search_radius", distance_search_radius);
    ROSUtil::getParam(n, "cube_identifier_params/num_vectors", num_similar_vectors_thresh);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_x", voxel_grid_x);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_y", voxel_grid_y);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_z", voxel_grid_z);

    ROS_INFO("Successfully read params in cube_identifier.");
    ROS_INFO_STREAM("Theta: " << theta);
    ROS_INFO_STREAM("Distanse search radius: " << distance_search_radius);
    ROS_INFO_STREAM("Numbers of parallel vectors needed: " << num_similar_vectors_thresh);
    ROS_INFO_STREAM("Voxel grid size: ("
                    << voxel_grid_x << "," << voxel_grid_y << "," << voxel_grid_z << ")");
}

void CubeIdentifierNode::downSample(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) {
    //pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZRGB> > sor;
    pcl::VoxelGrid<pcl::PointXYZRGB > sor;
    sor.setInputCloud (input);
    sor.setLeafSize (voxel_grid_x, voxel_grid_y, voxel_grid_z);
    sor.filter (*output);
}

bool CubeIdentifierNode::closeEnough(const pcl::Normal &reference, const pcl::Normal &other,
                                     float min_thresh) {
    float ux,uy,uz,vx,vy,vz;
    if(other.normal_y < 0) {
        vx = -other.normal_x;
        vy = -other.normal_y;
        vz = -other.normal_z;
    } else {
        vx = other.normal_x;
        vy = other.normal_y;
        vz = other.normal_z;
    }
    ux = reference.normal_x;
    uy = reference.normal_y;
    uz = reference.normal_z;

    float top = ux*vx+uy*vy+uz*vz;
    float bot = std::sqrt(vx*vx+vy*vy+vz*vz);
    return ((top/bot) >= min_thresh);


/*
    float top = ux*vx+uy*vy+uz*vz;
    float bot = std::sqrt(ux*ux+uy*uy+uz*uz)*std::sqrt(vx*vx+vy*vy+vz*vz);
    float angle = std::acos(top/bot);
    float indegrees = (angle*180)/M_PI;

    return indegrees < 5;
*/
}

void CubeIdentifierNode::update() {
    std_msgs::Bool msg;
    msg.data = false;

    if((ros::Time::now()-t_coeff).toSec()>1.0) {
        cube_publisher.publish(msg);
        return;
    }
    if((ros::Time::now()-t_pc).toSec()>1.0) {
        cube_publisher.publish(msg);
        return;
    }

    //ROS_INFO("BEGIN UPDATE");
    pcl::Normal normal(p_coeff->values[0],p_coeff->values[1], p_coeff->values[2]);
    //pcl::PointCloud& pc = *p_pc;

    //make sure the normal points up.
    if(normal.normal_y < 0) {
        normal.normal_x = -normal.normal_x;
        normal.normal_y = -normal.normal_y;
        normal.normal_z = -normal.normal_z;
    }

    //downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sampled(new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c3po = p_pc;
    //*c3po = *p_pc;

    //downSample(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr(p_pc),down_sampled);
    //downSample(p_pc,down_sampled);
    downSample(c3po,down_sampled);


    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    ne.setInputCloud (down_sampled);
    ROS_INFO_STREAM("SIZE BEFORE DOWNSAMPLING: " << p_pc->size() << " AFTER: " << down_sampled->size());

    //ROS_INFO("INITTED NORMAL ESTIMATION");

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset
    // (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());


    ne.setSearchMethod (tree);

    //ROS_INFO("KD_TREES SET");

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);


    // Use all neighbors in a sphere of radius 2cm
    ne.setRadiusSearch (distance_search_radius);

    // Compute the features
    ne.compute (*cloud_normals);
    //ROS_INFO("SEARCH_DONE");

    float min_thresh = close_enough_thresh *
            std::sqrt(std::pow(normal.normal_x,2)+
                      std::pow(normal.normal_y,2)+
                      std::pow(normal.normal_z,2));

    int numClose = 0;
    //std::vector<pcl::PointXYZ> parallel;

    for(int i = 0; i < cloud_normals->size(); ++i) {
        if(closeEnough(normal,(*cloud_normals)[i],min_thresh)) {
            ++numClose;
            //parallel.push_back(i);
            //parallel.push_back(pcl::PointXYZ(
            //                  (*down_sampled)[i].x,(*down_sampled)[i].y,(*down_sampled)[i].z);

        }
    }

    ROS_INFO_STREAM("NUM CLOSE: " << numClose);

    if(numClose >= num_similar_vectors_thresh) {
        msg.data = true;
    }


    cube_publisher.publish(msg);
/*

    if(parallel.size() >= num_similar_vectors_thresh) {
        for(int i = 0; i < parallel.size(); ++i) {

        }
    }
*/
    //if(numClose > 1000)
     //  ROS_INFO("SUSPECTING WE GOT A CUBE...");


    //ROS_INFO_STREAM("Num point input: " << p_pc->size() << " normals: " << cloud_normals->size());


}

ros::NodeHandle CubeIdentifierNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "CubeIdentifier");
    ros::NodeHandle handle;

    readParams(handle);

    close_enough_thresh = std::cos((theta*M_PI) / 180);



    pc_subscriber = handle.subscribe("/plane_extraction/plane_removed", 1,
                                         &CubeIdentifierNode::pcCallback, this);
    coeffs_subscriber = handle.subscribe("/plane_extraction/plane_coefficients", 1,
                                         &CubeIdentifierNode::coeffsCallback, this);
    cube_publisher = handle.advertise<std_msgs::Bool>("/vision/cube_identifier",5);

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
