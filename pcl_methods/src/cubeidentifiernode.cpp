#include "cubeidentifiernode.hpp"

/*void CubeIdentifierNode::coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg) {
    t_coeff = ros::Time::now();
    p_coeff = msg;
}

void CubeIdentifierNode::pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg) {
    t_pc = ros::Time::now();
    p_pc = msg;
}*/

void CubeIdentifierNode::planeCallback(const vision_msgs::plane_extracted::ConstPtr& msg) {
    t_plane = ros::Time::now();
    p_plane = msg;
}

void CubeIdentifierNode::colorCallback(const vision_msgs::colors_detected::ConstPtr& msg) {
    t_color = ros::Time::now();
    c_colors = msg;
}

void CubeIdentifierNode::readParams(ros::NodeHandle& n) {

    ROSUtil::getParam(n, "cube_identifier_params/theta", theta);
    ROSUtil::getParam(n, "cube_identifier_params/search_radius", distance_search_radius);
    ROSUtil::getParam(n, "cube_identifier_params/num_vectors", num_similar_vectors_thresh);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_x", voxel_grid_x);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_y", voxel_grid_y);
    ROSUtil::getParam(n, "cube_identifier_params/voxel_grid_z", voxel_grid_z);
    ROSUtil::getParam(n, "cube_identifier_params/check_patric_cube", check_patric_cube);
    ROSUtil::getParam(n, "cube_identifier_params/check_purple_cube", check_purple_cube);
    ROSUtil::getParam(n, "cube_identifier_params/size_around_color", size_around_color);


    ROS_INFO("Successfully read params in cube_identifier.");
    ROS_INFO_STREAM("Theta: " << theta);
    ROS_INFO_STREAM("Distanse search radius: " << distance_search_radius);
    ROS_INFO_STREAM("Numbers of parallel vectors needed: " << num_similar_vectors_thresh);
    ROS_INFO_STREAM("Voxel grid size: ("
                    << voxel_grid_x << "," << voxel_grid_y << "," << voxel_grid_z << ")");
}

/**
Downsamples the pointcloud given and puts it into the output cloud. Uses a voxel grid
of size (voxel_grid_x, voxel_grid_y, voxel_grid_z), which should have been read as
parameters.
*/
void CubeIdentifierNode::downSample(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) {
    //pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZRGB> > sor;
    pcl::VoxelGrid<pcl::PointXYZRGB > sor;
    sor.setInputCloud (input);
    sor.setLeafSize (voxel_grid_x, voxel_grid_y, voxel_grid_z);
    sor.filter (*output);
}

/**
Crops input, puts all points from the input pointcloud in the rectangle defined by
minRow,maxRow,minCol,maxCol into the output pointcloud.
*/
void CubeIdentifierNode::cropToArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, int minRow, int maxRow,
                                    int minCol, int maxCol) {

    for(int row = minRow; row <= maxRow; ++row) {
        for(int col = minCol; col <= maxCol; ++col) {
            if(input->at(col,row).z >= 0 )
                output->push_back(input->at(col,row));
        }
    }
}

/**
Key method. The basic idea is to check wether the two vectors reference and other are
"parallel" enough. Basically check the angle between them, and if its small enough
(around5 degrees) they are deemed parallel. The angle is checked with the dot product.
Specifically, the angle theta between two vectors v and u is:

cos(theta) = (v*u)/(|v|*|u|)

However, doing these calculations for every point is slow. First, v, the reference
vector, is not going to change in the frame. So |v| can be precomputed. Also, we want
to check if

theta < thresh_deg
where theta = arccos((v*u)/(|v|*|u|))
Thus thresh_deg > arccos((v*u)/(|v|*|u|))

This is eqvivalent to checking

cos(thresh_deg) < (v*u)/(|v|*|u|)

And because |v| could be precomputed:

cos(thresh_deg)*|v| < (v*u)/ |u|

the left side can be completely precomputed, but the right will haveto be done
for every pixel. min_thresh (the input) is the left side of this equation.
*/
bool CubeIdentifierNode::closeEnough(const pcl::Normal &reference, const pcl::Normal &other,
                                     float min_thresh) {
    //using other variables because their name is shorter.
    float ux,uy,uz,vx,vy,vz;

    //note: make sure the vector points up, similar to the plane normal.
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
    The full non-optimized calculations can be seen here:
    float top = ux*vx+uy*vy+uz*vz;
    float bot = std::sqrt(ux*ux+uy*uy+uz*uz)*std::sqrt(vx*vx+vy*vy+vz*vz);
    float angle = std::acos(top/bot);
    float indegrees = (angle*180)/M_PI;

    return indegrees < 5;
*/
}

void CubeIdentifierNode::update() {
    vision_msgs::colors_with_shape_info msg;

    for(int i = 0; i < 6; ++i) {
        msg.data[i].color.found = false;
    }

 /*   if((ros::Time::now()-t_coeff).toSec()>1.0) {
        cube_publisher.publish(msg);
        return;
    }*/
    if((ros::Time::now()-t_plane).toSec()>1.0) {
        cube_publisher.publish(msg);
        return;
    }
    if((ros::Time::now()-t_color).toSec()>0.5) {
        cube_publisher.publish(msg);
        return;
    }

    //initialize message
    msg.data[0].color = c_colors->blue;
    msg.data[1].color = c_colors->green;
    msg.data[2].color = c_colors->red;
    msg.data[3].color = c_colors->yellow;
    msg.data[4].color = c_colors->orange;
    msg.data[5].color = c_colors->purple;

    /*
        std::vector<std::pair<int,int> > pois(6,std::pair<int,int>(-1,-1)); //Point Of InterestS
        for(int i = 0; i < 6; ++i) {
            if(pois[i].color.found)
                pois[i] = std::pair<int,int>(c_colors->green.row,c_colors->green.col);
        }


        if(c_colors->blue.found)
            pois[0] = std::pair<int,int>(c_colors->blue.row,c_colors->blue.col);
        if(c_colors->green.found)
            pois[1] = std::pair<int,int>(c_colors->green.row,c_colors->green.col);
        if(c_colors->red.found)
            pois[2] = std::pair<int,int>(c_colors->red.row,c_colors->red.col);
        if(c_colors->yellow.found)
            pois[3] = std::pair<int,int>(c_colors->yellow.row,c_colors->yellow.col);
        if(c_colors->orange.found)
            pois[4] = std::pair<int,int>(c_colors->orange.row,c_colors->orange.col);
        if(c_colors->purple.found)
            pois[5] = std::pair<int,int>(c_colors->purple.row,c_colors->purple.col);
    */
    //plane normal is the plane's coefficients.
    //ROS_INFO("CHECK 1");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>());
    //ROS_INFO("CHECK 2");
    pcl::fromROSMsg(p_plane->plane_removed_org,*cloud_org);
    //ROS_INFO("CHECK 3");
    pcl::Normal normal(p_plane->plane_eq.values[0],p_plane->plane_eq.values[1], p_plane->plane_eq.values[2]);
    //ROS_INFO("CHECK 4");


    //make sure the normal points up.
    if(normal.normal_y < 0) {
        normal.normal_x = -normal.normal_x;
        normal.normal_y = -normal.normal_y;
        normal.normal_z = -normal.normal_z;
    }

    for(int i = 0; i < 6; ++i) {
        if(msg.data[i].color.found) {
            if(!check_patric_cube && i == 4)
                continue;
            if(!check_purple_cube && i == 5)
                continue;

            //downsample
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sampled(new pcl::PointCloud<pcl::PointXYZRGB> ());
            //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c3po = p_pc;

            int minrow = std::max(-size_around_color+msg.data[i].color.row,0);
            int maxrow = std::min(size_around_color+msg.data[i].color.row,
                                  (int)p_plane->plane_removed_org.height-1);
            int mincol = std::max(-size_around_color+msg.data[i].color.col,0);
            int maxcol = std::min(size_around_color+msg.data[i].color.col,
                                  (int)p_plane->plane_removed_org.width-1);
            cropToArea(cloud_org,down_sampled,minrow,maxrow,mincol,maxcol);

            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

            ne.setInputCloud (down_sampled);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset
            // (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

            ne.setSearchMethod (tree);

            //ROS_INFO("KD_TREES SET");

            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);


            // Use all neighbors in a sphere of radius whatever is given
            ne.setRadiusSearch (distance_search_radius);

            // Computes the normals for each point
            ne.compute (*cloud_normals);

            //precalculate the "left side" of the closeEnough comparison
            //see comments for that function.
            //close enough thresh is the cosine part already computed
            //that is done in nodeSetup.
            float min_thresh = close_enough_thresh *
                    std::sqrt(std::pow(normal.normal_x,2)+
                              std::pow(normal.normal_y,2)+
                              std::pow(normal.normal_z,2));

            int numClose = 0;

            for(int j = 0; j < cloud_normals->size(); ++j) {
                if(closeEnough(normal,(*cloud_normals)[j],min_thresh)) {
                    ++numClose;
                }
            }

            ROS_INFO_STREAM("NUM CLOSE "<< i << ": " << numClose << " Cloudsize: " << down_sampled->size());

            if(numClose >= num_similar_vectors_thresh) {
                msg.data[i].cube = true;
            } else {
                msg.data[i].cube = false;
            }
        }
    }

    cube_publisher.publish(msg);
}

ros::NodeHandle CubeIdentifierNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "CubeIdentifier");
    ros::NodeHandle handle;

    readParams(handle);

    close_enough_thresh = std::cos((theta*M_PI) / 180);

    /*pc_subscriber = handle.subscribe("/plane_extraction/plane_removed", 1,
                                         &CubeIdentifierNode::pcCallback, this);
    coeffs_subscriber = handle.subscribe("/plane_extraction/plane_coefficients", 1,
                                         &CubeIdentifierNode::coeffsCallback, this);*/
    plane_subscriber = handle.subscribe("/vision/plane_extraction",10,
                                        &CubeIdentifierNode::planeCallback, this);
    color_subscriber = handle.subscribe("/vision/color_classifier", 10,
                                         &CubeIdentifierNode::colorCallback, this);
    cube_publisher = handle.advertise<vision_msgs::colors_with_shape_info>("/vision/cube_identifier",5);

    return handle;
}

void CubeIdentifierNode::runNode(ros::NodeHandle handle) {
    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
    for(int i = 0; i < 10; ++i)
        loop_rate.sleep();


    while(ros::ok()) {
        ros::spinOnce();
        update();
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
