#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <string>
#include <rosutil/rosutil.hpp>

using namespace cv;
using std::string;
using std::cout;
using std::vector;
using ROSUtil::getParam;

using namespace cv;

class ModelParams {
public:
    ModelParams(){};
    ModelParams(string name, double mu_r, double mu_g, double mu_b, double std_r, double std_g, double std_b) {
	modelName = name;
	mu_b = mu_b;
	mu_g = mu_g;
	mu_r = mu_r;
	std_b = std_b;
	std_g = std_g;
	std_r = std_r;
    }
    string modelName;
    double mu_b;
    double mu_g;
    double mu_r;
    double std_b;
    double std_g;
    double std_r;
};

void classifyImages(string dirName) {
    Mat test;
    
}

ModelParams readModel(string modelName, ros::NodeHandle n) {
    ModelParams model;
    model.modelName = modelName;
    getParam(n, "/object_models/" + modelName + "/mu_b", model.mu_b);
    getParam(n, "/object_models/" + modelName + "/mu_g", model.mu_g);
    getParam(n, "/object_models/" + modelName + "/mu_r", model.mu_r);
    getParam(n, "/object_models/" + modelName + "/std_b", model.std_b);
    getParam(n, "/object_models/" + modelName + "/std_g", model.std_g);
    getParam(n, "/object_models/" + modelName + "/std_r", model.std_r);
    return model;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
	cout << "Please provide a directory from which to read images to classify.";
    }

    ros::init(argc, argv, "object_classifier");
    ros::NodeHandle n;
    
    
    vector<ModelParams> models;
    vector<string> names;
    getParam(n, "/object_models/model_names", names);
    for (vector<string>::iterator it = names.begin(); it != names.end(); it++) {
	models.push_back(readModel(*it, n));
    }

    classifyImages(argv[1]);

    return 0;
}
