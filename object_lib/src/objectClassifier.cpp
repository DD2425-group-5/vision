#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <string>
#include <rosutil/rosutil.hpp>
#include <sysutil/sysutil.hpp>
#include <cmath>

using namespace cv;
using std::string;
using std::cout;
using std::endl;
using std::vector;
using ROSUtil::getParam;
using namespace cv;

class ModelParams {
public:
    ModelParams(){};
    ModelParams(string name, float mu_r, float mu_g, float mu_b, float std_r, float std_g, float std_b) {
	modelName = name;
	mu_b = mu_b;
	mu_g = mu_g;
	mu_r = mu_r;
	std_b = std_b;
	std_g = std_g;
	std_r = std_r;
    }
    string modelName;
    float mu_b;
    float mu_g;
    float mu_r;
    float std_b;
    float std_g;
    float std_r;
};


ModelParams backgroundModel;

float gauss(float x, float mu, float sigma)
{
    return pow(x - mu, 2)/(2*pow(sigma,2));
}

/**
 * class_prob = log(prior) - sum(log(sigmas)) - sum(((x-mu)^2)/2sigma^2)
 */
float discriminant(float b, float g, float r, float prior, ModelParams model)
{
    float sigmasum = log(model.std_b) + log(model.std_g) + log(model.std_r);
    float musum = gauss(b, model.mu_b, model.std_b)
	+ gauss(g, model.mu_g, model.std_g)
	+ gauss(r, model.mu_r, model.std_r);
    
    float res = log(prior) - sigmasum - musum;
    return res;
}

void classifyImages(SysUtil::DirContents dir, ModelParams model) {
    Mat im;
    for (size_t i = 0; i < dir.files.size(); i++) {
	cout << dir.files[i] << endl;
	im = imread(dir.files[i], CV_LOAD_IMAGE_COLOR);
	
	// Probably very inefficient. Lots of splitting and stuff
	// Use a LUT for the gaussian computations on the model. Only 
	// 255 possible values exist for the discriminant for any given model.
	resize(im, im, Size(), 0.2, 0.2);
	Mat c = Mat::zeros(im.rows, im.cols, CV_32F);
	
	for (int row = 0; row < im.rows; row++) {
	    for (int col = 0; col < im.cols; col++) {
		Vec3b pixel = im.at<Vec3b>(row, col);
		c.at<float>(row, col) = discriminant(pixel.val[0], pixel.val[1], pixel.val[2], 0.5, model);
	    }
	}

	double max, min;
	minMaxIdx(c, &min, &max);
	float minm = min - min;
	float maxm = max - min;
	Mat gray = Mat::zeros(c.rows, c.cols, CV_32F);
	cout << minm << ", " << maxm << endl;
	cout << c.rows << ", " << c.cols << endl;
	for (int row = 0; row < gray.rows; row++) {
	    for (int col = 0; col < gray.cols; col++) {
		gray.at<float>(row, col) = (((c.at<float>(row, col) - min)/maxm));
	    }
	}

	minMaxIdx(gray, &min, &max);

	cout << "Min value: " << min << ", max value: " << max << endl;
	namedWindow("class", CV_WINDOW_AUTOSIZE);
	imshow("class", gray);
	waitKey(0);
    }
}

ModelParams readModel(string modelName, ros::NodeHandle n) {
    ModelParams model;
    model.modelName = modelName;
    getParam(n, "/models/" + modelName + "/mu_b", model.mu_b);
    getParam(n, "/models/" + modelName + "/mu_g", model.mu_g);
    getParam(n, "/models/" + modelName + "/mu_r", model.mu_r);
    getParam(n, "/models/" + modelName + "/std_b", model.std_b);
    getParam(n, "/models/" + modelName + "/std_g", model.std_g);
    getParam(n, "/models/" + modelName + "/std_r", model.std_r);
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
    getParam(n, "/models/model_names", names);
    for (vector<string>::iterator it = names.begin(); it != names.end(); it++) {
	// want to separate out the background model and object models
	if ((*it).compare("background")){
	    backgroundModel = readModel(*it, n);
	    continue;
	}
	models.push_back(readModel(*it, n));
    }

    vector<SysUtil::DirContents> dirsToProcess = SysUtil::getDirContents(argv[1]);
    for (size_t i = 0; i < dirsToProcess.size(); i++) {
	cout << dirsToProcess[i].path << endl;
	classifyImages(dirsToProcess[i], models[0]);
    }

    return 0;
}
