#include <ros/ros.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <sysutil/sysutil.hpp>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <stdlib.h>

using namespace cv;
using std::cout;
using std::string;
using std::vector;
using std::endl;
using std::pair;
using std::map;

static const string help = "Model learner - learns basic models from images by computing mean and standard deviation of colour channels.\n\n"\
"USAGE\n"\
"\t\t rosrun object_lib model_learner [OPTIONS] input_dir\n"\
"OPTIONS\n"\
"\t -b \t Indicates that the directories to be processed contain background images\n"\
"\t\t and that no thresholding should be used to extract black pixels.\n\n"\
"\t -o \t File into which to output the YAML file containing model parameters.";

void modelsToYAML(map<string, pair<Scalar,Scalar> > modelMap, std::string output){
    using namespace YAML;
    Emitter out;
    out << BeginMap;
    out << Key << "models";
    out << Value;
    out << BeginMap;
    map<string, pair<Scalar,Scalar> >::iterator it;
    vector<string> modelNames;
    for (it = modelMap.begin(); it != modelMap.end(); ++it) {
	modelNames.push_back(it->first);
    	string name = it->first;
    	pair<Scalar, Scalar> model = it->second;
	out << Key << name.c_str();
    	out << Value;
    	out << BeginMap;
    	out << Key << "mu_b";
    	out << Value << model.first.val[0];
    	out << Key << "mu_g";
    	out << Value << model.first.val[1];
    	out << Key << "mu_r";
    	out << Value << model.first.val[2];
    	out << Key << "std_b";
    	out << Value << model.second.val[0];
    	out << Key << "std_g";
    	out << Value << model.second.val[1];
    	out << Key << "std_r";
    	out << Value << model.second.val[2];
    	out << EndMap;
    }

    out << Key << "model_names";
    out << Value << Flow << modelNames;
    out << EndMap;

    cout << out.c_str() << endl;

    std::ofstream of;
    of.open(output.c_str());
    of << out.c_str();
    of.close();
}



int main(int argc, char *argv[]) {
    if (argc < 2){
	cout << "You must provide an argument with the directory to process." << endl;
	cout << help << endl;
	std::exit(1);
    }

    
    bool bg; // background - process without thresholding to extract the object.
    string outfile = "modelparams.yaml"; // default output
    int c;
    
    while((c = getopt(argc, argv, "bo:")) != -1){
	switch (c){
	case 'b': 
	    bg = true;
	    break;
	case 'o':
	    // user can provide output location for YAML
	    outfile = string(optarg);
	    break;
	case '?':
	    cout << "Don't know option " << optopt << endl;
	    std::exit(1);
	}
    }

    std::string dirName(argv[optind]);
    
    vector<SysUtil::DirContents> content = SysUtil::getDirContents(dirName);
    map<string, pair<Scalar,Scalar> > modelMap;
    for (size_t dirNum = 0; dirNum < content.size(); dirNum++) {
	SysUtil::DirContents dc = content[dirNum];
	cout << content[dirNum].path << endl;

	// Keep track of the mean and stdeviation averages for all images
	Scalar meanSum, stdevSum;

	cout << SysUtil::removeBaseName(content[dirNum].path) << endl;
	cout << SysUtil::removeBaseName(content[dirNum].path).compare("background")<< endl;
	for (size_t fileNum = 0; fileNum < dc.files.size(); fileNum++) {
//	    cout << dc.files[fileNum] << endl;
	    
	    Mat img = imread(dc.files[fileNum], CV_LOAD_IMAGE_COLOR);


	    Mat resized;
	    resize(img, resized, Size(), 0.2, 0.2);
	    // The default mask is to take everything in the image
	    Mat mask = Mat::ones(resized.rows, resized.cols, CV_8UC1);
	    // Only threshold if bg switch not set and the directory name is not "background".
	    if (!bg && SysUtil::removeBaseName(content[dirNum].path).compare("background") != 0) {
		// Use a threshold to get a mask in order to
		// extract the relevant pixels from the image,
		// if the image is for an object
		Mat gray;
		cvtColor(resized, gray, CV_RGB2GRAY);
		threshold(gray, mask, 0, 255, THRESH_BINARY | THRESH_OTSU);
	    }
	    
	    // namedWindow("d", CV_WINDOW_AUTOSIZE);
	    // imshow("d", m);
	    // waitKey(0);

	    // for this image
	    Scalar mean, stdev;
	    meanStdDev(resized, mean, stdev, mask);
//	    cout << "Mean: " << mean << "Stdev: " << stdev << endl;
	    meanSum = meanSum + mean;
	    stdevSum = stdevSum + stdev;
	}
	int size = dc.files.size();
	// Dividing by scalars is screwy, so do it manually instead.
	cout << "Final mean: " << meanSum << " Final stdev: " << stdevSum << endl;
	Scalar fmean = Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size);
	Scalar fstd = Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size);
	cout << "Final mean: " << Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size)
	     << " Final stdev: " << Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size) << endl;
	modelMap.insert(pair<string, pair<Scalar, Scalar> >(SysUtil::removeBaseName(dc.path) ,pair<Scalar,Scalar>(fmean,fstd)));
    }

    modelsToYAML(modelMap, outfile);
    return 0;
}
