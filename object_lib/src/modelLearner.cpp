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
#include <visionutil/visionmodels.hpp>

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

void modelsToYAML(color_model_vardim<double> model, std::string output){
    using namespace YAML;

    Emitter out;
    out << BeginMap;
        out << Key << "models";
        out << Value;
        out << BeginMap;
            out << Key << "purpleModel";
            out << Value;
            out << BeginMap;
                out << Key << "mu";
                out << Value;
                out << BeginSeq;
                for(int i = 0; i < model.dim; ++i)
                    out << model.mu[i];
                out << EndSeq;
                out << Key << "sigma";
                out << Value;
                out << BeginSeq;
                for(int i = 0; i < model.dim; ++i)
                    for(int j = 0; j < model.dim; ++j)
                        out << model.sigma[i][j];
                out << EndSeq;
            out << EndMap;
        out << EndMap;
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

    vector<Vec2d> pixels;
    for (size_t dirNum = 0; dirNum < content.size(); dirNum++) {
        SysUtil::DirContents dc = content[dirNum];
        cout << content[dirNum].path << endl;

        cout << SysUtil::removeBaseName(content[dirNum].path) << endl;
        cout << SysUtil::removeBaseName(content[dirNum].path).compare("background")<< endl;

        for (size_t fileNum = 0; fileNum < dc.files.size(); fileNum++) {

            Mat img = imread(dc.files[fileNum], CV_LOAD_IMAGE_COLOR);
            Mat img8;
            img.convertTo(img8,CV_8UC3); //not sure if needed


            Mat resized;
            resize(img8, resized, Size(), 0.2, 0.2);
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

            for(int row = 0; row < resized.rows; ++row) {
                for(int col = 0; col < resized.cols; ++col) {
                    if(mask.at<unsigned char>(row,col)) {
                        Vec3b pixel = resized.at<Vec3b>(row,col);
                        double r = pixel.val[2];
                        double g = pixel.val[1];
                        double b = pixel.val[0];
                        double intensity = r+b+g;
                        pixels.push_back(Vec2d(r/intensity,g/intensity));
                    }
                }
            }
        }
    }

    //calculate mean
    double mean_r = 0;
    double mean_g = 0;
    double n = pixels.size();
    color_model_vardim<double> model(2);

    for(long i = 0; i < pixels.size();++i) {
        mean_r += pixels[i].val[0];
        mean_g += pixels[i].val[1];
    }

    mean_r /= n;
    mean_g /= n;

    model.mu[0] = mean_r;
    model.mu[1] = mean_g;

    //calculate sigma
    //http://en.wikipedia.org/wiki/Sample_mean_and_sample_covariance#Sample_covariance
    //can be sped up because sigma[j][k] == sigma[k][j]
    for(int j = 0; j < 2; ++j) {
        for(int k = 0; k < 2; ++k) {
            double sum = 0;
            for(int i = 0; i < n; ++i) {
                sum+= (pixels[i].val[j] - model.mu[j]) * (pixels[i].val[k] - model.mu[k]);
            }
            model.sigma[j][k] = sum / (n-1);
        }
    }

    //model is done
    //write to file
    modelsToYAML(model, outfile);


    /*
	int size = dc.files.size();
	// Dividing by scalars is screwy, so do it manually instead.
	cout << "Final mean: " << meanSum << " Final stdev: " << stdevSum << endl;
	Scalar fmean = Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size);
	Scalar fstd = Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size);
	cout << "Final mean: " << Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size)
	     << " Final stdev: " << Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size) << endl;
	modelMap.insert(pair<string, pair<Scalar, Scalar> >(SysUtil::removeBaseName(dc.path) ,pair<Scalar,Scalar>(fmean,fstd)));
    */

    //modelsToYAML(modelMap, outfile);
    return 0;
}
