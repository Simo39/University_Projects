#include "BoWCreator.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <stdexcept>
#include <iostream>

using namespace cv;

BoWCreator::BoWCreator(std::string path) {
	utils::fs::glob(path, "*.jpg", file_list);
	if (file_list.size() == 0)
		throw std::runtime_error("The folder doesn't contain any .jpg file");
}

template <typename T> Mat BoWCreator::createBoW(Ptr<T>& detector) {
	Mat descriptors = computeDescriptors(detector);
	return createDictionary(descriptors);
}
Mat BoWCreator::createBoW() {
	Ptr<SIFT> detector = SIFT::create();
	return createBoW(detector);
};

template <typename T> Mat BoWCreator::computeDescriptors(Ptr<T>& detector) {
	double mean = 0;
	int number_images = file_list.size();
	Mat descriptors;
	for (std::string file : file_list) {
		Mat img = imread(file);
		std::vector<KeyPoint> kp;
		Mat descriptor;
		detector->detectAndCompute(img, Mat(), kp, descriptor);
		descriptors.push_back(descriptor);
		if (clusters_number == 0) {
			mean += (kp.size() / number_images);
		}
	}
	if (clusters_number == 0)
		clusters_number = mean;
	return descriptors;
}

Mat BoWCreator::createDictionary(Mat descriptors) {
	BOWKMeansTrainer trainer(clusters_number);
	return trainer.cluster(descriptors);
}

void BoWCreator::setNumberOfClusters(int n) {
	clusters_number = n;
}

int BoWCreator::getNumberOfClusters() {
	return clusters_number;
}