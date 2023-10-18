#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>

using namespace cv;

class BoWCreator {
public:
	/*
	* Constructor
	* 
	* It takes only the path to the folder containing all the images to extract
	*/
	BoWCreator(std::string folder_path);

	template <typename T> Mat createBoW(Ptr<T>& detector);
	Mat createBoW();
	
	void setNumberOfClusters(int n);
	int getNumberOfClusters();

private:

	std::vector<std::string> file_list;
	int clusters_number = 0;

	Ptr<SIFT> SIFT_detector = SIFT::create();

	template <typename T> Mat computeDescriptors(Ptr<T>& detector);
	Mat createDictionary(Mat descriptors);

};