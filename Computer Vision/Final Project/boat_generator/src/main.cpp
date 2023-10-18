
#include "debug_utils.h"
#include "FilteringTest.h"
#include "BoatDetection.h"
#include "Dictionary.h"
#include "Localization.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace cv;

const std::string INPUT_WINDOW_NAME = "Input";
const std::string RESULT_WINDOW_NAME = "Result";
const int WINDOW_FLAGS = WINDOW_NORMAL | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED;

std::vector<std::string> getImagesPath(std::string f);
std::vector<std::string> getGroundTruth(std::string f);
void testFilters();
Mat extractImages(std::vector<std::string> file_list);

int main(int argc, char** argv) {

	//testFilters();
	//return 0;

	std::string argument;
	std::vector<std::string> img_list;
	std::vector<std::string> ground_truth_txt;
	std::vector<std::string> pos,neg;
	try {
		if (argc != 2)
			throw std::invalid_argument("Not enough arguments");
		argument = (argv[1]);
		utils::fs::glob(argument,"ground_truth.txt", ground_truth_txt);
		utils::fs::glob(argument, "positive.yml", pos);
		utils::fs::glob(argument, "negative.yml", neg);
		if(ground_truth_txt.size() !=1 || pos.size() !=1 || neg.size() != 1)
			throw std::invalid_argument("invalid arguments");
	}
	catch (std::exception e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
		std::cerr << "1 parameters are requested: " << std::endl;
		std::cerr << "boat_detector.exe <data folder> " << std::endl;
		return -1;
	}

	//extraction of ground truth
	
	img_list = getImagesPath(ground_truth_txt[0]);
	std::vector<std::string> ground_truth = getGroundTruth(ground_truth_txt[0]);

	for (int i = 0; i < img_list.size(); i++) {
		Mat input = imread(argument + "\\" + img_list[i]);

		namedWindow(INPUT_WINDOW_NAME, WINDOW_FLAGS);
		imshow(INPUT_WINDOW_NAME, input);
		waitKey();

		Dictionary positive(pos[0]);
		Dictionary negative(neg[0]);


		Localization localizer(input);
		ClassificationBoat classificator(&positive, &negative);
		BoatDetection detector(&localizer, &classificator);

		Mat result = detector.getDetection();
		namedWindow(RESULT_WINDOW_NAME, WINDOW_FLAGS);
		imshow(RESULT_WINDOW_NAME, result);
		waitKey();

		detector.setGroundTruth(ground_truth[i]);
		result = detector.getImageWithScores();
		namedWindow(RESULT_WINDOW_NAME, WINDOW_FLAGS);
		imshow(RESULT_WINDOW_NAME, result);
		waitKey();


		std::cout << "Quality: %" << detector.getQualityScore() * 100 << std::endl;

		namedWindow(RESULT_WINDOW_NAME, WINDOW_FLAGS);
		imshow(RESULT_WINDOW_NAME, result);
		waitKey();
	}
	return 0;
}

std::vector<std::string> getImagesPath(std::string f) {
	std::ifstream file(f);
	std::string row;
	std::vector<std::string> paths;
	while (std::getline(file, row)) {
		paths.push_back(row);
	}
	file.close();
	std::vector<std::string> imgs_path;
	for (std::string path : paths) {
		imgs_path.push_back(path.substr(0, path.find(" ")));
	}
	return imgs_path;
}

std::vector<std::string> getGroundTruth(std::string f) {
	std::ifstream file(f);
	std::string row;
	std::vector<std::string> ground_truth;
	while (std::getline(file, row)) {
		ground_truth.push_back(row);
	}
	file.close();
	return ground_truth;
}

Mat extractImages(std::vector<std::string> file_list) {
	if (file_list.size() > 12) {
		std::cerr << "Too many images: MAX 12 images" << std::endl;
		throw -1;
	}

	Mat result;
	std::vector<Mat> row;
	float ratio = 0.8;
	Size size(400 * ratio, 300 * ratio);

	for (std::string file : file_list) {
		Mat temp = imread(file);
		resize(temp, temp, size);
		row.push_back(temp);
		hconcat(row, temp);
		if (row.size() == 3) {
			result.push_back(temp);
			row.clear();
		}
	}
	Mat last_row;
	while (row.size() < 3)
		row.push_back(Mat::zeros(size, result.type()));
	hconcat(row, last_row);
	result.push_back(last_row);
	
	return result;
}

void testFilters() {

	std::vector<std::string> kaggle_list;
	std::vector<std::string> venice_list;
	utils::fs::glob("C:\\Workspace\\CV project\\project\\data\\test\\kaggle", "*.jpg", kaggle_list);
	utils::fs::glob("C:\\Workspace\\CV project\\project\\data\\test\\venice", "*.png", venice_list);

	Mat imgs = extractImages(kaggle_list);
	testRegionProposal(imgs);

	//imgs = extractImages(venice_list);
	//testRegionProposal(imgs);
}



