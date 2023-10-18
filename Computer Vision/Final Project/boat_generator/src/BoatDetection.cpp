#include "debug_utils.h"
#include "BoatDetection.h"
#include "ClassificationBoat.h"
#include "Dictionary.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>

BoatDetection::BoatDetection(Localization* localizer, ClassificationBoat* classificator) {
	this->image = localizer->getInputImage();
	this->localizer = localizer;
	this->classificator = classificator;
	doDetection();
}

std::vector<Rect> BoatDetection::getBoats() {
	return boats;
}
Mat BoatDetection::getDetection() {
	Mat result = image.clone();
	Scalar color_detection(0, 0, 255);
	for (Rect boat : boats)
		drawRegion(result, boat, color_detection);
	return result;
}
void BoatDetection::setGroundTruth(std::vector<Rect> ground_truth) {
	this->ground_truth = ground_truth;
	computeGTScores();
	computeRegionsScores();
	computeQuality();
}
void BoatDetection::setGroundTruth(std::string path) {
	int pos = path.rfind(".") + 5;
	int next = path.find(" ", pos);
	std::string temp = path.substr(pos, next - pos);
	int num_rect = std::stoi(temp);
	pos = next + 1;
	std::vector<Rect> rects;
	for (int i = 0; i < num_rect; i++) {
		std::vector<int> values;
		for (int j = 0; j < 4; j++) {
			next = path.find(" ", pos);
			if (next == std::string::npos)
				next = path.size();
			std::string temp = path.substr(pos, next - pos);
			values.push_back(std::stoi(temp));
			pos = next + 1;
		}
		rects.push_back(Rect(values[0], values[1], values[2], values[3]));
	}
	setGroundTruth(rects);
}

std::vector<double> BoatDetection::getGroundTruthScores() {
	return ground_truth_scores;
	
}
std::vector<double> BoatDetection::getRegionsScores() {
	return regions_scores;
}
double BoatDetection::getQualityScore() {
	return quality;
}
Mat BoatDetection::getImageWithScores() {
	Mat result = image.clone(); 
	Scalar color_GT(0, 255, 0);
	Scalar color_zero_score_region(0, 0, 255);
	Scalar color_region(255, 0, 0);
	for (int i = 0; i < boats.size(); i++) {
		Scalar color = color_region;
		if (regions_scores[i] == 0)
			color = color_zero_score_region;
		drawRegion(result, boats[i], color);
	}
	Scalar color_best_match = color_GT + color_region;
	for (int i = 0; i < ground_truth.size(); i++) {
		drawRegion(result, ground_truth[i], color_GT, ground_truth_scores[i]);
		drawRegion(result, best_matches[i], color_best_match);
	}
	return result;
}

void BoatDetection::doDetection() {
	//pre processing
	localizer->doLocalization();
	std::vector<Rect> regions = localizer->getRegions();
	//classification step
	std::vector<Rect> boat_regions;
	for (Rect region : regions) {
		bool isBoat = classificator->isBoat(image, region);
		if (isBoat) {
			boat_regions.push_back(region);
		}
	}
	boats = boat_regions;
}

void BoatDetection::drawRegion(Mat& image, Rect region, Scalar color, double score) {
	rectangle(image, region, color, 2);
	if (score != 0) {
		double font_scale = 0.5;
		int thickness = 2;
		std::string text = "score: " + std::to_string(score);
		Point p(region.x, region.y - 5);
		putText(image, text, p, FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
	}
}

double BoatDetection::IoU(const Rect& lhs, const Rect& rhs){
	double i = static_cast<double>((lhs & rhs).area());
	double u = static_cast<double>((lhs).area()) + static_cast<double>((rhs).area()) - i;
	return i / u;
}

double BoatDetection::getRegionScore(std::vector<Rect> ground_truth, Rect region, Rect& best_match) {
	std::vector<double> scores;
	double best_score = 0;
	for (Rect GT_region : ground_truth) {
		double score = IoU(GT_region, region);
		if (best_score < score) {
			best_score = score;
			best_match = GT_region;
		}
	}
	return best_score;
}
void BoatDetection::computeRegionsScores() {
	std::vector<double> scores;
	double best_score = 0;
	for (Rect boat : boats) {
		Rect r(0, 0, 0, 0);
		double score = getRegionScore(ground_truth, boat, r);
		scores.push_back(score);
	}
	regions_scores = scores;
}
void BoatDetection::computeGTScores() {
	std::vector<double> scores;
	double best_score = 0;
	for (Rect gt_region : ground_truth) {
		Rect r(0, 0, 0, 0);
		double score = getRegionScore(boats, gt_region, r);
		scores.push_back(score);
		best_matches.push_back(r);
	}
	ground_truth_scores = scores;
}
void BoatDetection::computeQuality() {
	quality = 0;
	for (double score : ground_truth_scores)
		quality += score;
	quality /= (regions_scores.size());
}