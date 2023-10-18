#pragma once
#include "ClassificationBoat.h"
#include "Localization.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;

class BoatDetection {
public:
	BoatDetection(Localization* localizer, ClassificationBoat* classificator);

	std::vector<Rect> getBoats();
	Mat getDetection();
	void setGroundTruth(std::vector<Rect> ground_truth);
	void setGroundTruth(std::string path);
	std::vector<double> getGroundTruthScores();
	std::vector<double> getRegionsScores();
	double getQualityScore();
	Mat getImageWithScores();
	
protected:
	Mat image;
	Localization* localizer;
	ClassificationBoat* classificator;
	std::vector<Rect> boats;
	std::vector<Rect> ground_truth;
	std::vector<Rect> best_matches;
	std::vector<double> regions_scores;
	std::vector<double> ground_truth_scores;
	double quality = -1;
	bool show_steps;


	BoatDetection();
	void doDetection();
	void drawRegion(Mat& image, Rect region, Scalar color, double score = 0.0);

	double IoU(const Rect& lhs, const Rect& rhs);

	double getRegionScore(std::vector<Rect> ground_truth, Rect region, Rect& best_match);
	void computeRegionsScores();
	void computeGTScores();
	void computeQuality();
};